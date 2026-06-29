#include "wifi_server.h"
#include "hx711.h"
#include "ts_protocol.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_http_server.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include <inttypes.h>
#include <string.h>
#include <stdio.h>

static const char *TAG = "wifi_server";

// ─── HX711 pin configuration ──────────────────────────────────────────────────
#define HX711_CLK    GPIO_NUM_4
#define HX711_DATA0  GPIO_NUM_21  // Ch0 — DRDY ISR
#define HX711_DATA1  GPIO_NUM_5   // Ch1
#define HX711_DATA2  GPIO_NUM_26  // Ch2 (20 kg) — displayed
#define HX711_DATA3  GPIO_NUM_14  // Ch3 (20 kg) — displayed

// ─── Recording ring buffer ────────────────────────────────────────────────────
// 1000 packets × 26 bytes = 26 KB  →  ~12.5 s at 80 SPS.
#define RING_CAP 1000

static ts_espnow_packet_t  s_ring[RING_CAP];
static int                 s_ring_head  = 0;
static int                 s_ring_tail  = 0;
static int                 s_ring_count = 0;
static SemaphoreHandle_t   s_ring_mutex;
static volatile bool       s_collecting = false;

// ─── Latest packet (for /latest polling endpoint) ─────────────────────────────
static ts_espnow_packet_t  s_latest;
static SemaphoreHandle_t   s_latest_mutex;
static SemaphoreHandle_t   s_data_sem;       // signals SSE task on new reading

// ─── Shared state ─────────────────────────────────────────────────────────────
static hx711_bus_t        s_bus;
static uint16_t           s_seq = 0;
static httpd_handle_t     s_server = NULL;

static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONNECTED_BIT BIT0

// ─── Ring buffer ──────────────────────────────────────────────────────────────
static void ring_push(const ts_espnow_packet_t *pkt)
{
    if (!s_collecting) return;
    xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
    s_ring[s_ring_head] = *pkt;
    s_ring_head = (s_ring_head + 1) % RING_CAP;
    if (s_ring_count < RING_CAP) {
        s_ring_count++;
    } else {
        s_ring_tail = (s_ring_tail + 1) % RING_CAP;
    }
    xSemaphoreGive(s_ring_mutex);
}

// ─── 5-sample median filter (per channel) ────────────────────────────────────
// Rejects single-sample spikes caused by WiFi TX bursts / EMI without
// introducing the lag of a moving average.
#define MEDIAN_N 3

static int32_t s_med_buf[TS_MAX_CHANNELS][MEDIAN_N];
static int     s_med_idx = 0;
static bool    s_med_full = false;

static int cmp_i32(const void *a, const void *b)
{
    int32_t x = *(const int32_t *)a, y = *(const int32_t *)b;
    return (x > y) - (x < y);
}

static void median_push(const int32_t *in, int32_t *out, int nch)
{
    for (int c = 0; c < nch; c++)
        s_med_buf[c][s_med_idx] = in[c];

    s_med_idx = (s_med_idx + 1) % MEDIAN_N;
    if (s_med_idx == 0) s_med_full = true;

    int len = s_med_full ? MEDIAN_N : s_med_idx;
    int32_t tmp[MEDIAN_N];
    for (int c = 0; c < nch; c++) {
        memcpy(tmp, s_med_buf[c], sizeof(int32_t) * len);
        qsort(tmp, len, sizeof(int32_t), cmp_i32);
        out[c] = tmp[len / 2];
    }
}

// ─── TX task: reads HX711 on Core 1 ──────────────────────────────────────────
static void tx_task(void *pvParam)
{
    int32_t raw[TS_MAX_CHANNELS];
    int32_t filtered[TS_MAX_CHANNELS];
    ts_espnow_packet_t pkt = {
        .magic         = TS_ESPNOW_MAGIC,
        .channel_count = s_bus.channel_count,
    };

    while (1) {
        int n = hx711_read_all(&s_bus, raw);
        if (n == 0) continue;   // skip failed reads — don't poison median or ring
        median_push(raw, filtered, s_bus.channel_count);

        pkt.seq          = s_seq++;
        pkt.timestamp_us = (uint32_t)esp_timer_get_time();
        pkt.flags        = 0x00;
        memcpy(pkt.readings, filtered, sizeof(int32_t) * s_bus.channel_count);

        ring_push(&pkt);

        xSemaphoreTake(s_latest_mutex, portMAX_DELAY);
        s_latest = pkt;
        xSemaphoreGive(s_latest_mutex);
        xSemaphoreGive(s_data_sem);   // wake SSE streaming task
        // No delay — hx711_read_all blocks on DRDY, giving natural 80 Hz pacing.
    }
}

// ─── HTTP: POST /control — body: "start" | "stop" ────────────────────────────
static esp_err_t control_handler(httpd_req_t *req)
{
    char body[8];
    int len = httpd_req_recv(req, body, sizeof(body) - 1);
    if (len <= 0) {
        httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "no body");
        return ESP_FAIL;
    }
    body[len] = '\0';

    if (strncmp(body, "start", 5) == 0) {
        xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
        s_ring_head  = 0;
        s_ring_tail  = 0;
        s_ring_count = 0;
        s_collecting = true;
        xSemaphoreGive(s_ring_mutex);
    } else if (strncmp(body, "stop", 4) == 0) {
        s_collecting = false;
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"ok\":true}");
    return ESP_OK;
}

// ─── HTTP: GET /status — ring buffer count + collecting state ─────────────────
static esp_err_t status_handler(httpd_req_t *req)
{
    xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
    int count = s_ring_count;
    xSemaphoreGive(s_ring_mutex);

    char json[64];
    snprintf(json, sizeof(json),
             "{\"count\":%d,\"cap\":%d,\"collecting\":%s}",
             count, RING_CAP, s_collecting ? "true" : "false");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, json);
    return ESP_OK;
}

// ─── HTTP: GET /events (Server-Sent Events push stream) ──────────────────────
// Runs in its own task so the httpd worker is freed for /latest and /data.
static void sse_stream_task(void *arg)
{
    httpd_req_t *req = (httpd_req_t *)arg;
    char buf[160];

    // Initial comment lets the browser confirm the connection immediately.
    if (httpd_resp_send_chunk(req, ": connected\n\n", HTTPD_RESP_USE_STRLEN) != ESP_OK)
        goto done;

    while (1) {
        // Wait up to 5 s for a new reading; send a keepalive comment on timeout
        // so the browser doesn't close a stale connection.
        if (xSemaphoreTake(s_data_sem, pdMS_TO_TICKS(5000)) != pdTRUE) {
            if (httpd_resp_send_chunk(req, ": ka\n\n", HTTPD_RESP_USE_STRLEN) != ESP_OK)
                break;
            continue;
        }

        xSemaphoreTake(s_latest_mutex, portMAX_DELAY);
        ts_espnow_packet_t p = s_latest;
        xSemaphoreGive(s_latest_mutex);

        int len = snprintf(buf, sizeof(buf),
            "data:{\"seq\":%" PRIu16 ",\"nch\":%" PRIu8 ",\"flags\":%" PRIu8 ","
            "\"ch\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "]}\n\n",
            p.seq, p.channel_count, p.flags,
            p.readings[0], p.readings[1], p.readings[2], p.readings[3]);

        if (httpd_resp_send_chunk(req, buf, len) != ESP_OK)
            break;
    }

done:
    httpd_resp_send_chunk(req, NULL, 0);
    httpd_req_async_handler_complete(req);
    vTaskDelete(NULL);
}

static esp_err_t events_handler(httpd_req_t *req)
{
    httpd_req_t *async_req;
    if (httpd_req_async_handler_begin(req, &async_req) != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "async failed");
        return ESP_FAIL;
    }
    httpd_resp_set_type(async_req, "text/event-stream");
    httpd_resp_set_hdr(async_req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(async_req, "X-Accel-Buffering", "no");
    xTaskCreate(sse_stream_task, "sse", 4096, async_req, 5, NULL);
    return ESP_OK;
}

// ─── HTTP: GET / (startup calibration gate + dual fixed-axis graphs) ──────────
// Two-screen UI:
//   Screen 1 (div#cal): tare + span calibration for all 4 channels.
//                        scale persists in localStorage; tare always required.
//   Screen 2 (div#live): two fixed-axis canvases (Ch0+1 = 20 kg, Ch2+3 = 5 kg).
//
// Calibration math (matches calibrate.py):
//   offset  = mean(tare raw counts)
//   scale   = known_force / (mean(loaded) - offset)   [N/count or Nm/count]
//   display = (raw - offset) * scale
static const char s_root_html[] =
    "<!DOCTYPE html><html><head><meta charset=utf-8><title>ThrustStand</title><link rel=preconnect href='https://fonts.googleapis.com'><link href='https://fonts.googleapis.com/css2?family=Space+Grotesk:wght@300;400;800&display=swap' rel=stylesheet><link rel=icon type='image/jpeg' href='data:image/jpeg;base64,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'><style>*{box-sizing:border-box;margin:0;padding:0}body{font-family:'Space Grotesk',sans-serif;background:#fff;color:#0d0d0d;max-width:960px;margin:0 auto;padding:36px 40px}.hdr{display:flex;align-items:center;justify-content:space-between;padding-bottom:16px}.brand{display:flex;align-items:center;gap:12px}.brand img{height:40px;width:auto}.brand-text{display:flex;flex-direction:column}.brand-name{font-weight:800;font-size:18px;line-height:1}.brand-sub{font-weight:300;font-size:11px;letter-spacing:.28em;text-transform:uppercase;color:#7a7a7a;margin-top:3px}.badge{display:flex;align-items:center;gap:8px;border:1px solid;padding:6px 14px;font-size:10px;font-weight:400;letter-spacing:.2em;text-transform:uppercase}.badge--idle{border-color:#e2e2e2;color:#7a7a7a}.badge--active{border-color:#f05a00;color:#f05a00}.badge--stopped{border-color:#0d0d0d;color:#0d0d0d}.badge-dot{width:7px;height:7px;border-radius:50%;background:currentColor}.badge--idle .badge-dot{opacity:.3}.badge--active .badge-dot{animation:pulse 1s ease-in-out infinite}@keyframes pulse{0%,100%{opacity:1}50%{opacity:.2}}.rule{height:2px;background:#f05a00;margin-bottom:28px}.sh{display:flex;align-items:center;gap:10px;margin:28px 0 16px}.sh::before{content:'';display:inline-block;width:0;height:0;border-top:6px solid transparent;border-bottom:6px solid transparent;border-left:8px solid #f05a00}.sh-text{font-weight:800;font-size:14px}.card-grid{background:#e2e2e2;display:flex;flex-direction:column;gap:1px}.card{background:#fff;padding:20px 24px}.btn-group{display:flex;margin-top:20px}.btn{padding:9px 18px;background:#fff;color:#0d0d0d;border:1px solid #0d0d0d;border-right:none;font-family:'Space Grotesk',sans-serif;font-size:10px;font-weight:400;letter-spacing:.2em;text-transform:uppercase;cursor:pointer;display:inline-flex;align-items:center;line-height:1}.btn:last-child{border-right:1px solid #0d0d0d}.btn:hover:not(:disabled){background:#0d0d0d;color:#fff}.btn--primary{border-color:#f05a00;color:#f05a00}.btn--primary:last-child{border-right:1px solid #f05a00}.btn--primary:hover:not(:disabled){background:#f05a00;color:#fff}.btn:disabled{border-color:#e2e2e2;color:#e2e2e2;cursor:default}.btn-link{text-decoration:none}table{border-collapse:collapse;width:100%;font-size:12px;margin-top:12px}td,th{padding:10px 12px;border:1px solid #e2e2e2}th{font-weight:400;font-size:10px;letter-spacing:.2em;text-transform:uppercase;color:#7a7a7a}input[type=number]{width:76px;background:#fff;color:#0d0d0d;border:1px solid #e2e2e2;padding:4px 6px;font-family:'Space Grotesk',sans-serif;font-size:12px}input[type=number]:focus{outline:1px solid #0d0d0d;outline-offset:-1px}.info{font-size:10px;font-weight:300;letter-spacing:.1em;color:#7a7a7a;margin-top:3px}#vals{display:flex;gap:40px;margin-bottom:16px}.v{font-weight:800;font-size:22px;font-variant-numeric:tabular-nums}#st{font-size:10px;font-weight:300;letter-spacing:.2em;color:#7a7a7a;margin:8px 0;text-transform:uppercase}canvas{max-width:100%;display:block;background:#fff;border:1px solid #e2e2e2;margin:12px 0}.desc{font-size:12px;font-weight:300;color:#7a7a7a;line-height:1.7;margin-bottom:16px}#live{display:none}</style></head><body><div class=hdr><div class=brand><img src='data:image/jpeg;base64,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' alt=Avol><div class=brand-text><span class=brand-name>Avol</span><span class=brand-sub>ThrustStand DAQ</span></div></div><div id=badge class='badge badge--idle'><span class=badge-dot></span><span id=badge-lbl>Idle</span></div><span id=rcnt style='font-size:10px;font-weight:300;letter-spacing:.2em;color:#7a7a7a;margin-left:16px'></span></div><div class=rule></div><div id=cal><div class=sh><span class=sh-text>Startup Calibration</span></div><p class=desc>Remove all load. Tare each channel (3 s), then apply a known mass and Calibrate (3 s). Scales persist in browser; tare is required each power-on.</p><div class=card-grid><div class=card><table><tr><th>Ch</th><th>1. Tare (no load)</th><th>Mass&nbsp;(kg)</th><th>Arm&nbsp;(m)&nbsp;&mdash;&nbsp;0&nbsp;=&nbsp;N</th><th>2. Calibrate</th><th>Status</th></tr><tr><td>Ch2 &mdash; 20&nbsp;kg</td><td><button class=btn onclick='tare(0,this)'>Tare 3s</button></td><td><input type=number id=m0 value=1 min=0.001 step=0.001></td><td><input type=number id=a0 value=0 min=0 step=0.001></td><td><button class=btn onclick='calSpan(0,this)'>Cal 3s</button></td><td><span id=s0>Tare first</span><br><span class=info id=i0></span></td></tr><tr><td>Ch3 &mdash; 20&nbsp;kg</td><td><button class=btn onclick='tare(1,this)'>Tare 3s</button></td><td><input type=number id=m1 value=1 min=0.001 step=0.001></td><td><input type=number id=a1 value=0 min=0 step=0.001></td><td><button class=btn onclick='calSpan(1,this)'>Cal 3s</button></td><td><span id=s1>Tare first</span><br><span class=info id=i1></span></td></tr></table></div></div><div class=btn-group><button class='btn btn--primary' id=startBtn onclick='startMonitoring()' disabled>Start Monitoring</button><button class=btn onclick='bypass()'>Skip &mdash; Raw Counts</button></div></div><div id=live><div id=vals><span class=v id=v0>Ch2: ---</span><span class=v id=v1 style=color:#f05a00>Ch3: ---</span><span class=v id=v2 style='display:none;color:#0066cc'>Sum: ---</span></div><div class=sh style=justify-content:space-between><span class=sh-text>20&nbsp;kg Cells &mdash; Ch&nbsp;2 &amp; 3</span><button class=btn id=modeBtn onclick='toggleParallel()' style=margin-left:auto>Mode: Independent</button></div><canvas id=cv0></canvas><div id=st>Waiting for data...</div><div class=btn-group><button class=btn onclick='recal()'>Recalibrate</button><button class='btn btn--primary' id=startColBtn onclick='startCollection()'>Start Collection</button><button class=btn id=stopColBtn onclick='stopCollection()' disabled>Stop Collection</button><button class=btn id=dlBtn onclick='downloadCSV()' disabled>Download CSV</button></div></div><script>const GRAV=9.80665;const CH_MAX_KG=[20,20];const C=['#0D0D0D','#F05A00'];const CH_LABEL=['Ch2','Ch3'];const CH_IDX=[2,3];const N=500,W=900,H=220,ML=58;let calData={};try{calData=JSON.parse(localStorage.getItem('ts_cal')||'{}');}catch(e){calData={};}let taredOK=[false,false];let buf=[],fc=0,t0=Date.now(),lastSeq=-1,es=null,statusTimer=null,parallel=false;const cv0=document.getElementById('cv0');const g0=cv0.getContext('2d');cv0.width=W;cv0.height=H;function setBadge(s){const b=document.getElementById('badge');const l=document.getElementById('badge-lbl');b.className='badge badge--'+s;l.textContent={idle:'Idle',active:'Recording',stopped:'Stopped'}[s]||s;}function saveCal(){localStorage.setItem('ts_cal',JSON.stringify(calData));}function applyC(i,r){const c=calData['c'+i];return c&&c.scale!=null?(r-c.offset)*c.scale:r;}function unitOf(i){const c=calData['c'+i];return c&&c.scale!=null?c.unit:'cts';}function showInfo(i){const c=calData['c'+i];document.getElementById('i'+i).textContent=c&&c.scale!=null?'off='+c.offset.toFixed(0)+' sc='+c.scale.toExponential(2)+' '+c.unit:'';}for(let i=0;i<2;i++)showInfo(i);function checkStart(){const ok=taredOK.every((v,i)=>v&&calData['c'+i]&&calData['c'+i].scale!=null);document.getElementById('startBtn').disabled=!ok;}async function sampleAvg(ch,sec,statusEl){const s=[];const end=Date.now()+sec*1000;let last=-1,dots=0;while(Date.now()<end){await new Promise(r=>setTimeout(r,50));try{const p=await(await fetch('/latest')).json();if(p.seq!==last){last=p.seq;s.push(p.ch[CH_IDX[ch]]);}if(statusEl&&!(dots++%4))statusEl.textContent=statusEl.textContent.replace(/ \\[.*/,'')+' ['+s.length+' samples]';}catch(e){if(statusEl)statusEl.textContent='Fetch error: '+e.message;await new Promise(r=>setTimeout(r,200));}}return s.length?s.reduce((a,b)=>a+b,0)/s.length:null;}async function tare(i,btn){btn.disabled=true;const el=document.getElementById('s'+i);el.textContent='Taring 3s - remove all load...';const offset=await sampleAvg(i,3,el);if(offset===null){el.textContent='No data - check connection';btn.disabled=false;return;}if(!calData['c'+i])calData['c'+i]={};calData['c'+i].offset=offset;calData['c'+i].scale=null;taredOK[i]=true;saveCal();showInfo(i);checkStart();el.textContent='Tared. off='+offset.toFixed(0)+'. Apply load and Cal.';btn.disabled=false;}async function calSpan(i,btn){const mass=parseFloat(document.getElementById('m'+i).value);const arm=parseFloat(document.getElementById('a'+i).value)||0;const el=document.getElementById('s'+i);if(!(mass>0)){el.textContent='Enter mass > 0';return;}if(!taredOK[i]){el.textContent='Tare first!';return;}btn.disabled=true;el.textContent='Calibrating 3s - hold load steady...';const loaded=await sampleAvg(i,3,el);if(loaded===null){el.textContent='No data';btn.disabled=false;return;}const delta=loaded-calData['c'+i].offset;if(Math.abs(delta)<50){el.textContent='Delta <50 cts - check wiring/load';btn.disabled=false;return;}const force=mass*GRAV;const known=arm>0?force*arm:force;calData['c'+i].scale=known/delta;calData['c'+i].unit=arm>0?'Nm':'N';calData['c'+i].maxN=CH_MAX_KG[i]*GRAV*(arm>0?arm:1);saveCal();showInfo(i);checkStart();el.textContent='OK '+(known/delta).toExponential(3)+' '+calData['c'+i].unit+'/ct';btn.disabled=false;}function startMonitoring(){document.getElementById('cal').style.display='none';document.getElementById('live').style.display='block';setBadge('active');startSSE();startStatusPoll();}function bypass(){for(let i=0;i<2;i++)calData['c'+i]={offset:0,scale:1,unit:'cts',maxN:1000000};document.getElementById('cal').style.display='none';document.getElementById('live').style.display='block';setBadge('active');startSSE();startStatusPoll();}function recal(){if(es){es.close();es=null;}clearInterval(statusTimer);statusTimer=null;fetch('/control',{method:'POST',body:'stop'});document.getElementById('startColBtn').disabled=false;document.getElementById('stopColBtn').disabled=true;document.getElementById('dlBtn').disabled=true;document.getElementById('rcnt').textContent='';taredOK=[false,false];for(let i=0;i<2;i++){calData['c'+i]={offset:null,scale:null,unit:null,maxN:null};document.getElementById('s'+i).textContent='Tare first';document.getElementById('i'+i).textContent='';}saveCal();checkStart();setBadge('idle');document.getElementById('live').style.display='none';document.getElementById('cal').style.display='block';}function startCollection(){fetch('/control',{method:'POST',body:'start'});document.getElementById('startColBtn').disabled=true;document.getElementById('stopColBtn').disabled=false;document.getElementById('dlBtn').disabled=true;}function stopCollection(){fetch('/control',{method:'POST',body:'stop'});document.getElementById('stopColBtn').disabled=true;document.getElementById('dlBtn').disabled=false;}async function downloadCSV(){const name=prompt('Filename:','thruststand')||'thruststand';const resp=await fetch('/data');const raw=await resp.text();let hdr='# ThrustStand DAQ Export\\n';hdr+='# Formula: force = (raw_count - offset) * scale\\n';for(let i=0;i<2;i++){const c=calData['c'+i];if(c&&c.scale!=null)hdr+='# '+CH_LABEL[i]+' (col ch'+CH_IDX[i]+'): offset='+c.offset.toFixed(0)+', scale='+c.scale.toExponential(4)+', unit='+c.unit+'\\n';}const blob=new Blob([hdr+raw],{type:'text/csv'});const url=URL.createObjectURL(blob);const a=document.createElement('a');a.href=url;a.download=name+'.csv';a.click();URL.revokeObjectURL(url);}function startStatusPoll(){clearInterval(statusTimer);statusTimer=setInterval(async function(){try{const s=await(await fetch('/status')).json();const el=document.getElementById('rcnt');if(s.collecting)el.textContent=s.count+' / '+s.cap+' samples';else if(s.count>0)el.textContent=s.count+' samples (stopped)';else el.textContent='';}catch(e){}},1000);}function toggleParallel(){parallel=!parallel;document.getElementById('modeBtn').textContent=parallel?'Mode: Parallel':'Mode: Independent';document.getElementById('v2').style.display=parallel?'':'none';}function drawG(ctx,chs){const GW=W-ML-8;let maxN=0;chs.forEach(ci=>{const c=calData['c'+ci];maxN=Math.max(maxN,c&&c.maxN?c.maxN:CH_MAX_KG[ci]*GRAV);});if(parallel){let s=0;chs.forEach(ci=>{const c=calData['c'+ci];s+=c&&c.maxN?c.maxN:CH_MAX_KG[ci]*GRAV;});maxN=s;}const yMin=-0.05*maxN,yMax=1.05*maxN,yR=yMax-yMin;ctx.fillStyle='#fff';ctx.fillRect(0,0,W,H);ctx.font='10px monospace';ctx.textAlign='right';[0,0.25,0.5,0.75,1].forEach(f=>{const v=f*maxN;const py=H-4-(v-yMin)/yR*(H-8);ctx.strokeStyle='#e2e2e2';ctx.lineWidth=1;ctx.beginPath();ctx.moveTo(ML,py);ctx.lineTo(W-4,py);ctx.stroke();ctx.fillStyle='#7a7a7a';ctx.fillText(v.toFixed(1),ML-4,py+3);});const u=unitOf(chs[0]);ctx.save();ctx.translate(10,H/2);ctx.rotate(-Math.PI/2);ctx.textAlign='center';ctx.fillStyle='#7a7a7a';ctx.font='10px monospace';ctx.fillText(u,0,0);ctx.restore();if(buf.length<2)return;chs.forEach(ci=>{const view=buf.slice(-N);const len=view.length;ctx.beginPath();ctx.strokeStyle=C[ci];ctx.lineWidth=1.5;view.forEach((s,i)=>{const x=ML+(len>1?i/(len-1):0)*GW;const v=applyC(ci,s.ch[ci]);const py=H-4-(v-yMin)/yR*(H-8);i?ctx.lineTo(x,py):ctx.moveTo(x,py);});ctx.stroke();});if(parallel){const view=buf.slice(-N);const len=view.length;ctx.beginPath();ctx.strokeStyle='#0066cc';ctx.lineWidth=2.5;view.forEach((s,i)=>{const x=ML+(len>1?i/(len-1):0)*GW;const v=applyC(0,s.ch[0])+applyC(1,s.ch[1]);const py=H-4-(v-yMin)/yR*(H-8);i?ctx.lineTo(x,py):ctx.moveTo(x,py);});ctx.stroke();}}function draw(){drawG(g0,[0,1]);}requestAnimationFrame(function loop(){requestAnimationFrame(loop);draw();});function startSSE(){if(es)es.close();es=new EventSource('/events');es.onmessage=function(e){const p=JSON.parse(e.data);if(p.seq!==lastSeq){lastSeq=p.seq;buf.push({seq:p.seq,ch:[p.ch[CH_IDX[0]],p.ch[CH_IDX[1]]]});if(buf.length>N)buf.shift();for(let i=0;i<2;i++){const v=p.ch[CH_IDX[i]],u=unitOf(i);const disp=u==='cts'?v:applyC(i,v).toFixed(3)+' '+u;document.getElementById('v'+i).textContent=CH_LABEL[i]+': '+disp;}if(parallel){const s0=applyC(0,p.ch[CH_IDX[0]]),s1=applyC(1,p.ch[CH_IDX[1]]);document.getElementById('v2').textContent='Sum: '+(s0+s1).toFixed(3)+' '+unitOf(0);}fc++;if(!(fc%20))document.getElementById('st').textContent='seq='+p.seq+' | '+((fc*1000/(Date.now()-t0))|0)+' Hz (SSE)';}};es.onerror=function(){document.getElementById('st').textContent='SSE disconnected';setBadge('stopped');};}</script></body></html>";

static esp_err_t root_handler(httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, s_root_html, HTTPD_RESP_USE_STRLEN);
}

// ─── HTTP: GET /latest (JSON, polled by browser at 20 Hz) ────────────────────
static esp_err_t latest_handler(httpd_req_t *req)
{
    xSemaphoreTake(s_latest_mutex, portMAX_DELAY);
    ts_espnow_packet_t p = s_latest;
    xSemaphoreGive(s_latest_mutex);

    char json[128];
    snprintf(json, sizeof(json),
             "{\"seq\":%" PRIu16 ",\"nch\":%" PRIu8 ",\"flags\":%" PRIu8 ","
             "\"ch\":[%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "]}",
             p.seq, p.channel_count, p.flags,
             p.readings[0], p.readings[1], p.readings[2], p.readings[3]);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// ─── HTTP: GET /data (ring buffer CSV download) ───────────────────────────────
static esp_err_t data_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/csv");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "attachment; filename=\"thruststand.csv\"");

    httpd_resp_sendstr_chunk(req, "seq,timestamp_us,flags,ch0,ch1,ch2,ch3\r\n");

    xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
    int count = s_ring_count;
    int tail  = s_ring_tail;
    xSemaphoreGive(s_ring_mutex);

    char line[96];
    for (int i = 0; i < count; i++) {
        xSemaphoreTake(s_ring_mutex, portMAX_DELAY);
        ts_espnow_packet_t p = s_ring[(tail + i) % RING_CAP];
        xSemaphoreGive(s_ring_mutex);

        snprintf(line, sizeof(line),
                 "%" PRIu16 ",%" PRIu32 ",%" PRIu8
                 ",%" PRId32 ",%" PRId32 ",%" PRId32 ",%" PRId32 "\r\n",
                 p.seq, p.timestamp_us, p.flags,
                 p.readings[0], p.readings[1], p.readings[2], p.readings[3]);
        httpd_resp_sendstr_chunk(req, line);
    }

    httpd_resp_sendstr_chunk(req, NULL);
    ESP_LOGI(TAG, "/data: streamed %d rows", count);
    return ESP_OK;
}

// ─── HTTP server startup ──────────────────────────────────────────────────────
static const httpd_uri_t s_uri_root = {
    .uri     = "/",
    .method  = HTTP_GET,
    .handler = root_handler,
};

static const httpd_uri_t s_uri_latest = {
    .uri     = "/latest",
    .method  = HTTP_GET,
    .handler = latest_handler,
};

static const httpd_uri_t s_uri_data = {
    .uri     = "/data",
    .method  = HTTP_GET,
    .handler = data_handler,
};

static const httpd_uri_t s_uri_events = {
    .uri     = "/events",
    .method  = HTTP_GET,
    .handler = events_handler,
};

static const httpd_uri_t s_uri_control = {
    .uri     = "/control",
    .method  = HTTP_POST,
    .handler = control_handler,
};

static const httpd_uri_t s_uri_status = {
    .uri     = "/status",
    .method  = HTTP_GET,
    .handler = status_handler,
};

static void start_http_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port       = 80;
    cfg.keep_alive_enable   = true;
    cfg.keep_alive_idle     = 5;
    cfg.keep_alive_interval = 5;
    cfg.keep_alive_count    = 3;
    cfg.max_open_sockets    = 7;
    cfg.lru_purge_enable    = true;

    ESP_ERROR_CHECK(httpd_start(&s_server, &cfg));
    httpd_register_uri_handler(s_server, &s_uri_root);
    httpd_register_uri_handler(s_server, &s_uri_latest);
    httpd_register_uri_handler(s_server, &s_uri_data);
    httpd_register_uri_handler(s_server, &s_uri_events);
    httpd_register_uri_handler(s_server, &s_uri_control);
    httpd_register_uri_handler(s_server, &s_uri_status);

    ESP_LOGI(TAG, "HTTP server listening on port %d", cfg.server_port);
}

// ─── WiFi event handler ───────────────────────────────────────────────────────
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();

    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi disconnected — reconnecting...");
        esp_wifi_connect();

    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "──────────────────────────────────────────");
        ESP_LOGI(TAG, "IP address : " IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "Open: http://" IPSTR, IP2STR(&e->ip_info.ip));
        ESP_LOGI(TAG, "──────────────────────────────────────────");
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

// ─── Public init ──────────────────────────────────────────────────────────────
void wifi_server_init(const char *ssid, const char *password)
{
    s_ring_mutex   = xSemaphoreCreateMutex();
    s_latest_mutex = xSemaphoreCreateMutex();
    s_data_sem     = xSemaphoreCreateBinary();
    s_wifi_eg      = xEventGroupCreate();

    // HX711 (DRDY ISR requires GPIO ISR service to already be installed)
    s_bus = (hx711_bus_t){
        .clk_pin       = HX711_CLK,
        .data_pins     = {HX711_DATA0, HX711_DATA1, HX711_DATA2, HX711_DATA3},
        .channel_count = HXL_CHANNEL_COUNT,
    };
    hx711_bus_init(&s_bus);

    // WiFi STA
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    wifi_config_t wcfg = {};
    strncpy((char *)wcfg.sta.ssid,     ssid,     sizeof(wcfg.sta.ssid)     - 1);
    strncpy((char *)wcfg.sta.password, password, sizeof(wcfg.sta.password) - 1);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(40));  // 10 dBm (~1/4 of max 20 dBm)

    ESP_LOGI(TAG, "Connecting to SSID: %s ...", ssid);
    xEventGroupWaitBits(s_wifi_eg, WIFI_CONNECTED_BIT,
                        pdFALSE, pdTRUE, portMAX_DELAY);

    start_http_server();

    // Acquisition task pinned to Core 1 — isolates HX711 bit-bang from WiFi.
    xTaskCreatePinnedToCore(tx_task, "tx_task", 4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "ThrustStand ready");
}
