# This file was automatically generated for projects
# without default 'CMakeLists.txt' file.

if(NOT DEFINED APP_DIR)
    set(APP_DIR "transmitter")
endif()

FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/${APP_DIR}/*.*)

idf_component_register(SRCS ${app_sources}
                        INCLUDE_DIRS "${CMAKE_SOURCE_DIR}/include"
                        REQUIRES nvs_flash)
