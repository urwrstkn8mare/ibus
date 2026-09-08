# Copyright 2024 Zorxx Software. All rights reserved.
if(IDF_TARGET)
    # sys_esp.h is a public header and includes the i2c and uart driver
    # headers, so those have to be REQUIRES rather than PRIV_REQUIRES —
    # otherwise anything including ibus.h fails to find them. Naming the
    # specific drivers rather than the "driver" shim keeps the dependency
    # honest about what is actually used.
    idf_component_register(SRCS "lib/ibus.c" "lib/sys/esp-idf.c"
                           INCLUDE_DIRS "include"
                           PRIV_INCLUDE_DIRS "lib" "include/ibus"
                           REQUIRES "esp_driver_i2c" "esp_driver_uart"
                           PRIV_REQUIRES "esp_timer")
    return()
endif()

cmake_minimum_required(VERSION 3.5)
set(project ibus)
project(${project} LANGUAGES C VERSION 1.0.0)

add_library(ibus STATIC lib/ibus.c lib/sys/linux.c)
target_compile_definitions(ibus PRIVATE SYS_DEBUG_ENABLE=1)
target_include_directories(ibus PUBLIC include)
target_include_directories(ibus PRIVATE lib include/ibus)
install(TARGETS ibus LIBRARY DESTINATION lib)
install(DIRECTORY include/ibus DESTINATION include)

add_subdirectory(examples/linux)
