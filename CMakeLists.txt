# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(adc_dt_1)

FILE(GLOB app_sources 
    src/*.c
    src/utilities/*.c
    src/utilities/libpqueue/src/*.c
    src/utilities/circularbuffer/*.c
    src/main.c
    src/app_io.c
    src/ble.c
    src/ems_pwm.c
)

target_sources(app PRIVATE ${app_sources})
