# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(ares_native_sim_knob_haptic)

# 手感算法与示例放在一起，测试直接复用同一份源码
set(KNOB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../samples/motor/dji_m3508_demo/src)

target_include_directories(app PRIVATE ${KNOB_DIR})
target_sources(app PRIVATE src/main.c ${KNOB_DIR}/knob_haptic.c)
