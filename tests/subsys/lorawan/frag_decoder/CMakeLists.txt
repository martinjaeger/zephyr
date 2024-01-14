# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lorawan)

# Generate include files from binary firmware files
# (see tests/application_development/gen_inc_file for details)
set(gen_dir ${ZEPHYR_BINARY_DIR}/include/generated/)
generate_inc_file_for_target(app
        data/helloworld-stm32wl.signed.bin
        ${gen_dir}/fw_raw.bin.inc
)
generate_inc_file_for_target(app
        data/helloworld-stm32wl_with_fragments.signed.bin
        ${gen_dir}/fw_coded.bin.inc
)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
