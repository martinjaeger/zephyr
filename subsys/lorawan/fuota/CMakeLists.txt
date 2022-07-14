# SPDX-License-Identifier: Apache-2.0

zephyr_library_sources_ifdef(CONFIG_LORAWAN_FUOTA
        clock_sync.c
        fuota.c
)
