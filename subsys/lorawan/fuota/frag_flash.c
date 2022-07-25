/*
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fuota.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fuota_frag_flash, CONFIG_LORAWAN_FUOTA_LOG_LEVEL);

int fuota_frag_flash_init(void)
{
	LOG_DBG("mass-erase flash");

	return 0;
}

int8_t fuota_frag_flash_write(uint32_t addr, uint8_t *data, uint32_t size)
{
	LOG_DBG("write %u bytes to addr 0x%x", size, addr);

	return 0;
}

int8_t fuota_frag_flash_read(uint32_t addr, uint8_t *data, uint32_t size)
{
	LOG_DBG("read %u bytes from addr 0x%x", size, addr);

	return 0;
}

void fuota_frag_flash_finish(void)
{
	/* ToDo: Finish flash writing and reboot? */

	LOG_DBG("frag decoder finish");
}
