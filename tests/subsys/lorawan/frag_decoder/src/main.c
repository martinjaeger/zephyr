/*
 * Copyright (c) 2024 A Labs GmbH
 * Copyright (c) 2024 tado GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/lorawan/emul.h>
#include <zephyr/sys/util.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/ztest.h>

static const uint8_t fw_raw[] = {
#include "fw_raw.bin.inc"
};

static const uint8_t fw_coded[] = {
#include "fw_coded.bin.inc"
};

#define FRAG_SIZE (232)
#define NB_FRAG   (DIV_ROUND_UP(sizeof(fw_raw), FRAG_SIZE))
#define PADDING   (NB_FRAG * FRAG_SIZE - sizeof(fw_raw))

#define CMD_FRAG_SESSION_SETUP (0x02)
#define CMD_DATA_FRAGMENT      (0x08)
#define FRAG_TRANSPORT_PORT    (201)
#define FRAG_SESSION_INDEX     (1)

#define TARGET_IMAGE_AREA FIXED_PARTITION_ID(slot1_partition)

static const struct flash_area *fa;

static struct k_sem fuota_finished_sem;

static void fuota_finished(void)
{
	k_sem_give(&fuota_finished_sem);
}

ZTEST(frag_decoder, test_frag_transport)
{
	uint8_t buf[256]; /* maximum size of one LoRaWAN message */
	uint8_t frag_session_setup_req[] = {
		CMD_FRAG_SESSION_SETUP,
		0x1f,
		NB_FRAG & 0xFF,
		(NB_FRAG >> 8) & 0xFF,
		FRAG_SIZE,
		0x01,
		PADDING,
		0x00,
		0x00,
		0x00,
		0x00,
	};
	int ret;

	k_sem_reset(&fuota_finished_sem);

	lorawan_emul_send_downlink(FRAG_TRANSPORT_PORT, false, 0, 0, sizeof(frag_session_setup_req),
				   frag_session_setup_req);

	for (int i = 0; i < sizeof(fw_coded) / FRAG_SIZE; i++) {
		if (i % 10 == 9) {
			/* loose every 10th packet */
			continue;
		}
		buf[0] = CMD_DATA_FRAGMENT;
		buf[1] = (i + 1) & 0xFF;
		buf[2] = (FRAG_SESSION_INDEX << 6) | ((i + 1) >> 8);
		memcpy(buf + 3, fw_coded + i * FRAG_SIZE, FRAG_SIZE);
		lorawan_emul_send_downlink(FRAG_TRANSPORT_PORT, false, 0, 0, FRAG_SIZE + 3, buf);
	}

	for (int i = 0; i < NB_FRAG; i++) {
		size_t num_bytes = (i == NB_FRAG - 1) ? (FRAG_SIZE - PADDING) : FRAG_SIZE;

		flash_area_read(fa, i * FRAG_SIZE, buf, num_bytes);
		zassert_mem_equal(buf, fw_raw + i * FRAG_SIZE, num_bytes, "fragment %d invalid",
				  i + 1);
	}

	ret = k_sem_take(&fuota_finished_sem, K_MSEC(100));
	zassert_equal(ret, 0, "FUOTA finish timed out");
}

static void *frag_decoder_setup(void)
{
	const struct device *lora_dev = DEVICE_DT_GET(DT_ALIAS(lora0));
	struct lorawan_join_config join_cfg = {0};
	int ret;

	k_sem_init(&fuota_finished_sem, 0, 1);

	ret = flash_area_open(TARGET_IMAGE_AREA, &fa);
	zassert_equal(ret, 0, "opening flash area failed: %d", ret);

	zassert_true(device_is_ready(lora_dev), "LoRa device not ready");

	ret = lorawan_start();
	zassert_equal(ret, 0, "lorawan_start failed: %d", ret);

	ret = lorawan_join(&join_cfg);
	zassert_equal(ret, 0, "lorawan_join failed: %d", ret);

	lorawan_frag_transport_run(fuota_finished);

	return NULL;
}

ZTEST_SUITE(frag_decoder, NULL, frag_decoder_setup, NULL, NULL, NULL);
