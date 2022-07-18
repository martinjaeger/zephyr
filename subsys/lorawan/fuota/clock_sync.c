/*
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fuota.h"

#include <LoRaMac.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fuota_clock_sync, CONFIG_LORAWAN_LOG_LEVEL);

/* LoRaWAN Application Layer Clock Synchronization Specification TS003-2.0.0 */
#define CLOCK_SYNC_PACKAGE_VERSION 2

enum clock_sync_commands {
	CLOCK_SYNC_CMD_PKG_VERSION                 = 0x00,
	CLOCK_SYNC_CMD_APP_TIME                    = 0x01,
	CLOCK_SYNC_CMD_DEVICE_APP_TIME_PERIODICITY = 0x02,
	CLOCK_SYNC_CMD_FORCE_DEVICE_RESYNC         = 0x03,
};

struct clock_sync_context {
	struct k_work_q *workq;
	struct k_work_delayable sync_work;

	uint8_t req_token;
	uint8_t nb_transmissions;

	/**
	 * Offset to be added to system uptime to get GPS time (as used by LoRaWAN)
	 */
	int64_t time_correction;

	bool app_time_req_pending;

	bool adr_enabled_prev;
	uint8_t nb_transmissions_prev;
	uint8_t datarate_prev;

	/**
	 * AppTimeReq retransmission interval in seconds
	 *
	 * Valid range between 128 (0x80) and 8388608 (0x800000)
	 */
	uint32_t periodicity;
};

static struct clock_sync_context ctx;

/*
 * Writes the DeviceTime into the buffer.
 *
 * @returns number of bytes written or -1 in case of error
 */
static int clock_sync_serialize_device_time(uint8_t *buf, size_t size)
{
	uint64_t device_time = k_uptime_get() / 1000 + ctx.time_correction;

	if (size < 4) {
		return -1;
	}

	buf[0] = (device_time >> 0) & 0xFF;
	buf[1] = (device_time >> 8) & 0xFF;
	buf[2] = (device_time >> 16) & 0xFF;
	buf[3] = (device_time >> 24) & 0xFF;

	return 4;
}

static void clock_sync_package_callback(uint8_t port, bool data_pending, int16_t rssi, int8_t snr,
					uint8_t len, const uint8_t *rx_buf)
{
	uint8_t rx_buf_pos = 0;
	uint8_t tx_buf_pos = 0;
	uint8_t tx_buf[6];

	if (port != LORAWAN_PORT_CLOCK_SYNC) {
		LOG_ERR("Wrong port %d for clock sync package", port);
		return;
	}

	while (rx_buf_pos < len) {
		uint8_t command_id = rx_buf[rx_buf_pos++];

		LOG_DBG("Received clock sync cmd 0x%.2x", command_id);

		switch (command_id) {
		case CLOCK_SYNC_CMD_PKG_VERSION:
			tx_buf[tx_buf_pos++] = CLOCK_SYNC_CMD_PKG_VERSION;
			tx_buf[tx_buf_pos++] = LORAWAN_PACKAGE_ID_CLOCK_SYNC;
			tx_buf[tx_buf_pos++] = CLOCK_SYNC_PACKAGE_VERSION;
			break;
		case CLOCK_SYNC_CMD_APP_TIME: {
			/* answer from application server */
			ctx.nb_transmissions = 0;
			int32_t time_correction = rx_buf[rx_buf_pos++];

			time_correction	+= rx_buf[rx_buf_pos++] << 8;
			time_correction	+= rx_buf[rx_buf_pos++] << 16;
			time_correction	+= rx_buf[rx_buf_pos++] << 24;

			uint8_t token = rx_buf[rx_buf_pos++] & 0x0F;

			if (token == ctx.req_token) {
				ctx.time_correction += time_correction;
				ctx.req_token = (ctx.req_token + 1) % 16;
			}
			break;
		}
		case CLOCK_SYNC_CMD_DEVICE_APP_TIME_PERIODICITY: {
			/* ToDo: Extract periodicity and consider it for clock sync */
			rx_buf_pos++;

			tx_buf[tx_buf_pos++] = CLOCK_SYNC_CMD_DEVICE_APP_TIME_PERIODICITY;
			tx_buf[tx_buf_pos++] = 0x01; /* Status: NotSupported */

			tx_buf_pos +=
				clock_sync_serialize_device_time(tx_buf + tx_buf_pos,
								 sizeof(tx_buf) - tx_buf_pos);
			break;
		}
		case CLOCK_SYNC_CMD_FORCE_DEVICE_RESYNC: {
			uint8_t nb_transmissions = rx_buf[rx_buf_pos++] & 0x07;

			if (nb_transmissions != 0) {
				ctx.nb_transmissions = nb_transmissions;
			}
			break;
		}
		default:
			return;
		}
	}

	if (tx_buf_pos > 0) {
		int err = lorawan_send(LORAWAN_PORT_CLOCK_SYNC, tx_buf, tx_buf_pos,
				LORAWAN_MSG_UNCONFIRMED);
		if (err) {
			LOG_ERR("Sending clock sync answer failed: %d", err);
		}
	}
}

static int clock_sync_app_time_req(void)
{
	uint8_t tx_buf_pos = 0;
	uint8_t tx_buf[6];

	if (LoRaMacIsBusy()) {
		LOG_ERR("LoRaMAC is busy");
		return -EBUSY;
	}

	if (ctx.app_time_req_pending) {
		MibRequestConfirm_t mib_req;

		/* Disable ADR */
		mib_req.Type = MIB_ADR;
		LoRaMacMibGetRequestConfirm(&mib_req);
		ctx.adr_enabled_prev = mib_req.Param.AdrEnable;
		mib_req.Param.AdrEnable = false;
		LoRaMacMibSetRequestConfirm(&mib_req);

		/* Set NbTrans = 1 */
		mib_req.Type = MIB_CHANNELS_NB_TRANS;
		LoRaMacMibGetRequestConfirm(&mib_req);
		ctx.nb_transmissions_prev = mib_req.Param.ChannelsNbTrans;
		mib_req.Param.ChannelsNbTrans = 1;
		LoRaMacMibSetRequestConfirm(&mib_req);

		/* Store data rate */
		mib_req.Type = MIB_CHANNELS_DATARATE;
		LoRaMacMibGetRequestConfirm(&mib_req);
		ctx.datarate_prev = mib_req.Param.ChannelsDatarate;
	}

	tx_buf[tx_buf_pos++] = CLOCK_SYNC_CMD_APP_TIME;
	tx_buf_pos += clock_sync_serialize_device_time(tx_buf + tx_buf_pos,
						       sizeof(tx_buf) - tx_buf_pos);

	/* Param: AnsRequired = 0 | TokenReq */
	tx_buf[tx_buf_pos++] = ctx.req_token;

	LOG_DBG("Sending clock sync AppTimeReq");

	ctx.app_time_req_pending = true;
	int err = lorawan_send(LORAWAN_PORT_CLOCK_SYNC, tx_buf, tx_buf_pos,
			LORAWAN_MSG_UNCONFIRMED);
	if (err) {
		LOG_ERR("Sending clock sync AppTimeReq failed: %d", err);
	}

	return err;
}

static void clock_sync_handler(struct k_work *work)
{
	clock_sync_app_time_req();

	/* ToDo: Add random value to periodicity (see spec) */
	k_work_reschedule_for_queue(ctx.workq, &ctx.sync_work, K_SECONDS(ctx.periodicity));
}

static struct lorawan_downlink_cb downlink_cb = {
	.port = (uint8_t)LORAWAN_PORT_CLOCK_SYNC,
	.cb = clock_sync_package_callback
};

void fuota_clock_sync_start(struct lorawan_fuota_context *fuota_ctx)
{
	lorawan_register_downlink_callback(&downlink_cb);

	ctx.workq = &fuota_ctx->work_queue;
	ctx.periodicity = 128; /* lowest valid value for testing */

	k_work_init_delayable(&ctx.sync_work, clock_sync_handler);
	k_work_reschedule_for_queue(ctx.workq, &ctx.sync_work, K_NO_WAIT);
}

uint32_t fuota_clock_sync_get_time(void)
{
	return (uint32_t)(k_uptime_get() / 1000 + ctx.time_correction);
}
