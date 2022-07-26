/*
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fuota.h"

#include <LoRaMac.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fuota_clock_sync, CONFIG_LORAWAN_FUOTA_LOG_LEVEL);

/* maximum length of clock sync answers */
#define MAX_CLOCK_SYNC_ANS_LEN 6

/* delay between consecutive transmissions of AppTimeReq */
#define CLOCK_RESYNC_DELAY 10

enum clock_sync_commands {
	CLOCK_SYNC_CMD_PKG_VERSION                 = 0x00,
	CLOCK_SYNC_CMD_APP_TIME                    = 0x01,
	CLOCK_SYNC_CMD_DEVICE_APP_TIME_PERIODICITY = 0x02,
	CLOCK_SYNC_CMD_FORCE_DEVICE_RESYNC         = 0x03,
};

struct clock_sync_context {
	/* work item for regular (re-)sync requests (uplink messages) */
	struct k_work_delayable resync_work;

	/* work item for answers to requests from app server (uplink messages) */
	struct k_work tx_work;
	uint8_t tx_buf[3 * MAX_CLOCK_SYNC_ANS_LEN];
	uint8_t tx_pos;

	uint8_t req_token;
	uint8_t nb_transmissions;

	/**
	 * Offset to be added to system uptime to get GPS time (as used by LoRaWAN)
	 */
	int64_t time_correction;

	/**
	 * AppTimeReq retransmission interval in seconds
	 *
	 * Valid range between 128 (0x80) and 8388608 (0x800000)
	 */
	uint32_t periodicity;
};

static struct lorawan_fuota_context *fuota_ctx;

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

static void clock_sync_tx_handler(struct k_work *work)
{
	int err;

	err = lorawan_send(LORAWAN_PORT_CLOCK_SYNC, ctx.tx_buf, ctx.tx_pos,
			LORAWAN_MSG_UNCONFIRMED);
	if (err) {
		LOG_ERR("Sending clock sync answer failed: %d", err);
	}
}

static void clock_sync_package_callback(uint8_t port, bool data_pending, int16_t rssi, int8_t snr,
					uint8_t len, const uint8_t *rx_buf)
{
	uint8_t rx_pos = 0;

	if (port != LORAWAN_PORT_CLOCK_SYNC) {
		LOG_ERR("Wrong port %d for clock sync package", port);
		return;
	}

	if (k_work_is_pending(&ctx.tx_work)) {
		/* we are not allowed to use the tx buffer */
		LOG_ERR("tx_work pending, cannot process package");
		return;
	}

	ctx.tx_pos = 0;

	while (rx_pos < len) {
		uint8_t command_id = rx_buf[rx_pos++];

		if (sizeof(ctx.tx_buf) - ctx.tx_pos < MAX_CLOCK_SYNC_ANS_LEN) {
			LOG_ERR("insufficient tx_buf size, some requests discarded");
			break;
		}

		switch (command_id) {
		case CLOCK_SYNC_CMD_PKG_VERSION:
			ctx.tx_buf[ctx.tx_pos++] = CLOCK_SYNC_CMD_PKG_VERSION;
			ctx.tx_buf[ctx.tx_pos++] = LORAWAN_PACKAGE_ID_CLOCK_SYNC;
			ctx.tx_buf[ctx.tx_pos++] = CONFIG_LORAWAN_APP_CLOCK_SYNC_VERSION;
			LOG_DBG("PackageVersionReq");
			break;
		case CLOCK_SYNC_CMD_APP_TIME: {
			/* answer from application server */
			ctx.nb_transmissions = 0;
			int32_t time_correction = rx_buf[rx_pos++];

			time_correction	+= rx_buf[rx_pos++] << 8;
			time_correction	+= rx_buf[rx_pos++] << 16;
			time_correction	+= rx_buf[rx_pos++] << 24;

			uint8_t token = rx_buf[rx_pos++] & 0x0F;

			if (token == ctx.req_token) {
				ctx.time_correction += time_correction;
				ctx.req_token = (ctx.req_token + 1) % 16;

				LOG_DBG("AppTimeAns time_correction %d (token %d)",
					time_correction, token);
			} else {
				LOG_WRN("AppTimeAns with outdated token %d", token);
			}
			break;
		}
		case CLOCK_SYNC_CMD_DEVICE_APP_TIME_PERIODICITY: {
			/* ToDo: Extract periodicity and consider it for clock sync */
			rx_pos++;

			ctx.tx_buf[ctx.tx_pos++] = CLOCK_SYNC_CMD_DEVICE_APP_TIME_PERIODICITY;
			ctx.tx_buf[ctx.tx_pos++] = 0x01; /* Status: NotSupported */

			ctx.tx_pos +=
				clock_sync_serialize_device_time(ctx.tx_buf + ctx.tx_pos,
								 sizeof(ctx.tx_buf) - ctx.tx_pos);

			LOG_DBG("DeviceAppTimePeriodicityReq");
			break;
		}
		case CLOCK_SYNC_CMD_FORCE_DEVICE_RESYNC: {
			uint8_t nb_transmissions = rx_buf[rx_pos++] & 0x07;

			if (nb_transmissions != 0) {
				ctx.nb_transmissions = nb_transmissions;
				k_work_reschedule_for_queue(&fuota_ctx->work_queue,
					&ctx.resync_work, K_NO_WAIT);
			}

			LOG_DBG("ForceDeviceResyncCmd nb_transmissions: %u", nb_transmissions);
			break;
		}
		default:
			return;
		}
	}

	if (ctx.tx_pos > 0) {
		k_work_submit_to_queue(&fuota_ctx->work_queue, &ctx.tx_work);
	}
}

static int clock_sync_app_time_req(void)
{
	uint8_t tx_pos = 0;
	uint8_t tx_buf[6];

#if CONFIG_LORAWAN_APP_CLOCK_SYNC_VERSION == 1
	MibRequestConfirm_t mib_req;
	bool adr_enabled_prev;
	uint8_t nb_trans_prev;
	uint8_t datarate_prev;
#endif

	if (LoRaMacIsBusy()) {
		LOG_ERR("LoRaMAC is busy");
		return -EBUSY;
	}

	if (fuota_ctx->active_class_c_sessions > 0) {
		/* avoid disturbing the session and causing potential package loss */
		LOG_DBG("AppTimeReq not sent because of active class C session");
		return -EBUSY;
	}

	tx_buf[tx_pos++] = CLOCK_SYNC_CMD_APP_TIME;
	tx_pos += clock_sync_serialize_device_time(tx_buf + tx_pos,
						       sizeof(tx_buf) - tx_pos);

	/* Param: AnsRequired = 0 | TokenReq */
	tx_buf[tx_pos++] = ctx.req_token;

	LOG_DBG("Sending clock sync AppTimeReq (token %d)", ctx.req_token);

#if CONFIG_LORAWAN_APP_CLOCK_SYNC_VERSION == 1
	/* Disable ADR */
	mib_req.Type = MIB_ADR;
	LoRaMacMibGetRequestConfirm(&mib_req);
	adr_enabled_prev = mib_req.Param.AdrEnable;
	mib_req.Param.AdrEnable = false;
	LoRaMacMibSetRequestConfirm(&mib_req);

	/* Set NbTrans = 1 */
	mib_req.Type = MIB_CHANNELS_NB_TRANS;
	LoRaMacMibGetRequestConfirm(&mib_req);
	nb_trans_prev = mib_req.Param.ChannelsNbTrans;
	mib_req.Param.ChannelsNbTrans = 1;
	LoRaMacMibSetRequestConfirm(&mib_req);

	/* Store data rate */
	mib_req.Type = MIB_CHANNELS_DATARATE;
	LoRaMacMibGetRequestConfirm(&mib_req);
	datarate_prev = mib_req.Param.ChannelsDatarate;
#endif /* CONFIG_LORAWAN_APP_CLOCK_SYNC_VERSION == 1 */

	int err = lorawan_send(LORAWAN_PORT_CLOCK_SYNC, tx_buf, tx_pos,
			LORAWAN_MSG_UNCONFIRMED);
	if (err) {
		LOG_ERR("Sending clock sync AppTimeReq failed: %d", err);
	}

#if CONFIG_LORAWAN_APP_CLOCK_SYNC_VERSION == 1
	/* Revert ADR setting */
	mib_req.Type = MIB_ADR;
	mib_req.Param.AdrEnable = adr_enabled_prev;
	LoRaMacMibSetRequestConfirm(&mib_req);

	/* Revert NbTrans setting */
	mib_req.Type = MIB_CHANNELS_NB_TRANS;
	mib_req.Param.ChannelsNbTrans = nb_trans_prev;
	LoRaMacMibSetRequestConfirm(&mib_req);

	/* Revert data rate setting */
	mib_req.Type = MIB_CHANNELS_DATARATE;
	mib_req.Param.ChannelsDatarate = datarate_prev;
	LoRaMacMibSetRequestConfirm(&mib_req);
#endif /* CONFIG_LORAWAN_APP_CLOCK_SYNC_VERSION == 1 */

	if (ctx.nb_transmissions > 0) {
		if (!err) {
			ctx.nb_transmissions--;
		}
		k_work_reschedule_for_queue(&fuota_ctx->work_queue, &ctx.resync_work,
			K_SECONDS(CLOCK_RESYNC_DELAY));
	}

	return err;
}

static void clock_sync_resync_handler(struct k_work *work)
{
	clock_sync_app_time_req();

	/* ToDo: Add random value to periodicity (see spec) */
	k_work_reschedule_for_queue(&fuota_ctx->work_queue, &ctx.resync_work,
		K_SECONDS(ctx.periodicity));
}

static struct lorawan_downlink_cb downlink_cb = {
	.port = (uint8_t)LORAWAN_PORT_CLOCK_SYNC,
	.cb = clock_sync_package_callback
};

void fuota_clock_sync_start(struct lorawan_fuota_context *fctx)
{
	fuota_ctx = fctx;
	ctx.periodicity = 128; /* lowest valid value for testing */

	k_work_init(&ctx.tx_work, clock_sync_tx_handler);

	lorawan_register_downlink_callback(&downlink_cb);

	k_work_init_delayable(&ctx.resync_work, clock_sync_resync_handler);
	k_work_reschedule_for_queue(&fuota_ctx->work_queue, &ctx.resync_work, K_NO_WAIT);
}

uint32_t fuota_clock_sync_get_time(void)
{
	return (uint32_t)(k_uptime_get() / 1000 + ctx.time_correction);
}
