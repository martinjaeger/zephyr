/*
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fuota.h"

#include <LoRaMac.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/rand32.h>

LOG_MODULE_REGISTER(fuota_clock_sync, CONFIG_LORAWAN_FUOTA_LOG_LEVEL);

/* Maximum length of clock sync answers */
#define MAX_CLOCK_SYNC_ANS_LEN 6

/* Delay between consecutive transmissions of AppTimeReq */
#define CLOCK_RESYNC_DELAY 10

/*
 * Maximum deviation in seconds to consider the clock sufficiently synchronized.
 *
 * The standard states "near-second accuracy" and "application-specific threshold".
 */
#define CLOCK_SYNC_MAX_DEVIATION 2

enum clock_sync_commands {
	CLOCK_SYNC_CMD_PKG_VERSION                 = 0x00,
	CLOCK_SYNC_CMD_APP_TIME                    = 0x01,
	CLOCK_SYNC_CMD_DEVICE_APP_TIME_PERIODICITY = 0x02,
	CLOCK_SYNC_CMD_FORCE_DEVICE_RESYNC         = 0x03,
};

struct clock_sync_context {
	/** Work item for regular (re-)sync requests (uplink messages) */
	struct k_work_delayable resync_work;
	/** Continuously incremented token to map clock sync answers and requests */
	uint8_t req_token;
	/** Number of requested clock sync requests left to be transmitted */
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
	/** Indication if the clock is considered sufficiently synchronized. */
	bool synchronized;
};

static struct lorawan_fuota_context *fuota_ctx;

static struct clock_sync_context ctx;

/**
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
	uint8_t tx_buf[3 * MAX_CLOCK_SYNC_ANS_LEN];
	uint8_t tx_pos = 0;
	uint8_t rx_pos = 0;

	if (port != LORAWAN_PORT_CLOCK_SYNC) {
		LOG_ERR("Wrong port %d for clock sync package", port);
		return;
	}

	while (rx_pos < len) {
		uint8_t command_id = rx_buf[rx_pos++];

		if (sizeof(tx_buf) - tx_pos < MAX_CLOCK_SYNC_ANS_LEN) {
			LOG_ERR("insufficient tx_buf size, some requests discarded");
			break;
		}

		switch (command_id) {
		case CLOCK_SYNC_CMD_PKG_VERSION:
			tx_buf[tx_pos++] = CLOCK_SYNC_CMD_PKG_VERSION;
			tx_buf[tx_pos++] = LORAWAN_PACKAGE_ID_CLOCK_SYNC;
			tx_buf[tx_pos++] = CONFIG_LORAWAN_APP_CLOCK_SYNC_VERSION;
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

				if (time_correction >= -CLOCK_SYNC_MAX_DEVIATION &&
				    time_correction < CLOCK_SYNC_MAX_DEVIATION) {
					ctx.synchronized = true;
				} else {
					ctx.synchronized = false;
				}
				LOG_DBG("AppTimeAns time_correction %d (token %d)",
					time_correction, token);
			} else {
				LOG_WRN("AppTimeAns with outdated token %d", token);
			}
			break;
		}
		case CLOCK_SYNC_CMD_DEVICE_APP_TIME_PERIODICITY: {
			uint8_t period = rx_buf[rx_pos++] & 0x0F;

			ctx.periodicity = 1U << (period + 7);

			tx_buf[tx_pos++] = CLOCK_SYNC_CMD_DEVICE_APP_TIME_PERIODICITY;
			tx_buf[tx_pos++] = 0x00; /* Status: OK */

			tx_pos += clock_sync_serialize_device_time(tx_buf + tx_pos,
								   sizeof(tx_buf) - tx_pos);

			LOG_DBG("DeviceAppTimePeriodicityReq period: %u", period);
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

	if (tx_pos > 0) {
		fuota_schedule_uplink(LORAWAN_PORT_CLOCK_SYNC, tx_buf, tx_pos,
				      LORAWAN_MSG_UNCONFIRMED, K_NO_WAIT);
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

	fuota_schedule_uplink(LORAWAN_PORT_CLOCK_SYNC, tx_buf, tx_pos, LORAWAN_MSG_UNCONFIRMED,
			      K_SECONDS(CLOCK_RESYNC_DELAY));

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
		ctx.nb_transmissions--;
		k_work_reschedule_for_queue(&fuota_ctx->work_queue, &ctx.resync_work,
					    K_SECONDS(CLOCK_RESYNC_DELAY));
	}

	return 0;
}

static void clock_sync_resync_handler(struct k_work *work)
{
	uint32_t periodicity;

	clock_sync_app_time_req();

	/* Add +-30s jitter to actual periodicity as required */
	periodicity = ctx.periodicity - 30 + sys_rand32_get() % 61;

	k_work_reschedule_for_queue(&fuota_ctx->work_queue, &ctx.resync_work,
				    K_SECONDS(periodicity));
}

static struct lorawan_downlink_cb downlink_cb = {
	.port = (uint8_t)LORAWAN_PORT_CLOCK_SYNC,
	.cb = clock_sync_package_callback
};

void fuota_clock_sync_start(struct lorawan_fuota_context *fctx)
{
	fuota_ctx = fctx;
	ctx.periodicity = CONFIG_LORAWAN_APP_CLOCK_SYNC_PERIODICITY;

	lorawan_register_downlink_callback(&downlink_cb);

	k_work_init_delayable(&ctx.resync_work, clock_sync_resync_handler);
	k_work_reschedule_for_queue(&fuota_ctx->work_queue, &ctx.resync_work, K_NO_WAIT);
}

int lorawan_fuota_get_clock(uint32_t *gps_time)
{
	__ASSERT(gps_time != NULL, "gps_time parameter is required");

	*gps_time = (uint32_t)(k_uptime_get() / 1000 + ctx.time_correction);
	if (ctx.synchronized) {
		return 0;
	} else {
		return -EAGAIN;
	}
}
