/*
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fuota.h"
#include "../lw_priv.h"

#include <LoRaMac.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/logging/log.h>
#include <zephyr/random/rand32.h>

LOG_MODULE_REGISTER(fuota_multicast, CONFIG_LORAWAN_FUOTA_LOG_LEVEL);

/* maximum length of multicast answers */
#define MAX_MULTICAST_ANS_LEN 5

enum multicast_commands {
	MULTICAST_CMD_PKG_VERSION              = 0x00,
	MULTICAST_CMD_MC_GROUP_STATUS          = 0x01,
	MULTICAST_CMD_MC_GROUP_SETUP           = 0x02,
	MULTICAST_CMD_MC_GROUP_DELETE          = 0x03,
	MULTICAST_CMD_MC_GROUP_CLASS_C_SESSION = 0x04,
	MULTICAST_CMD_MC_GROUP_CLASS_B_SESSION = 0x05,
};

struct multicast_context {
	/* McAddr: multicast group network address */
	uint32_t mc_addr;
	/* McKey_encrypted: encrypted multicast group key used to derive McAppSKey and McNetSKey */
	uint8_t mc_key_encrypted[16];
	/* minMcFCount: next frame counter value of the multicast downlink to be sent */
	uint32_t mc_fcnt_min;
	/* maxMcFCount: lifetime of this multicast group expressed as a maximum number of frames */
	uint32_t mc_fcnt_max;
	/** Start of the Class C window as GPS epoch modulo 2^32 */
	uint32_t session_time;
	/** Maximum duration of MC session before device reverts to class A */
	uint32_t session_timeout;
	/** Receive parameters for MC session */
	McRxParams_t rx_params;

	struct k_work_delayable session_start_work;
	struct k_work_delayable session_stop_work;
};

static struct lorawan_fuota_context *fuota_ctx;

static struct multicast_context ctx[LORAMAC_MAX_MC_CTX];

static void multicast_session_start(struct k_work *work)
{
	int err;

	k_mutex_lock(&fuota_ctx->mutex, K_FOREVER);

	err = lorawan_set_class(LORAWAN_CLASS_C);
	if (err) {
		LOG_WRN("Failed to switch to class C: %d. Retrying in 1s.", err);
		k_work_reschedule_for_queue(&fuota_ctx->work_queue,
					    k_work_delayable_from_work(work), K_SECONDS(1));
	} else {
		LOG_DBG("Switched to class C");
		fuota_ctx->active_class_c_sessions++;
	}

	k_mutex_unlock(&fuota_ctx->mutex);
}

static void multicast_session_stop(struct k_work *work)
{
	int err;

	k_mutex_lock(&fuota_ctx->mutex, K_FOREVER);

	if (fuota_ctx->active_class_c_sessions > 1) {
		fuota_ctx->active_class_c_sessions--;
	} else if (fuota_ctx->active_class_c_sessions == 1) {
		err = lorawan_set_class(LORAWAN_CLASS_A);
		if (err) {
			LOG_WRN("Failed to revert to class A: %d. Retrying in 1s.", err);
			k_work_reschedule_for_queue(&fuota_ctx->work_queue,
						k_work_delayable_from_work(work), K_SECONDS(1));
		} else {
			LOG_DBG("Reverted to class A");
			fuota_ctx->active_class_c_sessions--;
		}
	}

	k_mutex_unlock(&fuota_ctx->mutex);
}

static void multicast_package_callback(uint8_t port, bool data_pending, int16_t rssi, int8_t snr,
				       uint8_t len, const uint8_t *rx_buf)
{
	uint8_t tx_buf[3 * MAX_MULTICAST_ANS_LEN];
	uint8_t tx_pos = 0;
	uint8_t rx_pos = 0;

	if (port != LORAWAN_PORT_MULTICAST) {
		LOG_ERR("Wrong port %d for remote multicast package", port);
		return;
	}

	while (rx_pos < len) {
		uint8_t command_id = rx_buf[rx_pos++];

		switch (command_id) {
		case MULTICAST_CMD_PKG_VERSION:
			tx_buf[tx_pos++] = MULTICAST_CMD_PKG_VERSION;
			tx_buf[tx_pos++] = LORAWAN_PACKAGE_ID_REMOTE_MULTICAST_SETUP;
			tx_buf[tx_pos++] = CONFIG_LORAWAN_REMOTE_MULTICAST_VERSION;
			LOG_DBG("PackageVersionReq");
			break;
		case MULTICAST_CMD_MC_GROUP_STATUS:
			LOG_ERR("McGroupStatusReq not implemented");
			return;
		case MULTICAST_CMD_MC_GROUP_SETUP: {
			uint8_t id = rx_buf[rx_pos++] & 0x03;

			ctx[id].mc_addr = rx_buf[rx_pos++];
			ctx[id].mc_addr += (rx_buf[rx_pos++] << 8);
			ctx[id].mc_addr += (rx_buf[rx_pos++] << 16);
			ctx[id].mc_addr += (rx_buf[rx_pos++] << 24);

			for (int i = 0; i < 16; i++) {
				ctx[id].mc_key_encrypted[i] = rx_buf[rx_pos++];
			}

			ctx[id].mc_fcnt_min = rx_buf[rx_pos++];
			ctx[id].mc_fcnt_min += (rx_buf[rx_pos++] << 8);
			ctx[id].mc_fcnt_min += (rx_buf[rx_pos++] << 16);
			ctx[id].mc_fcnt_min += (rx_buf[rx_pos++] << 24);

			ctx[id].mc_fcnt_max = rx_buf[rx_pos++];
			ctx[id].mc_fcnt_max += (rx_buf[rx_pos++] << 8);
			ctx[id].mc_fcnt_max += (rx_buf[rx_pos++] << 16);
			ctx[id].mc_fcnt_max += (rx_buf[rx_pos++] << 24);

			LOG_DBG("McGroupSetupReq id: %u, addr: 0x%.8X, "
				"fcnt_min: %u, fcnt_max: %u", id, ctx[id].mc_addr,
				ctx[id].mc_fcnt_min, ctx[id].mc_fcnt_max);

			McChannelParams_t channel = {
				.IsRemotelySetup = true,
				.IsEnabled = true,
				.GroupID = (AddressIdentifier_t)id,
				.Address = ctx[id].mc_addr,
				.McKeys.McKeyE = ctx[id].mc_key_encrypted,
				.FCountMin = ctx[id].mc_fcnt_min,
				.FCountMax = ctx[id].mc_fcnt_max,
				.RxParams = {0}
			};
			LoRaMacStatus_t ret = LoRaMacMcChannelSetup(&channel);

			tx_buf[tx_pos++] = MULTICAST_CMD_MC_GROUP_SETUP;
			if (ret == LORAMAC_STATUS_OK) {
				tx_buf[tx_pos++] = id;
			} else if (ret == LORAMAC_STATUS_MC_GROUP_UNDEFINED) {
				/* set IDerror flag */
				tx_buf[tx_pos++] = (1U << 2) | id;
			} else {
				LOG_ERR("McGroupSetupReq failed: %s",
					lorawan_status2str(ret));
				return;
			}
			break;
		}
		case MULTICAST_CMD_MC_GROUP_DELETE: {
			uint8_t id = rx_buf[rx_pos++] & 0x03;

			LoRaMacStatus_t ret = LoRaMacMcChannelDelete((AddressIdentifier_t)id);

			LOG_DBG("McGroupDeleteReq id: %d", id);

			tx_buf[tx_pos++] = MULTICAST_CMD_MC_GROUP_DELETE;
			if (ret == LORAMAC_STATUS_OK) {
				tx_buf[tx_pos++] = id;
			} else if (ret == LORAMAC_STATUS_MC_GROUP_UNDEFINED) {
				/* set McGroupUndefined flag */
				tx_buf[tx_pos++] = (1U << 2) | id;
			} else {
				LOG_ERR("McGroupDeleteReq failed: %s",
					lorawan_status2str(ret));
				return;
			}
			break;
		}
		case MULTICAST_CMD_MC_GROUP_CLASS_C_SESSION: {
			uint8_t status = 0x00;
			uint8_t id = rx_buf[rx_pos++] & 0x03;

			ctx[id].session_time = rx_buf[rx_pos++];
			ctx[id].session_time += rx_buf[rx_pos++] << 8;
			ctx[id].session_time += rx_buf[rx_pos++] << 16;
			ctx[id].session_time += rx_buf[rx_pos++] << 24;

			ctx[id].session_timeout = 1U << (rx_buf[rx_pos++] & 0x0F);

			ctx[id].rx_params.Class = CLASS_C;

			ctx[id].rx_params.Params.ClassC.Frequency = rx_buf[rx_pos++];
			ctx[id].rx_params.Params.ClassC.Frequency += rx_buf[rx_pos++] << 8;
			ctx[id].rx_params.Params.ClassC.Frequency += rx_buf[rx_pos++] << 16;
			ctx[id].rx_params.Params.ClassC.Frequency *= 100;

			ctx[id].rx_params.Params.ClassC.Datarate = rx_buf[rx_pos++];

			LOG_DBG("McClassCSessionReq time: %u, timeout: %u, freq: %u, DR: %d",
				ctx[id].session_time, ctx[id].session_timeout,
				ctx[id].rx_params.Params.ClassC.Frequency,
				ctx[id].rx_params.Params.ClassC.Datarate);

			LoRaMacStatus_t ret = LoRaMacMcChannelSetupRxParams(
				(AddressIdentifier_t)id, &ctx[id].rx_params, &status);

			tx_buf[tx_pos++] = MULTICAST_CMD_MC_GROUP_CLASS_C_SESSION;
			if (ret == LORAMAC_STATUS_OK) {
				uint32_t current_time;
				int32_t time_to_start;

				if (lorawan_fuota_get_clock(&current_time) != 0) {
					LOG_WRN("Clock may not be synchronized");
				}

				time_to_start =	ctx[id].session_time - current_time;

				if (time_to_start > 0xFFFFFF) {
					/* truncated value indicates that clocks are out of sync */
					time_to_start = 0xFFFFFF;
				}

				if (time_to_start > 0) {
					LOG_DBG("Starting class C session in %d s",
						time_to_start);

					k_work_reschedule_for_queue(&fuota_ctx->work_queue,
						&ctx[id].session_start_work,
						K_SECONDS(time_to_start));

					k_work_reschedule_for_queue(&fuota_ctx->work_queue,
						&ctx[id].session_stop_work,
						K_SECONDS(time_to_start + ctx[id].session_timeout));

					tx_buf[tx_pos++] = status;
					tx_buf[tx_pos++] = (time_to_start >> 0) & 0xFF;
					tx_buf[tx_pos++] = (time_to_start >> 8) & 0xFF;
					tx_buf[tx_pos++] = (time_to_start >> 16) & 0xFF;
				} else {
					LOG_ERR("Missed class C session start at %d in %d s",
						ctx[id].session_time, time_to_start);
#if CONFIG_LORAWAN_REMOTE_MULTICAST_VERSION >= 2
					/* set StartMissed flag */
					tx_buf[tx_pos++] = (1U << 5) | status;
#endif
				}
			} else {
				LOG_ERR("McClassCSessionReq failed: %s",
					lorawan_status2str(ret));
				if (ret == LORAMAC_STATUS_MC_GROUP_UNDEFINED) {
					/* set McGroupUndefined flag */
					tx_buf[tx_pos++] = (1U << 4) | status;
				} else if (ret == LORAMAC_STATUS_FREQ_AND_DR_INVALID) {
					/* set FreqError and DR Error flags */
					tx_buf[tx_pos++] = (3U << 2) | status;
					return;
				}
			}
			break;
		}
		case MULTICAST_CMD_MC_GROUP_CLASS_B_SESSION:
			LOG_ERR("McClassBSessionReq not implemented");
			return;
		default:
			return;
		}
	}

	if (tx_pos > 0) {
		/* Random delay 2+-1 seconds according to RP002-1.0.3, chapter 2.3 */
		uint32_t delay = 1 + sys_rand32_get() % 3;

		fuota_schedule_uplink(LORAWAN_PORT_MULTICAST, tx_buf, tx_pos,
				      LORAWAN_MSG_UNCONFIRMED, K_SECONDS(delay));
	}
}

static struct lorawan_downlink_cb downlink_cb = {
	.port = (uint8_t)LORAWAN_PORT_MULTICAST,
	.cb = multicast_package_callback
};

int fuota_multicast_init(struct lorawan_fuota_context *fctx)
{
	fuota_ctx = fctx;

	for (int i = 0; i < ARRAY_SIZE(ctx); i++) {
		k_work_init_delayable(&ctx[i].session_start_work, multicast_session_start);
		k_work_init_delayable(&ctx[i].session_stop_work, multicast_session_stop);
	}

	lorawan_register_downlink_callback(&downlink_cb);

	return 0;
}
