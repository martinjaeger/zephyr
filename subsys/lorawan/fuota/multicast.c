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

LOG_MODULE_REGISTER(fuota_multicast, CONFIG_LORAWAN_FUOTA_LOG_LEVEL);

/**
 * Select LoRaWAN Remote Multicast Setup Specification
 *
 * 1: TS005-1.0.0 (as used in LoRaMAC-node v4.5.x and v4.6.x)
 * 2: TS005-2.0.0 (not fully implemented)
 */
#define MULTICAST_PACKAGE_VERSION CONFIG_LORAWAN_FUOTA_SPEC_VERSION

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
	uint8_t group_id;
	uint32_t mc_addr;
	uint8_t mc_key_encrypted[16];
	uint32_t mc_fcnt_min;
	uint32_t mc_fcnt_max;
	/** start of the Class C window as GPS epoch modulo 32 */
	uint32_t session_time;
	/** maximum duration of MC session before device reverts to class A */
	uint32_t session_timeout;
	McRxParams_t rx_params;

	struct k_work_delayable session_start_work;
	struct k_work_delayable session_stop_work;
};

static struct k_work_q *workq;

static struct k_work_delayable tx_work;
static uint8_t tx_buf[3 * MAX_MULTICAST_ANS_LEN];
static uint8_t tx_pos;

static struct multicast_context ctx[LORAMAC_MAX_MC_CTX];

static void multicast_session_start(struct k_work *work)
{
	int err;

	err = lorawan_set_class(LORAWAN_CLASS_C);
	if (err) {
		LOG_ERR("Failed to switch to class C: %d", err);
	} else {
		LOG_DBG("Switched to class C");
	}
}

static void multicast_session_stop(struct k_work *work)
{
	int err;

	/* ToDo: Check if there are other MC sessions in progress before switching class */

	err = lorawan_set_class(LORAWAN_CLASS_A);
	if (err) {
		LOG_ERR("Failed to revert to class A: %d", err);
	} else {
		LOG_DBG("Reverted to class A");
	}
}

static void multicast_tx_handler(struct k_work *work)
{
	int err;

	err = lorawan_send(LORAWAN_PORT_MULTICAST, tx_buf, tx_pos, LORAWAN_MSG_UNCONFIRMED);
	if (err) {
		LOG_ERR("Sending multicast answer failed: %d", err);
	}
}

static void multicast_package_callback(uint8_t port, bool data_pending, int16_t rssi, int8_t snr,
				       uint8_t len, const uint8_t *rx_buf)
{
	uint8_t rx_pos = 0;

	if (port != LORAWAN_PORT_MULTICAST) {
		LOG_ERR("Wrong port %d for remote multicast package", port);
		return;
	}

	if (k_work_delayable_is_pending(&tx_work)) {
		/* we are not allowed to use the tx buffer */
		LOG_ERR("tx_work pending, cannot process package");
		return;
	}

	tx_pos = 0;

	while (rx_pos < len) {
		uint8_t command_id = rx_buf[rx_pos++];

		switch (command_id) {
		case MULTICAST_CMD_PKG_VERSION:
			tx_buf[tx_pos++] = MULTICAST_CMD_PKG_VERSION;
			tx_buf[tx_pos++] = LORAWAN_PACKAGE_ID_REMOTE_MULTICAST_SETUP;
			tx_buf[tx_pos++] = MULTICAST_PACKAGE_VERSION;
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

			LOG_DBG("McGroupSetupReq id: %d, addr: 0x%.8X, "
				"fcnt_min: %u, fcnt_max: %d", id, ctx[id].mc_addr,
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
				int32_t time_to_start =
					ctx[id].session_time - fuota_clock_sync_get_time();

				if (time_to_start > 0xFFFFFF) {
					/* truncated value indicates that clocks are out of sync */
					time_to_start = 0xFFFFFF;
				}

				if (time_to_start > 0) {
					LOG_DBG("Starting class C session in %d s",
						time_to_start);

					k_work_init_delayable(&ctx[id].session_start_work,
						multicast_session_start);
					k_work_reschedule_for_queue(workq,
						&ctx[id].session_start_work,
						K_SECONDS(time_to_start));

					k_work_init_delayable(&ctx[id].session_stop_work,
						multicast_session_stop);
					k_work_reschedule_for_queue(workq,
						&ctx[id].session_stop_work,
						K_SECONDS(time_to_start + ctx[id].session_timeout));

					tx_buf[tx_pos++] = status;
					tx_buf[tx_pos++] = (time_to_start >> 0) & 0xFF;
					tx_buf[tx_pos++] = (time_to_start >> 8) & 0xFF;
					tx_buf[tx_pos++] = (time_to_start >> 16) & 0xFF;
				} else {
					LOG_ERR("Missed class C session start at %d in %d s",
						ctx[id].session_time, time_to_start);
					/* set StartMissed flag */
					tx_buf[tx_pos++] = (1U << 5) | status;
				}
				break;
			} else {
				LOG_ERR("McClassCSessionReq failed: %s",
					lorawan_status2str(ret));
				if (ret == LORAMAC_STATUS_MC_GROUP_UNDEFINED) {
					/* set McGroupUndefined flag */
					tx_buf[tx_pos++] = (1U << 4) | status;
				} else {
					/* ToDo: consider FreqError and DR Errors */
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
		/* ToDo: Random delay 2+-1 seconds according to RP002-1.0.3, chapter 2.3 */
		k_work_reschedule_for_queue(workq, &tx_work, K_SECONDS(2));
	}
}

static struct lorawan_downlink_cb downlink_cb = {
	.port = (uint8_t)LORAWAN_PORT_MULTICAST,
	.cb = multicast_package_callback
};

int fuota_multicast_init(struct lorawan_fuota_context *fuota_ctx)
{
	workq = &fuota_ctx->work_queue;

	k_work_init_delayable(&tx_work, multicast_tx_handler);

	lorawan_register_downlink_callback(&downlink_cb);

	return 0;
}
