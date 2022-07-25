/*
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fuota.h"

#include <LoRaMac.h>
#include <FragDecoder.h>
#include <zephyr/lorawan/lorawan.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fuota_frag_transport, CONFIG_LORAWAN_FUOTA_LOG_LEVEL);

/**
 * Select LoRaWAN Fragmented Data Block Transport Specification
 *
 * 1: TS004-1.0.0 (as used in LoRaMAC-node v4.5.x and v4.6.x)
 * 2: TS004-2.0.0 (not fully implemented)
 */
#define FRAG_TRANSPORT_PACKAGE_VERSION CONFIG_LORAWAN_FUOTA_SPEC_VERSION

/* maximum length of frag_transport answers */
#define MAX_FRAG_TRANSPORT_ANS_LEN 5

enum frag_transport_commands {
	FRAG_TRANSPORT_CMD_PKG_VERSION         = 0x00,
	FRAG_TRANSPORT_CMD_FRAG_STATUS         = 0x01,
	FRAG_TRANSPORT_CMD_FRAG_SESSION_SETUP  = 0x02,
	FRAG_TRANSPORT_CMD_FRAG_SESSION_DELETE = 0x03,
#if FRAG_TRANSPORT_PACKAGE_VERSION >= 2
	FRAG_TRANSPORT_CMD_BLOCK_RECEIVED      = 0x04,
#endif /* FRAG_TRANSPORT_PACKAGE_VERSION */
	FRAG_TRANSPORT_CMD_DATA_FRAGMENT       = 0x08,
};

struct frag_transport_context {
	/** Stores if this session is active */
	bool is_active;
	union {
		uint8_t frag_session;
		struct {
			/** Multicast groups allowed to input to this frag session */
			uint8_t mc_group_bit_mask: 4;
			/** Identifies this session (equal to array index) */
			uint8_t frag_index: 2;
		};
	};
	/** Number of fragments of the data block for this session, max. 2^14-1 */
	uint16_t nb_frag;
	/** Size of each fragment in octets */
	uint8_t frag_size;
	union {
		uint8_t control;
		struct {
			/** Random delay to be added for some responses */
			uint8_t block_ack_delay: 3;
			/** Used fragmentation algorithm (0 for forward error correction) */
			uint8_t frag_algo: 3;
#if FRAG_TRANSPORT_PACKAGE_VERSION >= 2
			/** Specifies if full block reception should be ACKed */
			uint8_t ack_reception : 1;
#endif /* FRAG_TRANSPORT_PACKAGE_VERSION */
		};
	};
	/** Padding in the last fragment if total size is not a multiple of frag_size */
	uint8_t padding;
	/** Application-specific descriptor for the data block, e.g. firmware version */
	uint32_t descriptor;

	/* variables required for FragDecoder.h */
	FragDecoderStatus_t decoder_status;
	FragDecoderCallbacks_t decoder_callbacks;
	int32_t decoder_process_status;
};

static struct k_work_q *workq;

static struct k_work_delayable tx_work;
static uint8_t tx_buf[3 * MAX_FRAG_TRANSPORT_ANS_LEN];
static uint8_t tx_pos;

static struct frag_transport_context ctx[LORAMAC_MAX_MC_CTX];

static int8_t frag_decoder_write(uint32_t addr, uint8_t *data, uint32_t size)
{
	LOG_DBG("write %u bytes to addr 0x%x", size, addr);

	return 0;
}

static int8_t frag_decoder_read(uint32_t addr, uint8_t *data, uint32_t size)
{
	LOG_DBG("read %u bytes from addr 0x%x", size, addr);

	return 0;
}

static void frag_decoder_finish(void)
{
	/* ToDo: Finish flash writing and reboot? */

	LOG_DBG("frag decoder finish");
}

static void frag_transport_tx_handler(struct k_work *work)
{
	int err;

	err = lorawan_send(LORAWAN_PORT_FRAG_TRANSPORT, tx_buf, tx_pos, LORAWAN_MSG_UNCONFIRMED);
	if (err) {
		LOG_ERR("Sending frag data answer failed: %d", err);
	}
}

static void frag_transport_package_callback(uint8_t port, bool data_pending, int16_t rssi,
					    int8_t snr, uint8_t len, const uint8_t *rx_buf)
{
	uint8_t rx_pos = 0;
	bool delayed_answer = false;

	if (port != LORAWAN_PORT_FRAG_TRANSPORT) {
		LOG_ERR("Wrong port %d for frag data package", port);
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

		LOG_DBG("Received frag data cmd 0x%.2x", command_id);

		switch (command_id) {
		case FRAG_TRANSPORT_CMD_PKG_VERSION:
			/* ToDo: Don't process in case of multicast session */

			tx_buf[tx_pos++] = FRAG_TRANSPORT_CMD_PKG_VERSION;
			tx_buf[tx_pos++] = LORAWAN_PACKAGE_ID_FRAG_TRANSPORT_BLOCK;
			tx_buf[tx_pos++] = FRAG_TRANSPORT_PACKAGE_VERSION;
			break;
		case FRAG_TRANSPORT_CMD_FRAG_STATUS: {
			uint8_t frag_status = rx_buf[rx_pos++] & 0x07;
			uint8_t participants = frag_status & 0x01;
			uint8_t index = frag_status >> 1;

			ctx[index].decoder_status = FragDecoderGetStatus();

			if (participants == 1 || ctx[index].decoder_status.FragNbLost > 0) {
				tx_buf[tx_pos++] = FRAG_TRANSPORT_CMD_FRAG_STATUS;
				tx_buf[tx_pos++] = ctx[index].decoder_status.FragNbRx & 0xFF;
				tx_buf[tx_pos++] = (index << 6) |
					((ctx[index].decoder_status.FragNbRx >> 8) & 0x3F);
				tx_buf[tx_pos++] = ctx[index].decoder_status.FragNbLost;
				tx_buf[tx_pos++] = ctx[index].decoder_status.MatrixError & 0x01;

				delayed_answer = true;
			}
			break;
		}
		case FRAG_TRANSPORT_CMD_FRAG_SESSION_SETUP: {
			/* ToDo: Don't process in case of multicast session */

			uint8_t frag_session = rx_buf[rx_pos++] & 0x3F;
			uint8_t index = frag_session >> 4;
			uint8_t status = index << 6;

			ctx[index].frag_session = frag_session;

			ctx[index].nb_frag = rx_buf[rx_pos++];
			ctx[index].nb_frag |= rx_buf[rx_pos++] << 8;

			ctx[index].frag_size = rx_buf[rx_pos++];
			ctx[index].control = rx_buf[rx_pos++];
			ctx[index].padding = rx_buf[rx_pos++];

			ctx[index].descriptor = rx_buf[rx_pos++];
			ctx[index].descriptor += rx_buf[rx_pos++] << 8;
			ctx[index].descriptor += rx_buf[rx_pos++] << 16;
			ctx[index].descriptor += rx_buf[rx_pos++] << 24;

			LOG_DBG("FragSessionSetupReq index %d, nb_frag: %u, frag_size: %u, "
				"padding: %u, control: 0x%x, descriptor: 0x%.8x", index,
				ctx[index].nb_frag, ctx[index].frag_size, ctx[index].padding,
				ctx[index].control, ctx[index].descriptor);

			/* ToDo: Add new Spec v2 features
			 * - SessionCnt to prevent replay attacks
			 * - MIC for integrity check and authentication
			 */

			if (ctx[index].frag_algo > 0) {
				/* FragAlgo unsupported */
				status |= 1U << 0;
			}

			if (ctx[index].nb_frag > FRAG_MAX_NB ||
					ctx[index].frag_size > FRAG_MAX_SIZE ||
					ctx[index].nb_frag * ctx[index].frag_size >
					FragDecoderGetMaxFileSize()) {
				/* Not enough memory */
				status |= 1U << 1;
			}

			if (ctx[index].frag_index >= ARRAY_SIZE(ctx)) {
				/* FragIndex unsupported */
				status |= 1U << 2;
			}

			/* ToDo: Handle Wrong Descriptor error */

			if ((status & 0x1F) == 0)	{
				ctx[index].is_active = true;
				ctx[index].decoder_callbacks.FragDecoderWrite = frag_decoder_write;
				ctx[index].decoder_callbacks.FragDecoderRead = frag_decoder_read;
				FragDecoderInit(ctx[index].nb_frag, ctx[index].frag_size,
						&ctx[index].decoder_callbacks);
			}

			tx_buf[tx_pos++] = FRAG_TRANSPORT_CMD_FRAG_SESSION_SETUP;
			tx_buf[tx_pos++] = status;
			delayed_answer = false;
			break;
		}
		case FRAG_TRANSPORT_CMD_FRAG_SESSION_DELETE: {
			/* ToDo: Don't process in case of multicast session */

			uint8_t index = rx_buf[rx_pos++] & 0x03;
			uint8_t status = 0x00;

			status |= index;
			if (index >= ARRAY_SIZE(ctx) || ctx[index].is_active == false) {
				/* Session does not exist */
				status |= 1U << 3;
			} else {
				ctx[index].is_active = false;
			}

			tx_buf[tx_pos++] = FRAG_TRANSPORT_CMD_FRAG_SESSION_DELETE;
			tx_buf[tx_pos++] = status;
			delayed_answer = false;
			break;
		}
#if FRAG_TRANSPORT_PACKAGE_VERSION >= 2
		case FRAG_TRANSPORT_CMD_BLOCK_RECEIVED:
			LOG_ERR("FragDataBlockReceivedAns not implemented");
			return;
#endif /* FRAG_TRANSPORT_PACKAGE_VERSION */
		case FRAG_TRANSPORT_CMD_DATA_FRAGMENT: {
			uint8_t frag_index_n;

			frag_index_n = rx_buf[rx_pos++];
			frag_index_n |= rx_buf[rx_pos++] << 8;

			uint16_t frag_counter = frag_index_n & 0x3FFF;
			uint8_t index = (frag_index_n >> 14) & 0x03;

			LOG_DBG("DataFragment frag_counter: %u, index: %u",
				frag_counter, index);

			if (ctx[index].decoder_process_status == FRAG_SESSION_ONGOING) {
				ctx[index].decoder_process_status =
					FragDecoderProcess(frag_counter,
							   (uint8_t *)&rx_buf[rx_pos]);
				ctx[index].decoder_status = FragDecoderGetStatus();

				LOG_INF("received frag %d", frag_counter);
			} else {
				if (ctx[index].decoder_process_status >= 0) {
					/* fragmented data transfer finished */
					ctx[index].decoder_process_status =
									FRAG_SESSION_NOT_STARTED;
					frag_decoder_finish();
				}
			}
			rx_pos += ctx[index].frag_size;
			break;
		}
		default:
			return;
		}
	}

	if (tx_pos > 0) {
		/* ToDo: consider delayed_answer and add random number */
		k_work_reschedule_for_queue(workq, &tx_work, K_SECONDS(2));
	}
}

static struct lorawan_downlink_cb downlink_cb = {
	.port = (uint8_t)LORAWAN_PORT_FRAG_TRANSPORT,
	.cb = frag_transport_package_callback
};

int fuota_frag_transport_init(struct lorawan_fuota_context *fuota_ctx)
{
	workq = &fuota_ctx->work_queue;

	k_work_init_delayable(&tx_work, frag_transport_tx_handler);

	lorawan_register_downlink_callback(&downlink_cb);

	return 0;
}
