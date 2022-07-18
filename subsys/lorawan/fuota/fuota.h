/*
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_SUBSYS_LORAWAN_FUOTA_FUOTA_H_
#define ZEPHYR_SUBSYS_LORAWAN_FUOTA_FUOTA_H_

#include <zephyr.h>

/**
 * Unique package identifiers used for FUOTA services.
 */
enum lorawan_package_id {
	LORAWAN_PACKAGE_ID_COMPLIANCE = 0,
	LORAWAN_PACKAGE_ID_CLOCK_SYNC = 1,
	LORAWAN_PACKAGE_ID_REMOTE_MULTICAST_SETUP = 2,
	LORAWAN_PACKAGE_ID_FRAG_DATA_BLOCK = 3,
};

/**
 * Default ports used for FUOTA services.
 */
enum lorawan_port {
	LORAWAN_PORT_MULTICAST = 200,
	LORAWAN_PORT_FRAG_DATA = 201,
	LORAWAN_PORT_CLOCK_SYNC = 202,
};

struct lorawan_fuota_context {
	/**
	 * The FUOTA service needs a dedicated work queue as the LoRaWAN stack uses the system
	 * work queue and might get blocked if other LoRaWAN messages are sent and processed in
	 * parallel.
	 */
	struct k_work_q work_queue;
};

/**
 * Start clock synchronization work
 *
 * @param fuota_ctx Fuota context providing the work queue handle
 */
void fuota_clock_sync_start(struct lorawan_fuota_context *fuota_ctx);

/**
 * Retrieve the current synchronized time
 *
 * @returns synchronized time in GPS epoch format truncated to 32-bit
 */
uint32_t fuota_clock_sync_get_time(void);

/**
 * Initialize the multicast session layer
 *
 * This function assigns the downlink callback to receive incoming MC session
 * requests. After successfully setting up an MC session, the device is switched
 * to Class C mode.
 *
 * @param fuota_ctx Fuota context providing the work queue handle
 */
int fuota_multicast_init(struct lorawan_fuota_context *fuota_ctx);

#endif /* ZEPHYR_SUBSYS_LORAWAN_FUOTA_FUOTA_H_ */
