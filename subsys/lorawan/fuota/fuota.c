/*
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fuota.h"

#include <zephyr.h>

K_THREAD_STACK_DEFINE(thread_stack_area, CONFIG_LORAWAN_FUOTA_THREAD_STACK_SIZE);

static struct lorawan_fuota_context fuota_ctx;

void lorawan_start_fuota_service(void)
{
	k_work_queue_init(&fuota_ctx.work_queue);

	k_work_queue_start(&fuota_ctx.work_queue,
			   thread_stack_area, K_THREAD_STACK_SIZEOF(thread_stack_area),
			   CONFIG_LORAWAN_FUOTA_THREAD_PRIORITY, NULL);

	k_mutex_init(&fuota_ctx.mutex);

	k_thread_name_set(&fuota_ctx.work_queue.thread, "lorawan_fuota_work_q");

	fuota_multicast_init(&fuota_ctx);
	fuota_frag_transport_init(&fuota_ctx);

	/* initializes the fuota process on the server side */
	fuota_clock_sync_start(&fuota_ctx);
}
