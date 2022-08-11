/*
 * Copyright (c) 2022 Martin Jäger <martin@libre.solar>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "fuota.h"

#include <zephyr/zephyr.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(fuota, CONFIG_LORAWAN_FUOTA_LOG_LEVEL);

struct fuota_uplink_msg {
	bool used;
	uint8_t port;
	uint8_t data[10];
	uint8_t len;
	enum lorawan_message_type type;
	/* absolute ticks when this message should be scheduled */
	int64_t ticks;
	sys_snode_t node;
};

K_THREAD_STACK_DEFINE(thread_stack_area, CONFIG_LORAWAN_FUOTA_THREAD_STACK_SIZE);

static struct lorawan_fuota_context fuota_ctx;

static struct k_work_delayable uplink_work;

/* single-linked list (with pointers) and array for implementation of priority queue */
static struct fuota_uplink_msg messages[10];
static sys_slist_t msg_list;
static struct k_sem msg_sem;

static void fuota_uplink_handler(struct k_work *work)
{
	struct fuota_uplink_msg msg_copy;
	struct fuota_uplink_msg *first;
	sys_snode_t *node;
	int err;

	ARG_UNUSED(work);

	/* take semaphore and create a copy of the next message */
	k_sem_take(&msg_sem, K_FOREVER);

	node = sys_slist_get(&msg_list);
	if (node == NULL) {
		goto out;
	}

	first = CONTAINER_OF(node, struct fuota_uplink_msg, node);
	msg_copy = *first;
	first->used = false;
	sys_slist_remove(&msg_list, NULL, &first->node);

	/* semaphore must be given back before calling lorawan_send */
	k_sem_give(&msg_sem);

	err = lorawan_send(msg_copy.port, msg_copy.data, msg_copy.len, msg_copy.type);
	if (!err) {
		LOG_DBG("Message sent to port %d", msg_copy.port);
	} else {
		LOG_ERR("Sending message to port %d failed: %d",
			msg_copy.port, err);
	}

	/* take the semaphore again to schedule next uplink */
	k_sem_take(&msg_sem, K_FOREVER);

	node = sys_slist_peek_head(&msg_list);
	if (node == NULL) {
		goto out;
	}
	first = CONTAINER_OF(node, struct fuota_uplink_msg, node);
	//LOG_DBG("next message for port %d pending in %lld ticks",
	//	first->port, first->ticks - k_uptime_ticks());
	k_work_reschedule_for_queue(&fuota_ctx.work_queue, &uplink_work,
		K_TIMEOUT_ABS_TICKS(first->ticks));

out:
	k_sem_give(&msg_sem);
}

static inline void insert_uplink(struct fuota_uplink_msg *msg_new)
{
	struct fuota_uplink_msg *msg_prev;

	if (sys_slist_is_empty(&msg_list)) {
		sys_slist_append(&msg_list, &msg_new->node);
	} else {
		int count = 0;
		SYS_SLIST_FOR_EACH_CONTAINER(&msg_list, msg_prev, node) {
			count++;
			if (msg_prev->ticks <= msg_new->ticks) {
				break;
			}
		}
		if (msg_prev != NULL) {
			sys_slist_insert(&msg_list, &msg_prev->node, &msg_new->node);
		} else {
			sys_slist_append(&msg_list, &msg_new->node);
		}
	}
}

int fuota_schedule_uplink(uint8_t port, uint8_t *data, uint8_t len,
			  enum lorawan_message_type type, k_timeout_t timeout)
{
	struct fuota_uplink_msg *next;
	int64_t timeout_abs_ticks;

	if (len > sizeof(messages[0].data)) {
		LOG_ERR("Uplink payload too long.");
		return -EFBIG;
	}

	timeout_abs_ticks = k_uptime_ticks() + timeout.ticks;

	k_sem_take(&msg_sem, K_FOREVER);

	for (int i = 0; i < ARRAY_SIZE(messages); i++) {
		if (!messages[i].used) {
			memcpy(messages[i].data, data, len);
			messages[i].port = port;
			messages[i].len = len;
			messages[i].type = type;
			messages[i].ticks = timeout_abs_ticks;
			messages[i].used = true;

			insert_uplink(&messages[i]);

			next = SYS_SLIST_PEEK_HEAD_CONTAINER(&msg_list, next, node);
			if (next != NULL) {
				//LOG_DBG("uplink: next message for port %d pending in %lld ticks",
				//	next->port, next->ticks - k_uptime_ticks());
				k_work_reschedule_for_queue(&fuota_ctx.work_queue, &uplink_work,
					K_TIMEOUT_ABS_TICKS(next->ticks));
			}

			k_sem_give(&msg_sem);

			return 0;
		}
	}

	k_sem_give(&msg_sem);

	LOG_WRN("Message queue full, message for port %u dropped.", port);

	return -ENOSPC;
}

void lorawan_fuota_run(void (*fuota_finished_cb)(void))
{
	fuota_ctx.finished_cb = fuota_finished_cb;

	sys_slist_init(&msg_list);
	k_sem_init(&msg_sem, 1, 1);

	k_work_queue_init(&fuota_ctx.work_queue);
	k_work_queue_start(&fuota_ctx.work_queue,
			   thread_stack_area, K_THREAD_STACK_SIZEOF(thread_stack_area),
			   CONFIG_LORAWAN_FUOTA_THREAD_PRIORITY, NULL);

	k_work_init_delayable(&uplink_work, fuota_uplink_handler);

	k_mutex_init(&fuota_ctx.mutex);

	k_thread_name_set(&fuota_ctx.work_queue.thread, "lorawan_fuota_work_q");

	fuota_multicast_init(&fuota_ctx);
	fuota_frag_transport_init(&fuota_ctx);

	fuota_clock_sync_start(&fuota_ctx);
}
