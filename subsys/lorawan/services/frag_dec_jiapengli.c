/*
 * Copyright (c) 2022 Jiapeng Li
 *
 * Original source: https://github.com/JiapengLi/LoRaWANFragmentedDataBlockTransportAlgorithm
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "frag_dec_jiapengli.h"

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(lorawan_frag_dec, CONFIG_LORAWAN_SERVICES_LOG_LEVEL);

static bool is_power2(uint32_t num)
{
	return (num != 0) && ((num & (num - 1)) == 0);
}

static uint32_t prbs23(uint32_t x)
{
	uint32_t b0, b1;

	b0 = (x & 0x00000001);
	b1 = (x & 0x00000020) >> 5;
	return (x >> 1) + ((b0 ^ b1) << 22);
}

/* n: index of the uncoded and coded fragmentations, maximum N - 1, starts from 0 */
static int matrix_line_bm(bm_t *bm, int n, int m)
{
	int mm, x, nbCoeff, r;

	bit_clear_all(bm, m);

	/* from 0 to m - 1 */
	if (n < m) {
		bit_set(bm, n);
		return 0;
	}

	/* from m to N */
	n = n - m + 1;
	mm = 0;
	if (is_power2(m)) {
		mm = 1;
	}
	mm += m;

	x = 1 + (1001 * n);

	for (nbCoeff = 0; nbCoeff < (m / 2); nbCoeff++) {
		r = (1 << 16);
		while (r >= m) {
			x = prbs23(x);
			r = x % mm;
		}
		bit_set(bm, r);
	}

	return 0;
}

static int buf_xor(uint8_t *des, uint8_t *src, int len)
{
	for (int i = 0; i < len; i++) {
		des[i] ^= src[i];
	}

	return 0;
}

/* #define ALIGN4(x)           (x) = (((x) + 0x03) & ~0x03) */
#define ALIGN4(x) (x) = (x)
int frag_dec_init(frag_dec_t *obj)
{
	int i, j;

	i = 0;

	/* TODO: check if obj->cfg.dt is aligned */
	memset(obj->cfg.dt, 0, obj->cfg.maxlen);

	ALIGN4(i);
	obj->lost_frm_bm = (bm_t *)(obj->cfg.dt + i);
	i += (obj->cfg.nb + BM_UNIT - 1) / BM_UNIT * sizeof(bm_t);

	ALIGN4(i);
	obj->lost_frm_matrix_bm = (bm_t *)(obj->cfg.dt + i);
	/* left below of the matrix is useless compress used memory */
	i += (obj->cfg.tolerence * (obj->cfg.tolerence + 1) / 2 + BM_UNIT - 1) / BM_UNIT *
	     sizeof(bm_t);

	ALIGN4(i);
	obj->matched_lost_frm_bm0 = (bm_t *)(obj->cfg.dt + i);
	i += (obj->cfg.tolerence + BM_UNIT - 1) / BM_UNIT * sizeof(bm_t);

	ALIGN4(i);
	obj->matched_lost_frm_bm1 = (bm_t *)(obj->cfg.dt + i);
	i += (obj->cfg.tolerence + BM_UNIT - 1) / BM_UNIT * sizeof(bm_t);

	ALIGN4(i);
	obj->matrix_line_bm = (bm_t *)(obj->cfg.dt + i);
	i += (obj->cfg.nb + BM_UNIT - 1) / BM_UNIT * sizeof(bm_t);

	ALIGN4(i);
	obj->row_data_buf = obj->cfg.dt + i;
	i += obj->cfg.size;

	ALIGN4(i);
	obj->xor_row_data_buf = obj->cfg.dt + i;
	i += obj->cfg.size;

	ALIGN4(i);
	if (i > obj->cfg.maxlen) {
		return -1;
	}

	/* set all frame lost, from 0 to nb-1 */
	obj->lost_frm_count = obj->cfg.nb;
	for (j = 0; j < obj->cfg.nb; j++) {
		bit_set(obj->lost_frm_bm, j);
	}

	obj->filled_lost_frm_count = 0;
	obj->sta = FRAG_DEC_STA_UNCODED;

	return i;
}

void frag_dec_frame_received(frag_dec_t *obj, uint16_t index)
{
	if (bit_get(obj->lost_frm_bm, index)) {
		bit_clr(obj->lost_frm_bm, index);
		obj->lost_frm_count--;
		/* TODO: check and update other maps */
	}
}

void frag_dec_flash_wr(frag_dec_t *obj, uint16_t index, const uint8_t *buf)
{
	obj->cfg.fwr_func(index * obj->cfg.size, buf, obj->cfg.size);
	/*
	LOG_DBG("-> index %d, ", index);
	LOG_HEXDUMP_DBG(buf, obj->cfg.size);
	*/
}

void frag_dec_flash_rd(frag_dec_t *obj, uint16_t index, uint8_t *buf)
{
	obj->cfg.frd_func(index * obj->cfg.size, buf, obj->cfg.size);
	/*
	LOG_DBG("<- index %d, ", index);
	LOG_HEXDUMP_DBG(buf, obj->cfg.size);
	*/
}

void frag_dec_lost_frm_matrix_save(frag_dec_t *obj, uint16_t lindex, bm_t *map, int len)
{
	for (int i = 0; i < len; i++) {
		if (bit_get(map, i)) {
			m2t_set(obj->lost_frm_matrix_bm, i, lindex, len);
		} else {
			m2t_clr(obj->lost_frm_matrix_bm, i, lindex, len);
		}
	}
}

void frag_dec_lost_frm_matrix_load(frag_dec_t *obj, uint16_t lindex, bm_t *map, int len)
{
	for (int i = 0; i < len; i++) {
		if (m2t_get(obj->lost_frm_matrix_bm, i, lindex, len)) {
			bit_set(map, i);
		} else {
			bit_clr(map, i);
		}
	}
}

bool frag_dec_lost_frm_matrix_is_diagonal(frag_dec_t *obj, uint16_t lindex, int len)
{
	return m2t_get(obj->lost_frm_matrix_bm, lindex, lindex, len);
}

/* fcnt from 1 to nb */
int frag_dec(frag_dec_t *obj, uint16_t fcnt, const uint8_t *buf, int len)
{
	int i, j;
	int index, unmatched_frame_cnt;
	int lost_frame_index, frame_index, frame_index1;
	bool no_info;

	if (obj->sta == FRAG_DEC_STA_DONE) {
		return obj->lost_frm_count;
	}

	if (len != obj->cfg.size) {
		return FRAG_DEC_ERR_INVALID_FRAME;
	}

	/* clear all temporary bm and buf */
	bit_clear_all(obj->matched_lost_frm_bm0, obj->lost_frm_count);
	bit_clear_all(obj->matched_lost_frm_bm1, obj->lost_frm_count);

	/* back up input data so that not to mess input data */
	memcpy(obj->xor_row_data_buf, buf, obj->cfg.size);

	index = fcnt - 1;
	if ((index < obj->cfg.nb) && (obj->sta == FRAG_DEC_STA_UNCODED)) {
		/* uncoded frames under uncoded process */
		/* mark new received frame */
		frag_dec_frame_received(obj, index);
		/* save data to flash */
		frag_dec_flash_wr(obj, index, buf);
		/* if no frame lost finish decode process */
		if (obj->lost_frm_count == 0) {
			obj->sta = FRAG_DEC_STA_DONE;
			return obj->lost_frm_count;
		}
	} else {
		obj->sta = FRAG_DEC_STA_CODED;
		/* coded frames start processing, lost_frm_count is now frozen and should be not
		 * changed (!!!)
		 * too many packets are lost, it is not possible to reconstruct data block
		 */
		if (obj->lost_frm_count > obj->cfg.tolerence) {
			/* too many frames are lost, memory is not enough to reconstruct the
			 * packets
			 */
			return FRAG_DEC_ERR_TOO_MANY_FRAME_LOST;
		}
		unmatched_frame_cnt = 0;
		matrix_line_bm(obj->matrix_line_bm, index, obj->cfg.nb);
		for (i = 0; i < obj->cfg.nb; i++) {
			if (bit_get(obj->matrix_line_bm, i) == true) {
				if (bit_get(obj->lost_frm_bm, i) == false) {
					/* coded frame is matched one received uncoded frame */
					frag_dec_flash_rd(obj, i, obj->row_data_buf);
					buf_xor(obj->xor_row_data_buf, obj->row_data_buf,
						obj->cfg.size);
				} else {
					/* coded frame is not matched one received uncoded frame */
					/* matched_lost_frm_bm0 index is the nth lost frame */
					bit_set(obj->matched_lost_frm_bm0,
						bit_count_ones(obj->lost_frm_bm, i) - 1);
					unmatched_frame_cnt++;
				}
			}
		}
		if (unmatched_frame_cnt <= 0) {
			return FRAG_DEC_ONGOING;
		}

#ifdef DEBUG
		LOG_DBG("matrix_line_bm: %d, ", index);
		frag_dec_log_bits(obj->matrix_line_bm, obj->cfg.nb);
		LOG_DBG("matched_lost_frm_bm0: ");
		frag_dec_log_bits(obj->matched_lost_frm_bm0, obj->lost_frm_count);
#endif
		/* obj->matched_lost_frm_bm0 now saves new coded frame which excludes all received
		 * frames content start to diagonal obj->matched_lost_frm_bm0
		 */
		no_info = false;
		do {
			lost_frame_index = bit_ffs(obj->matched_lost_frm_bm0, obj->lost_frm_count);
			frame_index = bit_fns(obj->lost_frm_bm, obj->cfg.nb, lost_frame_index + 1);
			if (frame_index == -1) {
				//LOG_INF("matched_lost_frm_bm0: ");
				//frag_dec_log_bits(obj->matched_lost_frm_bm0, obj->lost_frm_count);
				//LOG_INF("lost_frm_bm: ");
				//frag_dec_log_bits(obj->lost_frm_bm, obj->cfg.nb);
				LOG_INF("frame_index: %d, lost_frame_index: %d\n", frame_index,
					lost_frame_index);
			}
#ifdef DEBUG
			LOG_DBG("matched_lost_frm_bm0: ");
			frag_dec_log_bits(obj->matched_lost_frm_bm0, obj->lost_frm_count);
			LOG_DBG("lost_frm_bm: ");
			frag_dec_log_bits(obj->lost_frm_bm, obj->cfg.nb);
			LOG_DBG("frame_index: %d, lost_frame_index: %d\n", frame_index,
				lost_frame_index);
#endif
			if (frag_dec_lost_frm_matrix_is_diagonal(obj, lost_frame_index,
								 obj->lost_frm_count) == false) {
				break;
			}

			frag_dec_lost_frm_matrix_load(obj, lost_frame_index,
						      obj->matched_lost_frm_bm1,
						      obj->lost_frm_count);
			bit_xor(obj->matched_lost_frm_bm0, obj->matched_lost_frm_bm1,
				obj->lost_frm_count);
			frag_dec_flash_rd(obj, frame_index, obj->row_data_buf);
			buf_xor(obj->xor_row_data_buf, obj->row_data_buf, obj->cfg.size);
			if (bit_is_all_clear(obj->matched_lost_frm_bm0, obj->lost_frm_count)) {
				no_info = true;
				break;
			}
		} while (1);
		if (!no_info) {
			/* current frame contains new information, save it */
			frag_dec_lost_frm_matrix_save(obj, lost_frame_index,
						      obj->matched_lost_frm_bm0,
						      obj->lost_frm_count);
			frag_dec_flash_wr(obj, frame_index, obj->xor_row_data_buf);
			obj->filled_lost_frm_count++;
		}
		if (obj->filled_lost_frm_count == obj->lost_frm_count) {
			/* all frame content is received, now to reconstruct the whole frame */
			if (obj->lost_frm_count > 1) {
				for (i = (obj->lost_frm_count - 2); i >= 0; i--) {
					frame_index = bit_fns(obj->lost_frm_bm, obj->cfg.nb, i + 1);
					frag_dec_flash_rd(obj, frame_index, obj->xor_row_data_buf);
					for (j = (obj->lost_frm_count - 1); j > i; j--) {
						frag_dec_lost_frm_matrix_load(
							obj, i, obj->matched_lost_frm_bm1,
							obj->lost_frm_count);
						frag_dec_lost_frm_matrix_load(
							obj, j, obj->matched_lost_frm_bm0,
							obj->lost_frm_count);
						if (bit_get(obj->matched_lost_frm_bm1, j)) {
							frame_index1 = bit_fns(obj->lost_frm_bm,
									       obj->cfg.nb, j + 1);
							frag_dec_flash_rd(obj, frame_index1,
									  obj->row_data_buf);
							bit_xor(obj->matched_lost_frm_bm1,
								obj->matched_lost_frm_bm0,
								obj->lost_frm_count);
							buf_xor(obj->xor_row_data_buf,
								obj->row_data_buf, obj->cfg.size);
							frag_dec_lost_frm_matrix_save(
								obj, i, obj->matched_lost_frm_bm1,
								obj->lost_frm_count);
						}
					}
					frag_dec_flash_wr(obj, frame_index, obj->xor_row_data_buf);
				}
			}
			obj->sta = FRAG_DEC_STA_DONE;
			return obj->lost_frm_count;
		}
	}
	/* process ongoing */
	return FRAG_DEC_ONGOING;
}

void frag_dec_log_buf(const uint8_t *buf, int len)
{
	for (int i = 0; i < len; i++) {
		printf("%02X, ", buf[i]);
	}
	printf("\n");
}

void frag_dec_log_bits(bm_t *bitmap, int len)
{
	for (int i = 0; i < len; i++) {
		if (bit_get(bitmap, i)) {
			printf("1 ");
		} else {
			printf("0 ");
		}
	}
	printf("\n");
}

void frag_dec_log_matrix_bits(bm_t *bitmap, int len)
{
	int i, j;

	for (i = 0; i < len; i++) {
		for (j = 0; j < len; j++) {
			if (m2t_get(bitmap, j, i, len)) {
				printf("1 ");
			} else {
				printf("0 ");
			}
		}
		printf("\n");
	}
	if (i == 0) {
		printf("\n");
	}
}

void frag_dec_log(frag_dec_t *obj)
{
	int i, j;

	printf("Decode %s\n", (obj->sta == FRAG_DEC_STA_DONE) ? "ok" : "ng");
	for (i = 0; i < obj->cfg.nb; i++) {
		frag_dec_flash_rd(obj, i, obj->row_data_buf);
		for (j = 0; j < obj->cfg.size; j++) {
			printf("%02X ", obj->row_data_buf[j]);
		}
		printf("\n");
	}

	printf("lost_frm_matrix_bm: (%d)\n", obj->lost_frm_count);
	frag_dec_log_matrix_bits(obj->lost_frm_matrix_bm, obj->lost_frm_count);
}
