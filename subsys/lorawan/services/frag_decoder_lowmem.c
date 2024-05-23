/*
 * Copyright (c) 2024 A Labs GmbH
 * Copyright (c) 2022 Jiapeng Li
 *
 * Based on: https://github.com/JiapengLi/LoRaWANFragmentedDataBlockTransportAlgorithm
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "frag_decoder_lowmem.h"
#include "frag_flash.h"

#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/bitarray.h>

LOG_MODULE_REGISTER(lorawan_frag_dec, CONFIG_LORAWAN_SERVICES_LOG_LEVEL);

SYS_BITARRAY_DEFINE_STATIC(lost_frm_bm, FRAG_MAX_NB);
SYS_BITARRAY_DEFINE_STATIC(lost_frm_matrix_bm, (FRAG_TOLERANCE * (FRAG_TOLERANCE + 1) / 2));
SYS_BITARRAY_DEFINE_STATIC(matched_lost_frm_bm0, FRAG_TOLERANCE);
SYS_BITARRAY_DEFINE_STATIC(matched_lost_frm_bm1, FRAG_TOLERANCE);
SYS_BITARRAY_DEFINE_STATIC(matrix_line_bm, FRAG_MAX_NB);

static inline size_t matrix_location_to_index(size_t x, size_t y, size_t m);

static bool triangular_matrix_get_entry(struct sys_bitarray *m2tbm, size_t x, size_t y, size_t m);
static void triangular_matrix_set_entry(struct sys_bitarray *m2tbm, size_t x, size_t y, size_t m);
static void triangular_matrix_clear_entry(struct sys_bitarray *m2tbm, size_t x, size_t y, size_t m);

static bool bit_get(struct sys_bitarray *bitmap, size_t index);
static void bit_set(struct sys_bitarray *bitmap, size_t index);
static void bit_clear(struct sys_bitarray *bitmap, size_t index);
static size_t bit_count_ones(struct sys_bitarray *bitmap, size_t index);
static void bit_xor(struct sys_bitarray *des, struct sys_bitarray *src, size_t size);
static void bit_clear_all(struct sys_bitarray *bitmap, size_t size);

/**
 * Generate a 23bit Pseudorandom Binary Sequence (PRBS)
 *
 * @param seed Seed input value
 *
 * @returns Pseudorandom output value
 */
static int32_t prbs23(int32_t seed);

/**
 * Generate vector for coded fragment n of the MxN parity matrix
 *
 * @param m Total number of uncoded fragments (M)
 * @param n Coded fragment number (starting at 1 and not 0)
 * @param vec Output vector (buffer size must be greater than m)
 */
static void lorawan_fec_parity_matrix_vector(size_t m, size_t n, struct sys_bitarray *vec);

static void frag_dec_frame_received(frag_dec_t *decoder, uint16_t index);
static void frag_dec_write_line(struct sys_bitarray *matrix, uint16_t line_index,
				struct sys_bitarray *vector, size_t len);
static void frag_dec_read_line(struct sys_bitarray *matrix, uint16_t line_index,
			       struct sys_bitarray *vector, size_t len);

int frag_dec(frag_dec_t *decoder, uint16_t frameCounter, const uint8_t *buf, size_t len)
{
	int ret;
	int i, j;
	size_t index, unmatched_frame_cnt;
	size_t lost_frame_index, frame_index;
	static uint8_t row_data_buf[FRAG_MAX_SIZE];
	static uint8_t xor_row_data_buf[FRAG_MAX_SIZE];

	if (decoder->status == FRAG_DEC_STA_DONE) {
		return decoder->lost_frame_count;
	}

	if (len != decoder->cfg.frag_size) {
		return FRAG_DEC_ERR_INVALID_FRAME;
	}

	__ASSERT_NO_MSG(frameCounter > 0);
	index = frameCounter - 1;

	if ((index < decoder->cfg.nb_frag) && (decoder->status == FRAG_DEC_STA_UNCODED)) {
		/* uncoded frames under uncoded process */
		/* mark new received frame */
		frag_dec_frame_received(decoder, index);
		/* save data to flash */
		frag_flash_write(index * decoder->cfg.frag_size, (uint8_t *)buf,
				 decoder->cfg.frag_size);
		/* if no frame lost finish decode process */
		if (decoder->lost_frame_count == 0) {
			decoder->status = FRAG_DEC_STA_DONE;
			return decoder->lost_frame_count;
		}
		return FRAG_DEC_ONGOING;
	}

	/* clear all temporary bm and buf */
	bit_clear_all(&matched_lost_frm_bm0, decoder->lost_frame_count);
	bit_clear_all(&matched_lost_frm_bm1, decoder->lost_frame_count);

	/* back up input data so that not to mess input data */
	memcpy(xor_row_data_buf, buf, decoder->cfg.frag_size);

	decoder->status = FRAG_DEC_STA_CODED;
	/* coded frames start processing, lost_frame_count is now frozen and should be not
	 * changed (!!!)
	 * too many packets are lost, it is not possible to reconstruct data block
	 */
	if (decoder->lost_frame_count > FRAG_TOLERANCE) {
		/* too many frames are lost, memory is not enough to reconstruct the
		 * packets
		 */
		return FRAG_DEC_ERR_TOO_MANY_FRAME_LOST;
	}
	unmatched_frame_cnt = 0;
	/* build parity matrix vector for current line */
	lorawan_fec_parity_matrix_vector(decoder->cfg.nb_frag, frameCounter - decoder->cfg.nb_frag,
					 &matrix_line_bm);
	for (i = 0; i < decoder->cfg.nb_frag; i++) {
		if (!bit_get(&matrix_line_bm, i)) {
			continue;
		}
		if (bit_get(&lost_frm_bm, i)) {
			/* coded frame is not matched one received uncoded frame */
			/* matched_lost_frm_bm0 index is the nth lost frame */
			bit_set(&matched_lost_frm_bm0, bit_count_ones(&lost_frm_bm, i) - 1);
			unmatched_frame_cnt++;
		} else {
			/* restore frame by xoring with already received frame */
			/* load frame with index i into row_data_buf */
			frag_flash_read(i * decoder->cfg.frag_size, row_data_buf,
					decoder->cfg.frag_size);
			/* xor previously received frame with data for current frame */
			mem_xor_n(xor_row_data_buf, xor_row_data_buf, row_data_buf,
				  decoder->cfg.frag_size);
		}
	}
	if (unmatched_frame_cnt == 0) {
		return FRAG_DEC_ONGOING;
	}

	/* &matched_lost_frm_bm0 now saves new coded frame which excludes all received
	 * frames content start to diagonal &matched_lost_frm_bm0
	 */
	do {
		ret = sys_bitarray_find_nth_set(&matched_lost_frm_bm0, 1, decoder->lost_frame_count,
						0, &lost_frame_index);
		if (ret == 1) {
			/* not found */
			break;
		}
		if (ret != 0) {
			return FRAG_DEC_ERR;
		}
		/** we know which one is the next lost frame, try to find it in the lost frame bm */
		ret = sys_bitarray_find_nth_set(&lost_frm_bm, lost_frame_index + 1,
						decoder->cfg.nb_frag, 0, &frame_index);
		if (ret == 1) {
			/* not found */
			break;
		}
		if (ret != 0) {
			return FRAG_DEC_ERR;
		}
		/* if current frame contains new information, save it */
		if (!triangular_matrix_get_entry(&lost_frm_matrix_bm, lost_frame_index,
						 lost_frame_index, decoder->lost_frame_count)) {
			frag_dec_write_line(&lost_frm_matrix_bm, lost_frame_index,
					    &matched_lost_frm_bm0, decoder->lost_frame_count);
			frag_flash_write(frame_index * decoder->cfg.frag_size, xor_row_data_buf,
					 decoder->cfg.frag_size);
			decoder->filled_lost_frm_count++;
			break;
		}

		frag_dec_read_line(&lost_frm_matrix_bm, lost_frame_index, &matched_lost_frm_bm1,
				   decoder->lost_frame_count);
		bit_xor(&matched_lost_frm_bm0, &matched_lost_frm_bm1, decoder->lost_frame_count);
		frag_flash_read(frame_index * decoder->cfg.frag_size, row_data_buf,
				decoder->cfg.frag_size);
		mem_xor_n(xor_row_data_buf, xor_row_data_buf, row_data_buf, decoder->cfg.frag_size);
	} while (!sys_bitarray_is_region_cleared(&matched_lost_frm_bm0, decoder->lost_frame_count,
						 0));

	if (decoder->filled_lost_frm_count != decoder->lost_frame_count ||
	    decoder->lost_frame_count < 2) {
		return FRAG_DEC_ONGOING;
	}

	/* all frame content is received, now to reconstruct the whole frame */
	for (i = (decoder->lost_frame_count - 2); i >= 0; i--) {
		ret = sys_bitarray_find_nth_set(&lost_frm_bm, i + 1, decoder->cfg.nb_frag, 0,
						&frame_index);
		if (ret != 0) {
			return FRAG_DEC_ERR;
		}
		frag_flash_read(frame_index * decoder->cfg.frag_size, xor_row_data_buf,
				decoder->cfg.frag_size);
		frag_dec_read_line(&lost_frm_matrix_bm, i, &matched_lost_frm_bm1,
				   decoder->lost_frame_count);
		for (j = (decoder->lost_frame_count - 1); j > i; j--) {
			if (!bit_get(&matched_lost_frm_bm1, j)) {
				continue;
			}
			ret = sys_bitarray_find_nth_set(&lost_frm_bm, j + 1, decoder->cfg.nb_frag,
							0, &lost_frame_index);
			if (ret != 0) {
				return FRAG_DEC_ERR;
			}
			frag_dec_read_line(&lost_frm_matrix_bm, j, &matched_lost_frm_bm0,
					   decoder->lost_frame_count);
			bit_xor(&matched_lost_frm_bm1, &matched_lost_frm_bm0,
				decoder->lost_frame_count);
			frag_flash_read(lost_frame_index * decoder->cfg.frag_size, row_data_buf,
					decoder->cfg.frag_size);
			mem_xor_n(xor_row_data_buf, xor_row_data_buf, row_data_buf,
				  decoder->cfg.frag_size);
			frag_dec_write_line(&lost_frm_matrix_bm, i, &matched_lost_frm_bm1,
					    decoder->lost_frame_count);
		}
		frag_flash_write(frame_index * decoder->cfg.frag_size, xor_row_data_buf,
				 decoder->cfg.frag_size);
	}
	decoder->status = FRAG_DEC_STA_DONE;
	return decoder->lost_frame_count;
}

/**
 * Map col (@p x) and row (@p y) of matrix with size @p m to 'flat' index in buffer.
 * @param x
 * @param y
 * @param m
 * @return index in flat buffer
 */
static inline size_t matrix_location_to_index(size_t x, size_t y, size_t m)
{
	return y * m + x;
}

static bool triangular_matrix_get_entry(struct sys_bitarray *m2tbm, size_t x, size_t y, size_t m)
{
	/* we are dealing with triangular matrices, so we don't expect actions in the lower half */
	__ASSERT(x >= y, "x: %d, y: %d, m: %d", x, y, m);
	size_t bit;
	int ret;

	ret = sys_bitarray_test_bit(m2tbm, matrix_location_to_index(x, y, m), &bit);
	__ASSERT_NO_MSG(ret == 0);

	return bit != 0;
}

static void triangular_matrix_set_entry(struct sys_bitarray *m2tbm, size_t x, size_t y, size_t m)
{
	/* we are dealing with triangular matrices, so we don't expect actions in the lower half */
	__ASSERT(x >= y, "x: %d, y: %d, m: %d", x, y, m);
	sys_bitarray_set_bit(m2tbm, matrix_location_to_index(x, y, m));
}

static void triangular_matrix_clear_entry(struct sys_bitarray *m2tbm, size_t x, size_t y, size_t m)
{
	/* we are dealing with triangular matrices, so we don't expect actions in the lower half */
	__ASSERT(x >= y, "x: %d, y: %d, m: %d", x, y, m);

	int ret;

	ret = sys_bitarray_clear_bit(m2tbm, matrix_location_to_index(x, y, m));
	__ASSERT_NO_MSG(ret == 0);
}

static bool bit_get(struct sys_bitarray *bitmap, size_t index)
{
	int lost_frm_bit, ret;

	ret = sys_bitarray_test_bit(bitmap, index, &lost_frm_bit);
	__ASSERT_NO_MSG(ret == 0);
	return lost_frm_bit != 0;
}

static void bit_set(struct sys_bitarray *bitmap, size_t index)
{
	int ret;

	ret = sys_bitarray_set_bit(bitmap, index);
	__ASSERT_NO_MSG(ret == 0);
}

static void bit_clear(struct sys_bitarray *bitmap, size_t index)
{
	int ret;

	ret = sys_bitarray_clear_bit(bitmap, index);
	__ASSERT_NO_MSG(ret == 0);
}

static size_t bit_count_ones(struct sys_bitarray *bitmap, size_t index)
{
	size_t count;
	int ret;

	ret = sys_bitarray_popcount_region(bitmap, index + 1, 0, &count);
	__ASSERT_NO_MSG(ret == 0);
	return count;
}

static void bit_xor(struct sys_bitarray *des, struct sys_bitarray *src, size_t size)
{
	int ret;

	ret = sys_bitarray_xor(des, src, size, 0);
	__ASSERT_NO_MSG(ret == 0);
}

static void bit_clear_all(struct sys_bitarray *bitmap, size_t size)
{
	int ret;

	ret = sys_bitarray_clear_region(bitmap, size, 0);
	__ASSERT_NO_MSG(ret == 0);
}

/**
 * Generate a 23bit Pseudorandom Binary Sequence (PRBS)
 *
 * @param seed Seed input value
 *
 * @returns Pseudorandom output value
 */
static int32_t prbs23(int32_t seed)
{
	int32_t b0 = seed & 1;
	int32_t b1 = (seed & 32) / 32;

	return (seed / 2) + ((b0 ^ b1) << 22);
}

/**
 * Generate vector for coded fragment n of the MxN parity matrix
 *
 * @param m Total number of uncoded fragments (M)
 * @param n Coded fragment number (starting at 1 and not 0)
 * @param vec Output vector (buffer size must be greater than m)
 */
static void lorawan_fec_parity_matrix_vector(size_t m, size_t n, struct sys_bitarray *vec)
{
	size_t mm, r;
	int32_t x;
	int ret;

	ret = sys_bitarray_clear_region(vec, m, 0);
	__ASSERT_NO_MSG(ret == 0);

	/*
	 * Powers of 2 must be treated differently to make sure matrix content is close
	 * to random. Powers of 2 tend to generate patterns.
	 */
	if (is_power_of_two(m)) {
		mm = m + 1;
	} else {
		mm = m;
	}

	x = 1 + (1001 * n);

	for (size_t nb_coeff = 0; nb_coeff < (m / 2); nb_coeff++) {
		r = (1 << 16);
		while (r >= m) {
			x = prbs23(x);
			r = x % mm;
		}
		ret = sys_bitarray_set_bit(vec, r);
		__ASSERT_NO_MSG(ret == 0);
	}
}

void frag_dec_init(frag_dec_t *decoder)
{
	size_t j;

	/* set all frame lost, from 0 to nb_frag-1 */
	decoder->lost_frame_count = decoder->cfg.nb_frag;
	for (j = 0; j < decoder->cfg.nb_frag; j++) {
		bit_set(&lost_frm_bm, j);
	}

	decoder->filled_lost_frm_count = 0;
	decoder->status = FRAG_DEC_STA_UNCODED;
}

void frag_dec_frame_received(frag_dec_t *decoder, uint16_t index)
{
	if (bit_get(&lost_frm_bm, index)) {
		bit_clear(&lost_frm_bm, index);
		decoder->lost_frame_count--;
		/* TODO: check and update other maps */
	}
}

static void frag_dec_write_line(struct sys_bitarray *matrix, uint16_t line_index,
				struct sys_bitarray *vector, size_t len)
{
	for (size_t i = line_index; i < len; i++) {
		if (bit_get(vector, i)) {
			triangular_matrix_set_entry(matrix, i, line_index, len);
		} else {
			triangular_matrix_clear_entry(matrix, i, line_index, len);
		}
	}
}

static void frag_dec_read_line(struct sys_bitarray *matrix, uint16_t line_index,
			       struct sys_bitarray *vector, size_t len)
{
	for (size_t i = 0; i < len; i++) {
		if (i >= line_index && triangular_matrix_get_entry(matrix, i, line_index, len)) {
			bit_set(vector, i);
		} else {
			bit_clear(vector, i);
		}
	}
}
