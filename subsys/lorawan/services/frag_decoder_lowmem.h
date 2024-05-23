/*
 * Copyright (c) 2024 A Labs GmbH
 * Copyright (c) 2022 Jiapeng Li
 *
 * Based on: https://github.com/JiapengLi/LoRaWANFragmentedDataBlockTransportAlgorithm
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef FRAG_DEC_H_
#define FRAG_DEC_H_

#include <stdint.h>
#include <stddef.h>

#define FRAG_MAX_NB                                                                                \
	(CONFIG_LORAWAN_FRAG_TRANSPORT_IMAGE_SIZE / CONFIG_LORAWAN_FRAG_TRANSPORT_MIN_FRAG_SIZE +  \
	 1U)
#define FRAG_MAX_SIZE  (CONFIG_LORAWAN_FRAG_TRANSPORT_MAX_FRAG_SIZE)
#define FRAG_TOLERANCE (FRAG_MAX_NB * CONFIG_LORAWAN_FRAG_TRANSPORT_MAX_REDUNDANCY / 100U)

/*
 * https://github.com/brocaar/lorawan/blob/master/applayer/fragmentation/encode.go
 * https://github.com/brocaar/lorawan/blob/master/applayer/fragmentation/encode_test.go
 */

#define FRAG_DEC_ONGOING                 (-1)
#define FRAG_DEC_ERR_INVALID_FRAME       (-2)
#define FRAG_DEC_ERR_TOO_MANY_FRAME_LOST (-3)
#define FRAG_DEC_ERR                     (-4)

typedef struct {
	/** number of fragments */
	uint16_t nb_frag;
	uint8_t frag_size;
} frag_dec_cfg_t;

typedef enum {
	/* wait uncoded fragmentations */
	FRAG_DEC_STA_UNCODED,
	/* wait coded fragmentations, uncoded frags are processed as coded ones */
	FRAG_DEC_STA_CODED,
	FRAG_DEC_STA_DONE,
} frag_dec_sta_t;

typedef struct {
	frag_dec_cfg_t cfg;
	frag_dec_sta_t status;

	uint16_t lost_frame_count;
	uint16_t filled_lost_frm_count;
} frag_dec_t;

void frag_dec_init(frag_dec_t *decoder);
int frag_dec(frag_dec_t *decoder, uint16_t frameCounter, const uint8_t *buf, size_t len);

#endif /* FRAG_DEC_H_ */
