#ifndef __CRC_CCITT_H
#define __CRC_CCITT_H

#include "main.h"


uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c);

/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len);

// CRSF: CRC-8 poly 0xD5, init 0x00, xorout 0x00, refin/refout=false (MSB-first)
uint8_t crsf_crc8_compute(const uint8_t *data, size_t len);

// 校验一整帧（buf[0]=0xC8, buf[1]=size, buf[2]=type ... buf[frame_len-1]=crc）
int crsf_frame_crc_ok(const uint8_t *buf, size_t frame_len);

#endif
