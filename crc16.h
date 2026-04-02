/**
 * @file    crc16.h
 * @brief   CRC-16/CCITT implementation for frame integrity
 * @author  ARYA mgc
 * @version 1.0.0
 * @date    2025
 */

#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Compute CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)
 * @param data  Pointer to data buffer
 * @param len   Number of bytes
 * @return      16-bit CRC value
 */
static inline uint16_t crc16_compute(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

/**
 * @brief Compute CRC over a NetworkFrame_t (excludes start, crc, end fields)
 * @param frame  Pointer to the frame
 * @return       16-bit CRC
 *
 * @note Coverage: src_id, dst_id, msg_type, seq_num, length, payload[0..length-1]
 */
#define CRC_FIELD_OFFSET   (offsetof(NetworkFrame_t, src_id))
#define CRC_HEADER_SIZE    (offsetof(NetworkFrame_t, payload) - CRC_FIELD_OFFSET)

static inline uint16_t frame_crc(const NetworkFrame_t *frame)
{
    uint16_t crc;
    /* Hash header fields */
    crc = crc16_compute((const uint8_t *)frame + CRC_FIELD_OFFSET, CRC_HEADER_SIZE);
    /* Continue over actual payload bytes only */
    for (size_t i = 0; i < frame->length; i++) {
        uint8_t b = frame->payload[i];
        crc ^= (uint16_t)b << 8;
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
            else              crc <<= 1;
        }
    }
    return crc;
}

#endif /* CRC16_H */
