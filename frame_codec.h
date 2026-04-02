/**
 * @file    frame_codec.h
 * @brief   UART frame encode / decode for the sensor network protocol
 * @author  ARYA mgc
 * @version 1.0.0
 * @date    2025
 */

#ifndef FRAME_CODEC_H
#define FRAME_CODEC_H

#include <string.h>
#include "network_protocol.h"
#include "crc16.h"

/* ============================================================
 *  Return codes
 * ============================================================ */

typedef enum {
    CODEC_OK            =  0,
    CODEC_ERR_NULL      = -1,
    CODEC_ERR_LENGTH    = -2,
    CODEC_ERR_START     = -3,
    CODEC_ERR_END       = -4,
    CODEC_ERR_CRC       = -5,
    CODEC_ERR_OVERFLOW  = -6,
} CodecResult_t;

/* ============================================================
 *  Sequence counter (per-node, static)
 * ============================================================ */

static uint16_t _seq_counter = 0;

/* ============================================================
 *  ENCODER
 * ============================================================ */

/**
 * @brief Build a complete NetworkFrame_t ready to transmit
 *
 * @param[out] frame      Output frame buffer
 * @param[in]  src        Source NodeID_t
 * @param[in]  dst        Destination NodeID_t (or BROADCAST_ID)
 * @param[in]  msg_type   MessageType_t
 * @param[in]  payload    Pointer to payload data (may be NULL if len=0)
 * @param[in]  length     Payload length (0..MAX_PAYLOAD_SIZE)
 * @return     CODEC_OK or error code
 */
static inline CodecResult_t frame_encode(
    NetworkFrame_t *frame,
    uint8_t         src,
    uint8_t         dst,
    uint8_t         msg_type,
    const void     *payload,
    uint8_t         length)
{
    if (!frame) return CODEC_ERR_NULL;
    if (length > MAX_PAYLOAD_SIZE) return CODEC_ERR_OVERFLOW;

    frame->start    = FRAME_START_BYTE;
    frame->src_id   = src;
    frame->dst_id   = dst;
    frame->msg_type = msg_type;
    frame->seq_num  = _seq_counter++;
    frame->length   = length;

    if (payload && length > 0)
        memcpy(frame->payload, payload, length);
    else
        memset(frame->payload, 0, MAX_PAYLOAD_SIZE);

    frame->crc16 = frame_crc(frame);
    frame->end   = FRAME_END_BYTE;

    return CODEC_OK;
}

/* ============================================================
 *  DECODER
 * ============================================================ */

/**
 * @brief Validate and decode a received frame
 *
 * @param[in]  frame   Pointer to the received frame
 * @return     CODEC_OK on success, error code on failure
 *
 * @note Does NOT copy payload – caller accesses frame->payload directly.
 */
static inline CodecResult_t frame_decode(const NetworkFrame_t *frame)
{
    if (!frame) return CODEC_ERR_NULL;
    if (frame->start != FRAME_START_BYTE) return CODEC_ERR_START;
    if (frame->end   != FRAME_END_BYTE)   return CODEC_ERR_END;
    if (frame->length > MAX_PAYLOAD_SIZE) return CODEC_ERR_LENGTH;

    uint16_t expected = frame_crc(frame);
    if (frame->crc16 != expected) return CODEC_ERR_CRC;

    return CODEC_OK;
}

/* ============================================================
 *  SERIALIZER  (frame → flat byte array for UART TX)
 * ============================================================ */

/**
 * @brief Serialize a frame into a raw byte buffer for UART transmission
 *
 * @param[in]  frame   Source frame
 * @param[out] buf     Output byte buffer
 * @param[in]  buf_sz  Size of output buffer
 * @return     Number of bytes written, or negative error code
 */
static inline int frame_serialize(const NetworkFrame_t *frame,
                                   uint8_t *buf, size_t buf_sz)
{
    /* Wire size: 1(start) + 1(src) + 1(dst) + 1(type) + 2(seq)
     *            + 1(len) + length(payload) + 2(crc) + 1(end)  */
    size_t wire_sz = 10 + frame->length;
    if (buf_sz < wire_sz) return CODEC_ERR_OVERFLOW;

    size_t i = 0;
    buf[i++] = frame->start;
    buf[i++] = frame->src_id;
    buf[i++] = frame->dst_id;
    buf[i++] = frame->msg_type;
    buf[i++] = (uint8_t)(frame->seq_num >> 8);
    buf[i++] = (uint8_t)(frame->seq_num & 0xFF);
    buf[i++] = frame->length;
    memcpy(&buf[i], frame->payload, frame->length);
    i += frame->length;
    buf[i++] = (uint8_t)(frame->crc16 >> 8);
    buf[i++] = (uint8_t)(frame->crc16 & 0xFF);
    buf[i++] = frame->end;

    return (int)wire_sz;
}

#endif /* FRAME_CODEC_H */
