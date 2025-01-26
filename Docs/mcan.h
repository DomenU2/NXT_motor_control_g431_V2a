/**
 * The MIT License (MIT)
 *
 * Copyright (c) 2018-2019 Erik Moqvist
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * This file was generated by cantools version 39.4.13 Sat Jan 25 22:27:40 2025.
 */

#ifndef MCAN_H
#define MCAN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifndef EINVAL
#    define EINVAL 22
#endif

/* Frame ids. */
#define MCAN_M_COMMAND_1_FRAME_ID (0x413u)
#define MCAN_M_STATUS_1_FRAME_ID (0x412u)
#define MCAN_M_REF_1_FRAME_ID (0x411u)
#define MCAN_M_POS_VEL_1_FRAME_ID (0x410u)
#define MCAN_M_COMMAND_2_FRAME_ID (0x423u)
#define MCAN_M_STATUS_2_FRAME_ID (0x422u)
#define MCAN_M_REF_2_FRAME_ID (0x421u)
#define MCAN_M_POS_VEL_2_FRAME_ID (0x420u)

/* Frame lengths in bytes. */
#define MCAN_M_COMMAND_1_LENGTH (8u)
#define MCAN_M_STATUS_1_LENGTH (8u)
#define MCAN_M_REF_1_LENGTH (8u)
#define MCAN_M_POS_VEL_1_LENGTH (8u)
#define MCAN_M_COMMAND_2_LENGTH (8u)
#define MCAN_M_STATUS_2_LENGTH (8u)
#define MCAN_M_REF_2_LENGTH (8u)
#define MCAN_M_POS_VEL_2_LENGTH (8u)

/* Extended or standard frame types. */
#define MCAN_M_COMMAND_1_IS_EXTENDED (0)
#define MCAN_M_STATUS_1_IS_EXTENDED (0)
#define MCAN_M_REF_1_IS_EXTENDED (0)
#define MCAN_M_POS_VEL_1_IS_EXTENDED (0)
#define MCAN_M_COMMAND_2_IS_EXTENDED (0)
#define MCAN_M_STATUS_2_IS_EXTENDED (0)
#define MCAN_M_REF_2_IS_EXTENDED (0)
#define MCAN_M_POS_VEL_2_IS_EXTENDED (0)

/* Frame cycle times in milliseconds. */


/* Signal choices. */


/* Frame Names. */
#define MCAN_M_COMMAND_1_NAME "m_command_1"
#define MCAN_M_STATUS_1_NAME "m_status_1"
#define MCAN_M_REF_1_NAME "m_ref_1"
#define MCAN_M_POS_VEL_1_NAME "m_pos_vel_1"
#define MCAN_M_COMMAND_2_NAME "m_command_2"
#define MCAN_M_STATUS_2_NAME "m_status_2"
#define MCAN_M_REF_2_NAME "m_ref_2"
#define MCAN_M_POS_VEL_2_NAME "m_pos_vel_2"

/* Signal Names. */
#define MCAN_M_COMMAND_1_M_DRIVE_EN_NAME "m_drive_en"
#define MCAN_M_COMMAND_1_M_EN_NAME "m_en"
#define MCAN_M_COMMAND_1_M_MODE_NAME "m_mode"
#define MCAN_M_STATUS_1_M_DRIVE_EN_NAME "m_drive_en"
#define MCAN_M_STATUS_1_M_EN_NAME "m_en"
#define MCAN_M_STATUS_1_M_MODE_NAME "m_mode"
#define MCAN_M_STATUS_1_M_ERROR_NAME "m_error"
#define MCAN_M_REF_1_POSITION_NAME "position"
#define MCAN_M_REF_1_VELOCITY_NAME "velocity"
#define MCAN_M_POS_VEL_1_POSITION_NAME "position"
#define MCAN_M_POS_VEL_1_VELOCITY_NAME "velocity"
#define MCAN_M_COMMAND_2_M_DRIVE_EN_NAME "m_drive_en"
#define MCAN_M_COMMAND_2_M_EN_NAME "m_en"
#define MCAN_M_COMMAND_2_M_MODE_NAME "m_mode"
#define MCAN_M_STATUS_2_M_DRIVE_EN_NAME "m_drive_en"
#define MCAN_M_STATUS_2_M_EN_NAME "m_en"
#define MCAN_M_STATUS_2_M_MODE_NAME "m_mode"
#define MCAN_M_STATUS_2_M_ERROR_NAME "m_error"
#define MCAN_M_REF_2_POSITION_NAME "position"
#define MCAN_M_REF_2_VELOCITY_NAME "velocity"
#define MCAN_M_POS_VEL_2_POSITION_NAME "position"
#define MCAN_M_POS_VEL_2_VELOCITY_NAME "velocity"

/**
 * Signals in message m_command_1.
 *
 * All signal values are as on the CAN bus.
 */
struct mcan_m_command_1_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_drive_en;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_en;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_mode;
};

/**
 * Signals in message m_status_1.
 *
 * All signal values are as on the CAN bus.
 */
struct mcan_m_status_1_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_drive_en;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_en;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_mode;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_error;
};

/**
 * Signals in message m_ref_1.
 *
 * All signal values are as on the CAN bus.
 */
struct mcan_m_ref_1_t {
    /**
     * Range: -
     * Scale: 0.001
     * Offset: 0
     */
    int32_t position;

    /**
     * Range: -
     * Scale: 0.001
     * Offset: 0
     */
    int32_t velocity;
};

/**
 * Signals in message m_pos_vel_1.
 *
 * All signal values are as on the CAN bus.
 */
struct mcan_m_pos_vel_1_t {
    /**
     * Range: -
     * Scale: 0.001
     * Offset: 0
     */
    int32_t position;

    /**
     * Range: -
     * Scale: 0.001
     * Offset: 0
     */
    int32_t velocity;
};

/**
 * Signals in message m_command_2.
 *
 * All signal values are as on the CAN bus.
 */
struct mcan_m_command_2_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_drive_en;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_en;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_mode;
};

/**
 * Signals in message m_status_2.
 *
 * All signal values are as on the CAN bus.
 */
struct mcan_m_status_2_t {
    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_drive_en;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_en;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_mode;

    /**
     * Range: -
     * Scale: 1
     * Offset: 0
     */
    uint8_t m_error;
};

/**
 * Signals in message m_ref_2.
 *
 * All signal values are as on the CAN bus.
 */
struct mcan_m_ref_2_t {
    /**
     * Range: -
     * Scale: 0.001
     * Offset: 0
     */
    int32_t position;

    /**
     * Range: -
     * Scale: 0.001
     * Offset: 0
     */
    int32_t velocity;
};

/**
 * Signals in message m_pos_vel_2.
 *
 * All signal values are as on the CAN bus.
 */
struct mcan_m_pos_vel_2_t {
    /**
     * Range: -
     * Scale: 0.001
     * Offset: 0
     */
    int32_t position;

    /**
     * Range: -
     * Scale: 0.001
     * Offset: 0
     */
    int32_t velocity;
};

/**
 * Pack message m_command_1.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int mcan_m_command_1_pack(
    uint8_t *dst_p,
    const struct mcan_m_command_1_t *src_p,
    size_t size);

/**
 * Unpack message m_command_1.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int mcan_m_command_1_unpack(
    struct mcan_m_command_1_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from m_command_1.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int mcan_m_command_1_init(struct mcan_m_command_1_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_command_1_m_drive_en_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_command_1_m_drive_en_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_command_1_m_drive_en_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_command_1_m_en_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_command_1_m_en_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_command_1_m_en_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_command_1_m_mode_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_command_1_m_mode_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_command_1_m_mode_is_in_range(uint8_t value);

/**
 * Pack message m_status_1.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int mcan_m_status_1_pack(
    uint8_t *dst_p,
    const struct mcan_m_status_1_t *src_p,
    size_t size);

/**
 * Unpack message m_status_1.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int mcan_m_status_1_unpack(
    struct mcan_m_status_1_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from m_status_1.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int mcan_m_status_1_init(struct mcan_m_status_1_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_status_1_m_drive_en_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_status_1_m_drive_en_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_status_1_m_drive_en_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_status_1_m_en_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_status_1_m_en_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_status_1_m_en_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_status_1_m_mode_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_status_1_m_mode_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_status_1_m_mode_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_status_1_m_error_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_status_1_m_error_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_status_1_m_error_is_in_range(uint8_t value);

/**
 * Pack message m_ref_1.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int mcan_m_ref_1_pack(
    uint8_t *dst_p,
    const struct mcan_m_ref_1_t *src_p,
    size_t size);

/**
 * Unpack message m_ref_1.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int mcan_m_ref_1_unpack(
    struct mcan_m_ref_1_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from m_ref_1.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int mcan_m_ref_1_init(struct mcan_m_ref_1_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int32_t mcan_m_ref_1_position_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_ref_1_position_decode(int32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_ref_1_position_is_in_range(int32_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int32_t mcan_m_ref_1_velocity_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_ref_1_velocity_decode(int32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_ref_1_velocity_is_in_range(int32_t value);

/**
 * Pack message m_pos_vel_1.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int mcan_m_pos_vel_1_pack(
    uint8_t *dst_p,
    const struct mcan_m_pos_vel_1_t *src_p,
    size_t size);

/**
 * Unpack message m_pos_vel_1.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int mcan_m_pos_vel_1_unpack(
    struct mcan_m_pos_vel_1_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from m_pos_vel_1.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int mcan_m_pos_vel_1_init(struct mcan_m_pos_vel_1_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int32_t mcan_m_pos_vel_1_position_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_pos_vel_1_position_decode(int32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_pos_vel_1_position_is_in_range(int32_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int32_t mcan_m_pos_vel_1_velocity_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_pos_vel_1_velocity_decode(int32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_pos_vel_1_velocity_is_in_range(int32_t value);

/**
 * Pack message m_command_2.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int mcan_m_command_2_pack(
    uint8_t *dst_p,
    const struct mcan_m_command_2_t *src_p,
    size_t size);

/**
 * Unpack message m_command_2.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int mcan_m_command_2_unpack(
    struct mcan_m_command_2_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from m_command_2.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int mcan_m_command_2_init(struct mcan_m_command_2_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_command_2_m_drive_en_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_command_2_m_drive_en_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_command_2_m_drive_en_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_command_2_m_en_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_command_2_m_en_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_command_2_m_en_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_command_2_m_mode_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_command_2_m_mode_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_command_2_m_mode_is_in_range(uint8_t value);

/**
 * Pack message m_status_2.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int mcan_m_status_2_pack(
    uint8_t *dst_p,
    const struct mcan_m_status_2_t *src_p,
    size_t size);

/**
 * Unpack message m_status_2.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int mcan_m_status_2_unpack(
    struct mcan_m_status_2_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from m_status_2.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int mcan_m_status_2_init(struct mcan_m_status_2_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_status_2_m_drive_en_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_status_2_m_drive_en_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_status_2_m_drive_en_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_status_2_m_en_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_status_2_m_en_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_status_2_m_en_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_status_2_m_mode_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_status_2_m_mode_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_status_2_m_mode_is_in_range(uint8_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
uint8_t mcan_m_status_2_m_error_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_status_2_m_error_decode(uint8_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_status_2_m_error_is_in_range(uint8_t value);

/**
 * Pack message m_ref_2.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int mcan_m_ref_2_pack(
    uint8_t *dst_p,
    const struct mcan_m_ref_2_t *src_p,
    size_t size);

/**
 * Unpack message m_ref_2.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int mcan_m_ref_2_unpack(
    struct mcan_m_ref_2_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from m_ref_2.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int mcan_m_ref_2_init(struct mcan_m_ref_2_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int32_t mcan_m_ref_2_position_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_ref_2_position_decode(int32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_ref_2_position_is_in_range(int32_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int32_t mcan_m_ref_2_velocity_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_ref_2_velocity_decode(int32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_ref_2_velocity_is_in_range(int32_t value);

/**
 * Pack message m_pos_vel_2.
 *
 * @param[out] dst_p Buffer to pack the message into.
 * @param[in] src_p Data to pack.
 * @param[in] size Size of dst_p.
 *
 * @return Size of packed data, or negative error code.
 */
int mcan_m_pos_vel_2_pack(
    uint8_t *dst_p,
    const struct mcan_m_pos_vel_2_t *src_p,
    size_t size);

/**
 * Unpack message m_pos_vel_2.
 *
 * @param[out] dst_p Object to unpack the message into.
 * @param[in] src_p Message to unpack.
 * @param[in] size Size of src_p.
 *
 * @return zero(0) or negative error code.
 */
int mcan_m_pos_vel_2_unpack(
    struct mcan_m_pos_vel_2_t *dst_p,
    const uint8_t *src_p,
    size_t size);

/**
 * Init message fields to default values from m_pos_vel_2.
 *
 * @param[in] msg_p Message to init.
 *
 * @return zero(0) on success or (-1) in case of nullptr argument.
 */
int mcan_m_pos_vel_2_init(struct mcan_m_pos_vel_2_t *msg_p);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int32_t mcan_m_pos_vel_2_position_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_pos_vel_2_position_decode(int32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_pos_vel_2_position_is_in_range(int32_t value);

/**
 * Encode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to encode.
 *
 * @return Encoded signal.
 */
int32_t mcan_m_pos_vel_2_velocity_encode(float value);

/**
 * Decode given signal by applying scaling and offset.
 *
 * @param[in] value Signal to decode.
 *
 * @return Decoded signal.
 */
float mcan_m_pos_vel_2_velocity_decode(int32_t value);

/**
 * Check that given signal is in allowed range.
 *
 * @param[in] value Signal to check.
 *
 * @return true if in range, false otherwise.
 */
bool mcan_m_pos_vel_2_velocity_is_in_range(int32_t value);


#ifdef __cplusplus
}
#endif

#endif
