// Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef CONTROLLER_COMMON_NODES__ENDIAN_MANIP_HPP_
#define CONTROLLER_COMMON_NODES__ENDIAN_MANIP_HPP_

/// @brief Serialize (Little Endian) a 32-bit value.
/// @param in Data to be serialized
/// @param[out] buf Buffer to write into
/// @return New position in the buffer
inline uint8_t * ser_uint32(uint32_t in, uint8_t * buf)
{
  buf[0] = in;
  buf[1] = in >> 8;
  buf[2] = in >> 16;
  buf[3] = in >> 24;
  return buf + 4;
}

/// @brief Serialize (Little Endian) a float.
/// @param in Data to be serialized
/// @param[out] buf Buffer to write into
/// @return New position in the buffer
inline uint8_t * ser_float(float in, uint8_t * buf)
{
  uint32_t raw;
  static_assert(sizeof(in) == sizeof(raw));
  std::memcpy(&raw, &in, sizeof(in));
  return ser_uint32(raw, buf);
}

/// @brief De-Serialize (Little Endian) uint32_t.
/// @param buf Data to be de-serialized
/// @param[out] De-serialized data
/// @return New position in the buffer
inline uint8_t * deser_uint32(uint8_t *buf, uint32_t *out)
{
  *out = buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24;
  return buf + 4;
}

/// @brief De-Serialize (Little Endian) int32_t.
/// @param buf Data to be de-serialized
/// @param[out] De-serialized data
/// @return New position in the buffer
inline uint8_t * deser_int32(uint8_t *buf, int32_t *out)
{
  uint32_t raw;
  static_assert(sizeof(*out) == sizeof(raw));
  buf = deser_uint32(buf, &raw);
  std::memcpy(out, &raw, sizeof(int32_t));
  return buf;
}

/// @brief De-Serialize (Little Endian) float.
/// @param buf Data to be de-serialized
/// @param[out] De-serialized data
/// @return New position in the buffer
inline uint8_t * deser_float(uint8_t *buf, float *out)
{
  uint32_t raw;
  static_assert(sizeof(*out) == sizeof(raw));
  buf = deser_uint32(buf, &raw);
  std::memcpy(out, &raw, sizeof(float));
  return buf;
}

#endif  // CONTROLLER_COMMON_NODES__ENDIAN_MANIP_HPP_
