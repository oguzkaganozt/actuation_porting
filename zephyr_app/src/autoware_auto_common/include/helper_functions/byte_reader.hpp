// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_common/include/helper_functions/byte_reader.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2017-2019, the Autoware Foundation
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief This file includes common helper functions

#ifndef HELPER_FUNCTIONS__BYTE_READER_HPP_
#define HELPER_FUNCTIONS__BYTE_READER_HPP_

#include <cstdint>
#include <cstring>
#include <vector>

namespace autoware
{
namespace common
{
namespace helper_functions
{
/// \brief A utility class to read byte vectors in big-endian order
class ByteReader
{
private:
  const std::vector<uint8_t> & byte_vector_;
  std::size_t index_;

public:
  /// \brief Default constructor, byte reader class
  /// \param[in] byte_vector A vector to read bytes from
  explicit ByteReader(const std::vector<uint8_t> & byte_vector)
  : byte_vector_(byte_vector),
    index_(0U)
  {
  }

  // brief Read bytes and store it in the argument passed in big-endian order
  /// \param[inout] value Read and store the bytes from the vector matching the size of the argument
  template<typename T>
  void read(T & value)
  {
    constexpr std::size_t kTypeSize = sizeof(T);
    union {
      T value;
      uint8_t byte_vector[kTypeSize];
    } tmp;

    for (std::size_t i = 0; i < kTypeSize; ++i) {
      tmp.byte_vector[i] = byte_vector_[index_ + kTypeSize - 1 - i];
    }

    value = tmp.value;

    index_ += kTypeSize;
  }

  void skip(std::size_t count)
  {
    index_ += count;
  }
};
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__BYTE_READER_HPP_
