// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/common/autoware_auto_common/include/helper_functions/crtp.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2017-2019, the Autoware Foundation
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
/// \file
/// \brief This file includes common helper functions

#ifndef HELPER_FUNCTIONS__CRTP_HPP_
#define HELPER_FUNCTIONS__CRTP_HPP_

namespace autoware
{
namespace common
{
namespace helper_functions
{
template<typename Derived>
class crtp
{
protected:
  const Derived & impl() const
  {
    // This is the CRTP pattern for static polymorphism: this is related, static_cast is the only
    // way to do this
    //lint -e{9005, 9176, 1939} NOLINT
    return *static_cast<const Derived *>(this);
  }

  Derived & impl()
  {
    // This is the CRTP pattern for static polymorphism: this is related, static_cast is the only
    // way to do this
    //lint -e{9005, 9176, 1939} NOLINT
    return *static_cast<Derived *>(this);
  }
};
}  // namespace helper_functions
}  // namespace common
}  // namespace autoware

#endif  // HELPER_FUNCTIONS__CRTP_HPP_
