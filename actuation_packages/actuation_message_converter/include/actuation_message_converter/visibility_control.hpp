// Copyright (c) 2021-2023, Arm Limited., The Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

// SPDX-License-Identifier: Apache-2.0

#ifndef ACTUATION_MESSAGE_CONVERTER__VISIBILITY_CONTROL_HPP_
#define ACTUATION_MESSAGE_CONVERTER__VISIBILITY_CONTROL_HPP_

////////////////////////////////////////////////////////////////////////////////
#if defined(__WIN32)
  #if defined(ACTUATION_MESSAGE_CONVERTER_BUILDING_DLL) || \
  defined(ACTUATION_MESSAGE_CONVERTER_EXPORTS)
    #define ACTUATION_MESSAGE_CONVERTER_PUBLIC __declspec(dllexport)
    #define ACTUATION_MESSAGE_CONVERTER_LOCAL
  #else
    #define ACTUATION_MESSAGE_CONVERTER_PUBLIC __declspec(dllimport)
    #define ACTUATION_MESSAGE_CONVERTER_LOCAL
  #endif
#elif defined(__linux__)
  #define ACTUATION_MESSAGE_CONVERTER_PUBLIC __attribute__((visibility("default")))
  #define ACTUATION_MESSAGE_CONVERTER_LOCAL __attribute__((visibility("hidden")))
#elif defined(__APPLE__)
  #define ACTUATION_MESSAGE_CONVERTER_PUBLIC __attribute__((visibility("default")))
  #define ACTUATION_MESSAGE_CONVERTER_LOCAL __attribute__((visibility("hidden")))
#else
  #error "Unsupported Build Configuration"
#endif

#endif  // ACTUATION_MESSAGE_CONVERTER__VISIBILITY_CONTROL_HPP_
