// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/motion_common/scripts/autogeneration_code_snippets/kinematic_bicycle.snippet.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, Christopher Ho
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__KINEMATIC_BICYCLE_SNIPPET_HPP_  // NOLINT
#define COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__KINEMATIC_BICYCLE_SNIPPET_HPP_  // NOLINT

// For help on getting random weird models to work, see:
// https://sourceforge.net/p/acado/discussion/general
// and
// https://github.com/acado/acado/issues

// Note: Order of variable declaration specifies order in C-arrays
//
// Variables
//

DifferentialState x, y, yaw;  // pose
DifferentialState u;
Control ax;  // acceleration
Control delta;  // wheel angle

// Vehicle parameters
OnlineData L_f, L_r;  // front, rear wheelbase length
//
// Differential algebraic equation
//

// Kinematic bicycle model:
// https://borrelli.me.berkeley.edu/pdfpub/IV_KinematicMPC_jason.pdf
// Intermediate variables
IntermediateState beta = atan((L_r * tan(delta)) / (L_f + L_r));

DifferentialEquation f;

f << dot(x) == u * cos(yaw + beta);
f << dot(y) == u * sin(yaw + beta);
f << dot(yaw) == (u * sin(beta)) / L_r;
f << dot(u) == ax;
#endif  // COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__KINEMATIC_BICYCLE_SNIPPET_HPP_  // NOLINT
