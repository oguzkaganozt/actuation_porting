// Based on: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/e3e26be1ab4b822996df60f261d031d7924f20ac/src/control/motion_common/scripts/autogeneration_code_snippets/linear_tire.snippet.hpp
// In open-source project: Autoware.Auto
// Original file: Copyright (c) 2019, Christopher Ho
// Modifications: Copyright (c) 2023, Arm Limited.
// SPDX-License-Identifier: Apache-2.0

#ifndef COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__LINEAR_TIRE_SNIPPET_HPP_  // NOLINT
#define COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__LINEAR_TIRE_SNIPPET_HPP_  // NOLINT
// For help on getting random weird models to work, see:
// https://sourceforge.net/p/acado/discussion/general
// and
// https://github.com/acado/acado/issues

// Note: Order of variable declaration specifies order in C-arrays
//
// Variables
//

DifferentialState x, y, yaw;  // pose
DifferentialState u, v, omega;  // velocities
DifferentialState delta;  // wheel angle
DifferentialState ax;  // acceleration
Control jx, delta_dot;

// Vehicle parameters
OnlineData L_f, L_r;  // front, rear wheelbase length
OnlineData C_f, C_r;  // front, rear cornering stiffness
OnlineData m, I;  // mass, moment of inertia

//
// Differential algebraic equation
//

// Using the linear model due to LaValle:
// http://planning.cs.uiuc.edu/node695.html
// Intermediate variables
IntermediateState F_f = C_f * (((v + (L_f * omega)) / u) + delta);
IntermediateState F_r = C_r * ((v - (L_r * omega)) / u);

DifferentialEquation f;

// Easy stuff
f << dot(x) == ((u * cos(yaw)) - (v * sin(yaw)));
f << dot(y) == ((u * sin(yaw)) + (v * cos(yaw)));
f << dot(yaw) == omega;
f << dot(u) == ax;
f << dot(v) == ((-u * omega) + ((F_f + F_r) / m));
f << dot(omega) == (((L_f * F_f) - (L_r * F_r)) / I);
f << dot(delta) == delta_dot;
f << dot(ax) == jx;
#endif  // COMMON__MOTION_COMMON__SCRIPTS__AUTOGENERATION_CODE_SNIPPETS__LINEAR_TIRE_SNIPPET_HPP_  // NOLINT
