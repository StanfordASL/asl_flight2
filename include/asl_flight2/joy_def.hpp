// Copyright 2022 Stanford ASL
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ASL_FLIGHT2__JOY_DEF_HPP_
#define ASL_FLIGHT2__JOY_DEF_HPP_

#define SHIELD

#ifdef SHIELD
#include <asl_flight2/joy_defs/shield_def.hpp>
#elif defined(PS4)
#include <asl_flight2/joy_defs/ps4_def.hpp>
#else
static_assert(false, "You must define a joystick to use");
#endif

#endif  // ASL_FLIGHT2__JOY_DEF_HPP_
