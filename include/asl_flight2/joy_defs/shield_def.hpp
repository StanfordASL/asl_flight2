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

#ifndef ASL_FLIGHT2__SHIELD_DEF_HPP_
#define ASL_FLIGHT2__SHIELD_DEF_HPP_

enum shield_axes_e
{
  LEFT_LR = 0,
  LEFT_UD,
  RIGHT_LR,
  L2,
  R2,
  RIGHT_UD
};

enum shield_btn_e
{
  X = 0,    // A
  O,        // B
  SQUARE,   // X
  TRIANGLE, // Y
  L1,
  R1
};

#endif  // ASL_FLIGHT2__SHIELD_DEF_HPP_
