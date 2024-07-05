/******************************************************************************
 * Copyright 2023 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "modules/canbus_vehicle/mt/protocol/current_feedback_12.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mt {

using ::apollo::drivers::canbus::Byte;

Currentfeedback12::Currentfeedback12() {}
const int32_t Currentfeedback12::ID = 0x12;

void Currentfeedback12::Parse(const std::uint8_t* bytes, int32_t length,
                         Mt* chassis) const {
  chassis->mutable_current_feedback_12()->set_motorcurrent(motorcurrent(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'motorcurrent', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-3276.800000000000181898940355|3276.700000000000181893389239]', 'physical_unit': 'A', 'precision': 0.1, 'type': 'double'}
double Currentfeedback12::motorcurrent(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.100000;
  return ret;
}
}  // namespace mt
}  // namespace canbus
}  // namespace apollo
