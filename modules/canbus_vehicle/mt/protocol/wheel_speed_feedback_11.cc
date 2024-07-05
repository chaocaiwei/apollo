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

#include "modules/canbus_vehicle/mt/protocol/wheel_speed_feedback_11.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mt {

using ::apollo::drivers::canbus::Byte;

Wheelspeedfeedback11::Wheelspeedfeedback11() {}
const int32_t Wheelspeedfeedback11::ID = 0x11;

void Wheelspeedfeedback11::Parse(const std::uint8_t* bytes, int32_t length,
                         Mt* chassis) const {
  chassis->mutable_wheel_speed_feedback_11()->set_wheelspeed(wheelspeed(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'wheelspeed', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-32768|32767]', 'physical_unit': 'rpm', 'precision': 1.0, 'type': 'int'}
int Wheelspeedfeedback11::wheelspeed(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 0);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  int ret = x;
  return ret;
}
}  // namespace mt
}  // namespace canbus
}  // namespace apollo
