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

#include "modules/canbus_vehicle/mt/protocol/speed_feedback_10.h"

#include "glog/logging.h"

#include "modules/drivers/canbus/common/byte.h"
#include "modules/drivers/canbus/common/canbus_consts.h"

namespace apollo {
namespace canbus {
namespace mt {

using ::apollo::drivers::canbus::Byte;

Speedfeedback10::Speedfeedback10() {}
const int32_t Speedfeedback10::ID = 0x10;

void Speedfeedback10::Parse(const std::uint8_t* bytes, int32_t length,
                         Mt* chassis) const {
  chassis->mutable_speed_feedback_10()->set_linearspeedfeedback(linearspeedfeedback(bytes, length));
  chassis->mutable_speed_feedback_10()->set_steeringanglefeedback(steeringanglefeedback(bytes, length));
}

// config detail: {'bit': 0, 'is_signed_var': True, 'len': 16, 'name': 'linearspeedfeedback', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-32768|32767]', 'physical_unit': 'mm/s', 'precision': 1.0, 'type': 'int'}
int Speedfeedback10::linearspeedfeedback(const std::uint8_t* bytes, int32_t length) const {
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

// config detail: {'bit': 16, 'is_signed_var': True, 'len': 16, 'name': 'steeringanglefeedback', 'offset': 0.0, 'order': 'intel', 'physical_range': '[-327.6800000000000068212102633|327.6700000000000068210020965]', 'physical_unit': 'degree', 'precision': 0.01, 'type': 'double'}
double Speedfeedback10::steeringanglefeedback(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 3);
  int32_t x = t0.get_byte(0, 8);

  Byte t1(bytes + 2);
  int32_t t = t1.get_byte(0, 8);
  x <<= 8;
  x |= t;

  x <<= 16;
  x >>= 16;

  double ret = x * 0.010000;
  return ret;
}
}  // namespace mt
}  // namespace canbus
}  // namespace apollo
