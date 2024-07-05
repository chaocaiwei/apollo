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

#include "modules/canbus_vehicle/mt/protocol/clear_error_status_1.h"

#include "modules/drivers/canbus/common/byte.h"

namespace apollo {
namespace canbus {
namespace mt {

using ::apollo::drivers::canbus::Byte;

const int32_t Clearerrorstatus1::ID = 0x1;

// public
Clearerrorstatus1::Clearerrorstatus1() { Reset(); }

uint32_t Clearerrorstatus1::GetPeriod() const {
  // TODO(All) :  modify every protocol's period manually
  static const uint32_t PERIOD = 20 * 1000;
  return PERIOD;
}

void Clearerrorstatus1::Parse(const std::uint8_t* bytes, int32_t length,
                         Mt* chassis) const {
  chassis->mutable_clear_error_status_1()->set_byte0(byte0(bytes, length));
  chassis->mutable_clear_error_status_1()->set_byte1(byte1(bytes, length));
  chassis->mutable_clear_error_status_1()->set_clearerror(clearerror(bytes, length));
}

void Clearerrorstatus1::UpdateData_Heartbeat(uint8_t* data) {
   // TODO(All) :  you should add the heartbeat manually
}

void Clearerrorstatus1::UpdateData(uint8_t* data) {
  set_p_byte0(data, byte0_);
  set_p_byte1(data, byte1_);
  set_p_clearerror(data, clearerror_);
}

void Clearerrorstatus1::Reset() {
  // TODO(All) :  you should check this manually
  byte0_ = 0;
  byte1_ = 0;
  clearerror_ = 0;
}

Clearerrorstatus1* Clearerrorstatus1::set_byte0(
    int byte0) {
  byte0_ = byte0;
  return this;
 }

// config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'Byte0', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Clearerrorstatus1::set_p_byte0(uint8_t* data,
    int byte0) {
  byte0 = ProtocolData::BoundedValue(0, 255, byte0);
  int x = byte0;

  Byte to_set(data + 0);
  to_set.set_value(x, 0, 8);
}


Clearerrorstatus1* Clearerrorstatus1::set_byte1(
    int byte1) {
  byte1_ = byte1;
  return this;
 }

// config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'Byte1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Clearerrorstatus1::set_p_byte1(uint8_t* data,
    int byte1) {
  byte1 = ProtocolData::BoundedValue(0, 255, byte1);
  int x = byte1;

  Byte to_set(data + 1);
  to_set.set_value(x, 0, 8);
}


Clearerrorstatus1* Clearerrorstatus1::set_clearerror(
    int clearerror) {
  clearerror_ = clearerror;
  return this;
 }

// config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'ClearError', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
void Clearerrorstatus1::set_p_clearerror(uint8_t* data,
    int clearerror) {
  clearerror = ProtocolData::BoundedValue(0, 255, clearerror);
  int x = clearerror;

  Byte to_set(data + 2);
  to_set.set_value(x, 0, 8);
}


int Clearerrorstatus1::byte0(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 0);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int Clearerrorstatus1::byte1(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 1);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}

int Clearerrorstatus1::clearerror(const std::uint8_t* bytes, int32_t length) const {
  Byte t0(bytes + 2);
  int32_t x = t0.get_byte(0, 8);

  int ret = x;
  return ret;
}
}  // namespace mt
}  // namespace canbus
}  // namespace apollo
