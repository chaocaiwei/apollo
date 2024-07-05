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

#pragma once

#include "modules/canbus_vehicle/mt/proto/mt.pb.h"

#include "modules/drivers/canbus/can_comm/protocol_data.h"

namespace apollo {
namespace canbus {
namespace mt {

class Clearerrorstatus1 : public ::apollo::drivers::canbus::ProtocolData<
                    ::apollo::canbus::Mt> {
 public:
  static const int32_t ID;

  Clearerrorstatus1();

  uint32_t GetPeriod() const override;

  void Parse(const std::uint8_t* bytes, int32_t length,
                     Mt* chassis) const override;

  void UpdateData_Heartbeat(uint8_t* data) override;

  void UpdateData(uint8_t* data) override;

  void Reset() override;

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'Byte0', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Clearerrorstatus1* set_byte0(int byte0);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'Byte1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Clearerrorstatus1* set_byte1(int byte1);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'ClearError', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  Clearerrorstatus1* set_clearerror(int clearerror);

 private:

  // config detail: {'bit': 0, 'is_signed_var': False, 'len': 8, 'name': 'Byte0', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_byte0(uint8_t* data, int byte0);

  // config detail: {'bit': 8, 'is_signed_var': False, 'len': 8, 'name': 'Byte1', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_byte1(uint8_t* data, int byte1);

  // config detail: {'bit': 16, 'is_signed_var': False, 'len': 8, 'name': 'ClearError', 'offset': 0.0, 'order': 'intel', 'physical_range': '[0|255]', 'physical_unit': '', 'precision': 1.0, 'type': 'int'}
  void set_p_clearerror(uint8_t* data, int clearerror);

  int byte0(const std::uint8_t* bytes, const int32_t length) const;

  int byte1(const std::uint8_t* bytes, const int32_t length) const;

  int clearerror(const std::uint8_t* bytes, const int32_t length) const;

 private:
  int byte0_;
  int byte1_;
  int clearerror_;
};

}  // namespace mt
}  // namespace canbus
}  // namespace apollo


