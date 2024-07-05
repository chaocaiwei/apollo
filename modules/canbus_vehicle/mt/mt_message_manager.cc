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

#include "modules/canbus_vehicle/mt/mt_message_manager.h"

#include "modules/canbus_vehicle/mt/protocol/clear_error_status_1.h"
#include "modules/canbus_vehicle/mt/protocol/current_feedback_12.h"
#include "modules/canbus_vehicle/mt/protocol/motor_fault_status_20.h"
#include "modules/canbus_vehicle/mt/protocol/speed_feedback_10.h"
#include "modules/canbus_vehicle/mt/protocol/wheel_speed_feedback_11.h"

namespace apollo {
namespace canbus {
namespace mt {

MtMessageManager::MtMessageManager() {
  // Control Messages
  AddSendProtocolData<Clearerrorstatus1, true>();

  // Report Messages
  AddRecvProtocolData<Currentfeedback12, true>();
  AddRecvProtocolData<Motorfaultstatus20, true>();
  AddRecvProtocolData<Speedfeedback10, true>();
  AddRecvProtocolData<Wheelspeedfeedback11, true>();
}

MtMessageManager::~MtMessageManager() {}

}  // namespace mt
}  // namespace canbus
}  // namespace apollo
