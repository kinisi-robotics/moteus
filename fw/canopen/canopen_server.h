// Copyright 2026 Kinisi Robotics
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

#pragma once

#include <cstdint>

#include "mjlib/base/inplace_function.h"

#include "fw/bldc_servo.h"
#include "fw/fdcan.h"
#include "fw/millisecond_timer.h"

extern "C" {
#include "CANopen.h"
#include "OD.h"
}

namespace moteus {

/// A CANopen node implementing the Kinisi actuator profile
/// (0x6000-0x601E) on top of CANopenNode.  See KINISI_CANOPEN.md.
///
/// Command semantics per SYNC when OPERATIONAL and controlWord bit 0
/// is set:
///   0x6001 commandPosition [deg]     -> position mode target
///   0x6002 commandVelocity [RPM]     -> velocity feedforward
///   0x6003 commandCurrent  [Nm]      -> feedforward torque (*)
///   0x6010 maxVelocity     [RPM]     -> velocity limit when non-zero
///   0x6014 currentLimit    [Nm]      -> maximum torque when > 0 (*)
///
/// (*) For the moteus device type these carry torque in Nm rather
/// than milliamps; the Kinisi wire format (float32) is unchanged.
///
/// Feedback published on each SYNC:
///   0x6004 position [deg], 0x6005 velocity [RPM],
///   0x6006 current [Nm of torque], 0x6007 statusWord
///   (bit0 = torque enabled, bit1 = fault), 0x6009 errorCode
///   (moteus fault code), 0x6015 temperature [C].
class CanopenServer {
 public:
  struct Options {
    // The CANopen node id to use, 1-127.
    uint8_t node_id = 127;

    // Invoked when the node id is changed via SDO write to 0x6011.
    // The new id takes effect after the configuration is persisted
    // and the device rebooted.
    mjlib::base::inplace_function<void (uint8_t)> node_id_updated;

    // Invoked when a configuration persist is requested via SDO
    // write to 0x601A.
    mjlib::base::inplace_function<void ()> persist_requested;
  };

  CanopenServer(FDCan* can, BldcServo* bldc, MillisecondTimer* timer);

  /// Start the CANopen stack.  The FDCan peripheral must already be
  /// configured for classic CAN 2.0.  @return false on
  /// initialization failure.
  bool Start(const Options& options);

  /// Must be called from the main loop as often as possible.
  void Poll();

  bool started() const { return started_; }

 private:
  bool ResetCommunication();
  void ProcessControl(bool sync_was);
  void UpdateFeedback();

  static ODR_t WriteNodeId(OD_stream_t* stream, const void* buf,
                           OD_size_t count, OD_size_t* count_written);
  static ODR_t WritePersistTrigger(OD_stream_t* stream, const void* buf,
                                   OD_size_t count, OD_size_t* count_written);

  FDCan* const can_;
  BldcServo* const bldc_;
  MillisecondTimer* const timer_;

  Options options_;
  CO_t* co_ = nullptr;
  bool started_ = false;

  uint8_t desired_node_id_ = 127;
  uint16_t baudrate_ = 1000;

  uint32_t last_process_us_ = 0;
  bool command_active_ = false;
  moteus::errc last_fault_ = moteus::errc::kSuccess;

  OD_extension_t node_id_extension_ = {};
  OD_extension_t persist_extension_ = {};
};

}
