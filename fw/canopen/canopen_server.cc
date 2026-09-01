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

#include "fw/canopen/canopen_server.h"

#include <cmath>

#include "mbed.h"

#include "fw/canopen/canopen_driver.h"

namespace moteus {

namespace {
// Values match the other Kinisi CANopen actuator nodes.
constexpr auto kNmtControl = static_cast<CO_NMT_control_t>(
    CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION);
constexpr uint16_t kFirstHbTimeMs = 500;
constexpr uint16_t kSdoServerTimeoutMs = 1000;
constexpr uint16_t kSdoClientTimeoutMs = 500;

// If the master stops sending SYNC/commands, the servo stops after
// this long.
constexpr float kCommandTimeoutS = 0.25f;

// We only run the protocol state machines this often (or immediately
// upon CAN activity).
constexpr uint32_t kProcessIntervalUs = 200;

// controlWord bits (Kinisi actuator profile).
constexpr uint8_t kControlWordTorqueEnable = 0x01;
constexpr uint8_t kControlWordReboot = 0x80;

// statusWord bits.
constexpr uint8_t kStatusWordTorqueOn = 0x01;
constexpr uint8_t kStatusWordError = 0x02;
}  // namespace

CanopenServer::CanopenServer(FDCan* can, BldcServo* bldc,
                             MillisecondTimer* timer)
    : can_(can), bldc_(bldc), timer_(timer) {}

bool CanopenServer::Start(const Options& options) {
  options_ = options;
  desired_node_id_ = options.node_id;

  uint32_t heap_used = 0;
  co_ = CO_new(nullptr, &heap_used);
  if (co_ == nullptr) { return false; }

  if (!ResetCommunication()) { return false; }

  started_ = true;
  last_process_us_ = timer_->read_us();
  return true;
}

bool CanopenServer::ResetCommunication() {
  co_->CANmodule->CANnormal = false;

  CO_CANsetConfigurationMode(can_);
  CO_CANmodule_disable(co_->CANmodule);

  if (CO_CANinit(co_, can_, 0) != CO_ERROR_NO) { return false; }

#if (CO_CONFIG_LSS) & CO_CONFIG_LSS_SLAVE
  CO_LSS_address_t lss_address = {};
  lss_address.identity.vendorID = OD_PERSIST_COMM.x1018_identity.vendor_ID;
  lss_address.identity.productCode =
      OD_PERSIST_COMM.x1018_identity.productCode;
  lss_address.identity.revisionNumber =
      OD_PERSIST_COMM.x1018_identity.revisionNumber;
  lss_address.identity.serialNumber =
      OD_PERSIST_COMM.x1018_identity.serialNumber;
  if (CO_LSSinit(co_, &lss_address,
                 &desired_node_id_, &baudrate_) != CO_ERROR_NO) {
    return false;
  }
#endif

  const uint8_t active_node_id = desired_node_id_;
  uint32_t err_info = 0;

  {
    const auto err = CO_CANopenInit(
        co_,
        nullptr,      // alternate NMT
        nullptr,      // alternate em
        OD,
        nullptr,      // optional status bits
        kNmtControl,
        kFirstHbTimeMs,
        kSdoServerTimeoutMs,
        kSdoClientTimeoutMs,
        false,        // no SDO client block transfer
        active_node_id,
        &err_info);
    if (err != CO_ERROR_NO && err != CO_ERROR_NODE_ID_UNCONFIGURED_LSS) {
      return false;
    }
  }

  if (active_node_id != CO_LSS_NODE_ID_ASSIGNMENT) {
    if (CO_CANopenInitPDO(co_, co_->em, OD, active_node_id,
                          &err_info) != CO_ERROR_NO) {
      return false;
    }
  }

  // Reflect our persisted node id in the object dictionary and hook
  // the profile's node id / persist entries.
  OD_PERSIST_COMM.x6011_nodeID = desired_node_id_;
  OD_RAM.x6012_controlMode = 1;  // position

  node_id_extension_.object = this;
  node_id_extension_.read = OD_readOriginal;
  node_id_extension_.write = &CanopenServer::WriteNodeId;
  OD_extension_init(OD_ENTRY_H6011, &node_id_extension_);

  persist_extension_.object = this;
  persist_extension_.read = OD_readOriginal;
  persist_extension_.write = &CanopenServer::WritePersistTrigger;
  OD_extension_init(OD_ENTRY_H601A, &persist_extension_);

  CO_CANsetNormalMode(co_->CANmodule);

  return true;
}

void CanopenServer::Poll() {
  if (!started_) { return; }

  const int received = CanopenPoll(co_->CANmodule);

  const uint32_t now_us = timer_->read_us();
  const uint32_t dt_us =
      MillisecondTimer::subtract_us(now_us, last_process_us_);
  if (received == 0 && dt_us < kProcessIntervalUs) { return; }
  last_process_us_ = now_us;

  CO_CANmodule_process(co_->CANmodule);

  const auto reset = CO_process(co_, false, dt_us, nullptr);
  if (reset == CO_RESET_APP || reset == CO_RESET_QUIT) {
    NVIC_SystemReset();
  } else if (reset == CO_RESET_COMM) {
    ResetCommunication();
    return;
  }

  if (co_->nodeIdUnconfigured) { return; }

  const bool sync_was = CO_process_SYNC(co_, dt_us, nullptr);
  CO_process_RPDO(co_, sync_was, dt_us, nullptr);

  if (sync_was) {
    ProcessControl(true);
    UpdateFeedback();
  }

  CO_process_TPDO(co_, sync_was, dt_us, nullptr);
}

void CanopenServer::ProcessControl(bool sync_was) {
  (void)sync_was;

  const uint8_t control_word = OD_RAM.x6007_controlWord;

  if (control_word & kControlWordReboot) {
    NVIC_SystemReset();
  }

  const bool operational =
      CO_NMT_getInternalState(co_->NMT) == CO_NMT_OPERATIONAL;
  const bool enabled =
      operational && (control_word & kControlWordTorqueEnable) != 0;

  if (!enabled) {
    if (command_active_) {
      command_active_ = false;
      BldcServo::CommandData command;
      command.mode = BldcServo::Mode::kStopped;
      bldc_->Command(command);
    }
    return;
  }

  command_active_ = true;

  BldcServo::CommandData command;
  command.mode = BldcServo::Mode::kPosition;
  command.position = OD_PERSIST_COMM.x6001_commandPosition / 360.0f;
  command.velocity = OD_RAM.x6002_commandVelocity / 60.0f;
  command.feedforward_Nm = OD_RAM.x6003_commandCurrent;
  command.timeout_s = kCommandTimeoutS;

  if (OD_RAM.x6014_currentLimit > 0.0f) {
    command.max_torque_Nm = OD_RAM.x6014_currentLimit;
  }
  if (OD_RAM.x6010_maxVelocity > 0) {
    command.velocity_limit =
        static_cast<float>(OD_RAM.x6010_maxVelocity) / 60.0f;
  }

  bldc_->Command(command);
}

void CanopenServer::UpdateFeedback() {
  const auto& status = bldc_->status();

  OD_RAM.x6004_position = status.position * 360.0f;
  OD_RAM.x6005_velocity = status.velocity * 60.0f;
  OD_RAM.x6006_current = status.torque_Nm;
  OD_RAM.x6015_temperature =
      std::isnan(status.filt_fet_temp_C) ?
      status.fet_temp_C : status.filt_fet_temp_C;

  const bool fault = status.mode == BldcServo::Mode::kFault;
  const bool torque_on = status.mode == BldcServo::Mode::kPosition;

  OD_RAM.x6008_statusWord =
      (torque_on ? kStatusWordTorqueOn : 0) |
      (fault ? kStatusWordError : 0);
  OD_RAM.x6009_errorCode = static_cast<int32_t>(status.fault);

  // Emit an emergency message on any new fault.
  if (status.fault != last_fault_) {
    if (status.fault != moteus::errc::kSuccess) {
      CO_errorReport(co_->em, CO_EM_GENERIC_ERROR, CO_EMC_DEVICE_HARDWARE,
                     static_cast<uint32_t>(status.fault));
    } else {
      CO_errorReset(co_->em, CO_EM_GENERIC_ERROR, 0);
    }
    last_fault_ = status.fault;
  }
}

ODR_t CanopenServer::WriteNodeId(OD_stream_t* stream, const void* buf,
                                 OD_size_t count, OD_size_t* count_written) {
  auto* const self = reinterpret_cast<CanopenServer*>(stream->object);

  if (buf == nullptr || count != 1) { return ODR_TYPE_MISMATCH; }

  const uint8_t new_id = *reinterpret_cast<const uint8_t*>(buf);
  if (new_id < 1 || new_id > 127) { return ODR_INVALID_VALUE; }

  if (self->options_.node_id_updated) {
    self->options_.node_id_updated(new_id);
  }

  // The new id takes effect after a persist and reboot.
  return OD_writeOriginal(stream, buf, count, count_written);
}

ODR_t CanopenServer::WritePersistTrigger(OD_stream_t* stream, const void* buf,
                                         OD_size_t count,
                                         OD_size_t* count_written) {
  auto* const self = reinterpret_cast<CanopenServer*>(stream->object);

  if (buf == nullptr || count != 1) { return ODR_TYPE_MISMATCH; }

  if (*reinterpret_cast<const uint8_t*>(buf) != 0) {
    // Persisting writes flash, which stalls the CPU: refuse while the
    // servo is actively controlling.
    if (self->command_active_) { return ODR_DATA_DEV_STATE; }

    if (self->options_.persist_requested) {
      self->options_.persist_requested();
    }
  }

  return OD_writeOriginal(stream, buf, count, count_written);
}

}  // namespace moteus
