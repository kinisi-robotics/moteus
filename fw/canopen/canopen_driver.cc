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

/// @file
///
/// CANopenNode driver implementation for the moteus FDCan
/// peripheral.  Unlike typical CANopenNode ports, everything here
/// runs from the cooperative main loop: received frames are polled
/// from the RX FIFO by CanopenPoll(), and transmissions that do not
/// fit in the hardware TX FIFO are queued in the CANopenNode transmit
/// buffer array and flushed by the same polling function.
///
/// The FDCan peripheral must already be configured for classic CAN
/// 2.0 operation (see CanMode::kCanopen in moteus.cc) before the
/// CANopen stack is started.  The CANptr passed to CO_CANmodule_init
/// is a moteus::FDCan*.

#include "fw/canopen/canopen_driver.h"

#include "mbed.h"

#include "fw/fdcan.h"

namespace {
constexpr uint16_t kCanIdMask = 0x07ffu;
constexpr uint16_t kFlagRtr = 0x0800u;

moteus::FDCan* GetCan(CO_CANmodule_t* module) {
  return reinterpret_cast<moteus::FDCan*>(module->CANptr);
}

moteus::FDCan::SendOptions ClassicSendOptions() {
  moteus::FDCan::SendOptions send_options;
  send_options.fdcan_frame = moteus::FDCan::Override::kDisable;
  send_options.bitrate_switch = moteus::FDCan::Override::kDisable;
  send_options.remote_frame = moteus::FDCan::Override::kDisable;
  send_options.extended_id = moteus::FDCan::Override::kDisable;
  return send_options;
}

bool SendBuffer(CO_CANmodule_t* module, CO_CANtx_t* buffer) {
  return GetCan(module)->TrySend(
      buffer->ident & kCanIdMask,
      std::string_view(reinterpret_cast<const char*>(buffer->data),
                       buffer->DLC),
      ClassicSendOptions());
}

// Recursive interrupt masking for the CO_LOCK_* macros.
volatile uint32_t g_lock_primask = 0;
volatile int32_t g_lock_depth = 0;
}

extern "C" {

void moteus_canopen_irq_lock(void) {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (g_lock_depth++ == 0) {
    g_lock_primask = primask;
  }
}

void moteus_canopen_irq_unlock(void) {
  if (--g_lock_depth == 0) {
    __set_PRIMASK(g_lock_primask);
  }
}

void CO_CANsetConfigurationMode(void* CANptr) {
  // The peripheral is configured by the moteus main() before the
  // CANopen stack is started.  Nothing to do here.
  (void)CANptr;
}

void CO_CANsetNormalMode(CO_CANmodule_t* CANmodule) {
  CANmodule->CANnormal = true;
}

CO_ReturnError_t CO_CANmodule_init(CO_CANmodule_t* CANmodule, void* CANptr,
                                   CO_CANrx_t rxArray[], uint16_t rxSize,
                                   CO_CANtx_t txArray[], uint16_t txSize,
                                   uint16_t CANbitRate) {
  (void)CANbitRate;  // Bitrate is configured by moteus main().

  if (CANmodule == NULL || rxArray == NULL || txArray == NULL) {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  CANmodule->CANptr = CANptr;
  CANmodule->rxArray = rxArray;
  CANmodule->rxSize = rxSize;
  CANmodule->txArray = txArray;
  CANmodule->txSize = txSize;
  CANmodule->CANerrorStatus = 0;
  CANmodule->CANnormal = false;
  // All acceptance filtering is done in software here; the hardware
  // filters are configured to accept all standard frames.
  CANmodule->useCANrxFilters = false;
  CANmodule->bufferInhibitFlag = false;
  CANmodule->firstCANtxMessage = true;
  CANmodule->CANtxCount = 0;
  CANmodule->errOld = 0;

  for (uint16_t i = 0; i < rxSize; i++) {
    rxArray[i].ident = 0;
    rxArray[i].mask = 0xffffu;
    rxArray[i].object = NULL;
    rxArray[i].CANrx_callback = NULL;
  }
  for (uint16_t i = 0; i < txSize; i++) {
    txArray[i].bufferFull = false;
  }

  return CO_ERROR_NO;
}

void CO_CANmodule_disable(CO_CANmodule_t* CANmodule) {
  if (CANmodule != NULL) {
    CANmodule->CANnormal = false;
  }
}

CO_ReturnError_t CO_CANrxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index,
                                    uint16_t ident, uint16_t mask, bool_t rtr,
                                    void* object,
                                    void (*CANrx_callback)(void* object,
                                                           void* message)) {
  if (CANmodule == NULL || object == NULL || CANrx_callback == NULL ||
      index >= CANmodule->rxSize) {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  CO_CANrx_t* const buffer = &CANmodule->rxArray[index];
  buffer->object = object;
  buffer->CANrx_callback = CANrx_callback;
  buffer->ident = (ident & kCanIdMask) | (rtr ? kFlagRtr : 0x00);
  buffer->mask = (mask & kCanIdMask) | kFlagRtr;

  return CO_ERROR_NO;
}

CO_CANtx_t* CO_CANtxBufferInit(CO_CANmodule_t* CANmodule, uint16_t index,
                               uint16_t ident, bool_t rtr, uint8_t noOfBytes,
                               bool_t syncFlag) {
  if (CANmodule == NULL || index >= CANmodule->txSize || noOfBytes > 8) {
    return NULL;
  }

  CO_CANtx_t* const buffer = &CANmodule->txArray[index];
  buffer->ident = (ident & kCanIdMask) | (rtr ? kFlagRtr : 0x00);
  buffer->DLC = noOfBytes;
  buffer->bufferFull = false;
  buffer->syncFlag = syncFlag;
  return buffer;
}

CO_ReturnError_t CO_CANsend(CO_CANmodule_t* CANmodule, CO_CANtx_t* buffer) {
  CO_ReturnError_t err = CO_ERROR_NO;

  // We do not transmit remote frames.
  if (buffer->ident & kFlagRtr) {
    return CO_ERROR_ILLEGAL_ARGUMENT;
  }

  if (buffer->bufferFull) {
    if (!CANmodule->firstCANtxMessage) {
      // Don't set an error if the bootup message is still on buffers.
      CANmodule->CANerrorStatus |= CO_CAN_ERRTX_OVERFLOW;
    }
    err = CO_ERROR_TX_OVERFLOW;
  }

  CO_LOCK_CAN_SEND(CANmodule);
  // To preserve ordering, transmit directly only if nothing is
  // queued in software.
  if (CANmodule->CANtxCount == 0 && SendBuffer(CANmodule, buffer)) {
    CANmodule->bufferInhibitFlag = buffer->syncFlag;
    CANmodule->firstCANtxMessage = false;
  } else if (!buffer->bufferFull) {
    buffer->bufferFull = true;
    CANmodule->CANtxCount++;
  }
  CO_UNLOCK_CAN_SEND(CANmodule);

  return err;
}

void CO_CANclearPendingSyncPDOs(CO_CANmodule_t* CANmodule) {
  uint32_t tpdo_deleted = 0;

  CO_LOCK_CAN_SEND(CANmodule);
  if (CANmodule->bufferInhibitFlag) {
    // A synchronous TPDO may still be waiting in the hardware queue.
    // We cannot selectively abort it, so just note the fact.
    CANmodule->bufferInhibitFlag = false;
    tpdo_deleted = 1;
  }
  if (CANmodule->CANtxCount > 0) {
    for (uint16_t i = 0; i < CANmodule->txSize; i++) {
      if (CANmodule->txArray[i].bufferFull &&
          CANmodule->txArray[i].syncFlag) {
        CANmodule->txArray[i].bufferFull = false;
        CANmodule->CANtxCount--;
        tpdo_deleted = 2;
      }
    }
  }
  CO_UNLOCK_CAN_SEND(CANmodule);

  if (tpdo_deleted != 0) {
    CANmodule->CANerrorStatus |= CO_CAN_ERRTX_PDO_LATE;
  }
}

void CO_CANmodule_process(CO_CANmodule_t* CANmodule) {
  auto* const can = GetCan(CANmodule);

  const auto status = can->status();

  uint16_t error_status = CANmodule->CANerrorStatus;
  error_status &= ~(CO_CAN_ERRTX_BUS_OFF |
                    CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING |
                    CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE);

  if (status.BusOff) {
    error_status |= CO_CAN_ERRTX_BUS_OFF;
    // Request recovery: the peripheral counts 129 bus idle sequences
    // before resuming operation.
    can->RecoverBusOff();
  }
  if (status.Warning) {
    error_status |= CO_CAN_ERRRX_WARNING | CO_CAN_ERRTX_WARNING;
  }
  if (status.ErrorPassive) {
    error_status |= CO_CAN_ERRRX_PASSIVE | CO_CAN_ERRTX_PASSIVE;
  }

  CANmodule->CANerrorStatus = error_status;
}

}  // extern "C"

namespace moteus {

int CanopenPoll(CO_CANmodule_t* module) {
  auto* const can = GetCan(module);

  // Dispatch received frames.
  FDCAN_RxHeaderTypeDef header = {};
  CO_CANrxMsg_t msg = {};
  char data[64] = {};
  int received = 0;

  while (can->Poll(&header, mjlib::base::string_span(data, sizeof(data)))) {
    received++;
    if (header.IdType != FDCAN_STANDARD_ID) { continue; }
    if (header.RxFrameType != FDCAN_DATA_FRAME) { continue; }
    if (header.FDFormat != FDCAN_CLASSIC_CAN) { continue; }

    const int size = FDCan::ParseDlc(header.DataLength);
    if (size > 8) { continue; }

    msg.ident = header.Identifier & kCanIdMask;
    msg.dlc = static_cast<uint8_t>(size);
    std::memcpy(msg.data, data, size);

    for (uint16_t i = 0; i < module->rxSize; i++) {
      CO_CANrx_t* const buffer = &module->rxArray[i];
      if (((msg.ident ^ buffer->ident) & buffer->mask) == 0) {
        if (buffer->CANrx_callback != NULL) {
          buffer->CANrx_callback(buffer->object, &msg);
        }
        break;
      }
    }
  }

  // Flush any software queued transmissions in buffer array order.
  if (module->CANtxCount > 0) {
    CO_LOCK_CAN_SEND(module);
    for (uint16_t i = 0;
         i < module->txSize && module->CANtxCount > 0 &&
             can->TxQueueFree() > 0;
         i++) {
      CO_CANtx_t* const buffer = &module->txArray[i];
      if (buffer->bufferFull) {
        if (!SendBuffer(module, buffer)) { break; }
        buffer->bufferFull = false;
        module->CANtxCount--;
        module->bufferInhibitFlag = buffer->syncFlag;
        module->firstCANtxMessage = false;
      }
    }
    CO_UNLOCK_CAN_SEND(module);
  }

  return received;
}

}  // namespace moteus
