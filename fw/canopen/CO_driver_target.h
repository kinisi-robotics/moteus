/* Device and application specific definitions for CANopenNode on the
 * moteus controller.
 *
 * Adapted from the CanOpenNodeSTM32 port:
 *   Copyright 2004 - 2020 Janez Paternoster
 *   Hamed Jafarzadeh 2022, Tilen Marjerle 2021
 *
 * This file is part of CANopenNode, an opensource CANopen Stack.
 * Project home page is <https://github.com/CANopenNode/CANopenNode>.
 * For more information on CANopen see <http://www.can-cia.org/>.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef CO_DRIVER_TARGET_H
#define CO_DRIVER_TARGET_H

/* This file contains device and application specific definitions.
 * It is included from CO_driver.h, which contains documentation
 * for common definitions below.
 *
 * On moteus, the CANopen stack runs entirely from the cooperative
 * main loop: CAN frames are polled from the FDCan peripheral's RX
 * FIFO and all protocol processing happens in main-loop context.
 * Nothing CANopen related runs from interrupt context. */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Allocate all CANopenNode objects statically instead of from the
 * heap. */
#define CO_USE_GLOBALS

/* Basic definitions. If big endian, CO_SWAP_xx macros must swap bytes. */
#define CO_LITTLE_ENDIAN
#define CO_SWAP_16(x) x
#define CO_SWAP_32(x) x
#define CO_SWAP_64(x) x

/* NULL is defined in stddef.h */
/* true and false are defined in stdbool.h */
/* int8_t to uint64_t are defined in stdint.h */
typedef uint_fast8_t bool_t;
typedef float float32_t;
typedef double float64_t;

/* Received CAN message structure (classic CAN only). */
typedef struct {
    uint32_t ident;  /* Standard identifier */
    uint8_t dlc;     /* Data length */
    uint8_t data[8]; /* Received data */
} CO_CANrxMsg_t;

/* Access to received CAN message */
#define CO_CANrxMsg_readIdent(msg) ((uint16_t)(((CO_CANrxMsg_t*)(msg))->ident))
#define CO_CANrxMsg_readDLC(msg)   ((uint8_t)(((CO_CANrxMsg_t*)(msg))->dlc))
#define CO_CANrxMsg_readData(msg)  ((uint8_t*)(((CO_CANrxMsg_t*)(msg))->data))

/* Received message object */
typedef struct {
    uint16_t ident;
    uint16_t mask;
    void* object;
    void (*CANrx_callback)(void* object, void* message);
} CO_CANrx_t;

/* Transmit message object */
typedef struct {
    uint32_t ident;
    uint8_t DLC;
    uint8_t data[8];
    volatile bool_t bufferFull;
    volatile bool_t syncFlag;
} CO_CANtx_t;

/* CAN module object */
typedef struct {
    void* CANptr; /* moteus::CanopenDriver* */
    CO_CANrx_t* rxArray;
    uint16_t rxSize;
    CO_CANtx_t* txArray;
    uint16_t txSize;
    uint16_t CANerrorStatus;
    volatile bool_t CANnormal;
    volatile bool_t useCANrxFilters;
    volatile bool_t bufferInhibitFlag;
    volatile bool_t firstCANtxMessage;
    volatile uint16_t CANtxCount;
    uint32_t errOld;
} CO_CANmodule_t;

/* Data storage object for one entry */
typedef struct {
    void* addr;
    size_t len;
    uint8_t subIndexOD;
    uint8_t attr;
} CO_storage_entry_t;

/* Critical sections.  All CANopen processing happens from the main
 * loop, but we still mask interrupts for safety against future ISR
 * usage.  The lock is recursive. */
void moteus_canopen_irq_lock(void);
void moteus_canopen_irq_unlock(void);

#define CO_LOCK_CAN_SEND(CAN_MODULE) moteus_canopen_irq_lock()
#define CO_UNLOCK_CAN_SEND(CAN_MODULE) moteus_canopen_irq_unlock()

#define CO_LOCK_EMCY(CAN_MODULE) moteus_canopen_irq_lock()
#define CO_UNLOCK_EMCY(CAN_MODULE) moteus_canopen_irq_unlock()

#define CO_LOCK_OD(CAN_MODULE) moteus_canopen_irq_lock()
#define CO_UNLOCK_OD(CAN_MODULE) moteus_canopen_irq_unlock()

/* Synchronization between CAN receive and message processing. */
#define CO_MemoryBarrier()
#define CO_FLAG_READ(rxNew) ((rxNew) != NULL)
#define CO_FLAG_SET(rxNew)                                              \
    do {                                                                \
        CO_MemoryBarrier();                                             \
        rxNew = (void*)1L;                                              \
    } while (0)
#define CO_FLAG_CLEAR(rxNew)                                            \
    do {                                                                \
        CO_MemoryBarrier();                                             \
        rxNew = NULL;                                                   \
    } while (0)

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CO_DRIVER_TARGET_H */
