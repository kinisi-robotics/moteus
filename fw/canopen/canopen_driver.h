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

extern "C" {
#include "301/CO_driver.h"
}

namespace moteus {
class FDCan;

/// Poll the FDCan peripheral associated with the given CANopenNode
/// CAN module: dispatch received classic CAN frames to the stack's
/// receive buffers and flush any software queued transmissions.
/// Must be called regularly from the main loop.  @return the number
/// of frames received.
int CanopenPoll(CO_CANmodule_t* module);
}
