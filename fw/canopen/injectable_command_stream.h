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

#include <cstring>
#include <string_view>

#include "mjlib/base/string_span.h"
#include "mjlib/micro/async_stream.h"

namespace moteus {

/// Wraps an AsyncStream so that synthetic input can be injected as
/// if it had been received from the underlying stream.  This is used
/// to trigger console commands (like "conf write") from the CANopen
/// SDO persist entry.
///
/// Reads from the underlying stream are continuously pumped into a
/// small internal buffer once Start() has been invoked.  Data is
/// only delivered to a pending reader from Poll(), never
/// synchronously from AsyncReadSome.
class InjectableCommandStream : public mjlib::micro::AsyncStream {
 public:
  explicit InjectableCommandStream(mjlib::micro::AsyncStream* base)
      : base_(base) {}

  /// Begin pumping the underlying stream.  Must only be called once
  /// the underlying stream is ready to be read from.
  void Start() {
    StartBaseRead();
  }

  /// Queue synthetic input.  @return false if there was insufficient
  /// buffer space.
  bool Inject(std::string_view data) {
    if (data.size() > Available()) { return false; }
    for (char c : data) { Push(c); }
    return true;
  }

  void Poll() {
    if (!read_callback_) { return; }
    if (Size() == 0) { return; }

    size_t count = 0;
    while (count < static_cast<size_t>(read_buffer_.size()) && Size() != 0) {
      read_buffer_.data()[count++] = Pop();
    }

    auto callback = read_callback_;
    read_callback_ = {};
    read_buffer_ = {};
    callback(mjlib::micro::error_code(), count);
  }

  void AsyncReadSome(const mjlib::base::string_span& buffer,
                     const mjlib::micro::SizeCallback& callback) override {
    // Only a single outstanding read is supported.
    read_buffer_ = buffer;
    read_callback_ = callback;
  }

  void AsyncWriteSome(const std::string_view& buffer,
                      const mjlib::micro::SizeCallback& callback) override {
    base_->AsyncWriteSome(buffer, callback);
  }

 private:
  void StartBaseRead() {
    base_->AsyncReadSome(
        mjlib::base::string_span(base_buffer_, sizeof(base_buffer_)),
        [this](mjlib::micro::error_code ec, size_t size) {
          if (!ec) {
            for (size_t i = 0; i < size && Available() != 0; i++) {
              Push(base_buffer_[i]);
            }
          }
          StartBaseRead();
        });
  }

  size_t Size() const {
    return static_cast<size_t>(head_ - tail_);
  }

  size_t Available() const {
    return sizeof(ring_) - Size();
  }

  void Push(char c) {
    ring_[head_ % sizeof(ring_)] = c;
    head_++;
  }

  char Pop() {
    const char c = ring_[tail_ % sizeof(ring_)];
    tail_++;
    return c;
  }

  mjlib::micro::AsyncStream* const base_;

  char base_buffer_[16] = {};

  char ring_[128] = {};
  uint32_t head_ = 0;
  uint32_t tail_ = 0;

  mjlib::base::string_span read_buffer_;
  mjlib::micro::SizeCallback read_callback_;
};

}
