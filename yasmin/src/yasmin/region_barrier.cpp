// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include <stdexcept>

#include "yasmin/region_barrier.hpp"

namespace yasmin {

RegionBarrier::RegionBarrier(int party_count)
    : party_count_(party_count), initial_count_(party_count) {
  if (party_count < 1) {
    throw std::invalid_argument("party_count must be >= 1");
  }
}

void RegionBarrier::arrive_and_wait() {
  std::unique_lock<std::mutex> lock(this->mtx_);
  this->arrived_++;
  if (this->arrived_ < this->party_count_) {
    int my_gen = this->generation_;
    this->cv_.wait(lock, [this, my_gen] {
      return this->generation_ != my_gen || this->canceled_;
    });
  } else {
    this->arrived_ = 0;
    this->generation_++;
    this->cv_.notify_all();
  }
}

void RegionBarrier::cancel() {
  std::lock_guard<std::mutex> lock(this->mtx_);
  this->canceled_ = true;
  this->cv_.notify_all();
}

void RegionBarrier::reset() {
  std::lock_guard<std::mutex> lock(this->mtx_);
  this->party_count_ = this->initial_count_;
  this->arrived_ = 0;
  this->canceled_ = false;
  this->generation_++;
  this->cv_.notify_all();
}

int RegionBarrier::get_party_count() const { return this->initial_count_; }

} // namespace yasmin
