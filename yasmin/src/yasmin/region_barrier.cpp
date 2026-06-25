// Copyright (C) 2026 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
