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
  std::unique_lock<std::mutex> lock(mtx_);
  arrived_++;
  if (arrived_ < party_count_) {
    int my_gen = generation_;
    cv_.wait(lock,
             [this, my_gen] { return generation_ != my_gen || canceled_; });
  } else {
    arrived_ = 0;
    generation_++;
    cv_.notify_all();
  }
}

void RegionBarrier::cancel() {
  std::lock_guard<std::mutex> lock(mtx_);
  canceled_ = true;
  cv_.notify_all();
}

void RegionBarrier::reset() {
  std::lock_guard<std::mutex> lock(mtx_);
  party_count_ = initial_count_;
  arrived_ = 0;
  canceled_ = false;
  generation_++;
  cv_.notify_all();
}

int RegionBarrier::get_party_count() const { return initial_count_; }

} // namespace yasmin
