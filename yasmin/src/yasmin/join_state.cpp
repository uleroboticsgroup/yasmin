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

#include "yasmin/join_state.hpp"

namespace yasmin {

JoinState::JoinState() : State({"joined"}), sync_id_(""), outcome_("joined") {}

JoinState::JoinState(const std::string &sync_id, const std::string &outcome)
    : State({outcome}), sync_id_(sync_id), outcome_(outcome) {}

void JoinState::configure() {
  if (this->has_parameter("sync_id")) {
    this->sync_id_ = this->get_parameter<std::string>("sync_id");
  }
  if (this->has_parameter("outcome")) {
    this->outcome_ = this->get_parameter<std::string>("outcome");
  }
}

void JoinState::set_barrier(RegionBarrier::SharedPtr barrier) {
  barrier_ = std::move(barrier);
}

const std::string &JoinState::get_sync_id() const { return sync_id_; }

RegionBarrier::SharedPtr JoinState::get_barrier() const { return barrier_; }

std::string JoinState::execute(Blackboard::SharedPtr blackboard) {
  (void)blackboard;
  if (barrier_) {
    barrier_->arrive_and_wait();
  }
  return outcome_;
}

std::string JoinState::to_string() const {
  return std::string("JoinState [") + sync_id_ + "]";
}

} // namespace yasmin
