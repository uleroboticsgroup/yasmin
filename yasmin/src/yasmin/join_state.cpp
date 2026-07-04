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
  this->barrier_ = std::move(barrier);
}

const std::string &JoinState::get_sync_id() const { return this->sync_id_; }

RegionBarrier::SharedPtr JoinState::get_barrier() const {
  return this->barrier_;
}

std::string JoinState::execute(Blackboard::SharedPtr blackboard) {
  (void)blackboard;
  if (this->barrier_) {
    this->barrier_->arrive_and_wait();
  }
  return this->outcome_;
}

std::string JoinState::to_string() const {
  return std::string("JoinState [") + this->sync_id_ + "]";
}

} // namespace yasmin
