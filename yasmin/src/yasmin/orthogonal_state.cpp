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

#include "yasmin/orthogonal_state.hpp"

#include <algorithm>
#include <exception>
#include <stdexcept>
#include <thread>

#include "yasmin/logs.hpp"
#include "yasmin/state_utils.hpp"

namespace yasmin {

OrthogonalState::GilHook OrthogonalState::before_fork_hook_ = nullptr;
OrthogonalState::GilHook OrthogonalState::after_join_hook_ = nullptr;

void OrthogonalState::set_thread_hooks(GilHook before_fork,
                                       GilHook after_join) {
  before_fork_hook_ = before_fork;
  after_join_hook_ = after_join;
}

OrthogonalState::OrthogonalState(const std::string &default_outcome,
                                 const OutcomeMap &outcome_map)
    : State(yasmin::generate_possible_outcomes(outcome_map, default_outcome)),
      outcome_map_(outcome_map), default_outcome_(default_outcome) {}

void OrthogonalState::add_region(const std::string &name,
                                 StateMachine::SharedPtr sm) {
  if (!sm) {
    throw std::invalid_argument("Region state machine cannot be null");
  }
  for (const auto &r : this->regions_) {
    if (r.name == name) {
      throw std::invalid_argument("Region '" + name + "' already exists");
    }
  }
  this->regions_.push_back({name, std::move(sm)});
  this->configured_.store(false);
}

const std::vector<OrthogonalState::RegionDescriptor> &
OrthogonalState::get_regions() const noexcept {
  return this->regions_;
}

const OutcomeMap &OrthogonalState::get_outcome_map() const noexcept {
  return this->outcome_map_;
}

const std::string &OrthogonalState::get_default_outcome() const noexcept {
  return this->default_outcome_;
}

void OrthogonalState::configure() {
  if (yasmin::check_already_configured(this->configured_, "OrthogonalState",
                                       this->to_string().c_str()))
    return;

  // Collect all JoinStates grouped by sync_id
  std::unordered_map<std::string, std::vector<JoinState *>> join_groups;

  for (auto &region : this->regions_) {
    for (const auto &[state_name, state] : region.sm->get_states()) {
      (void)state_name;
      JoinState *js = dynamic_cast<JoinState *>(state->get_inner_state());
      if (js) {
        join_groups[js->get_sync_id()].push_back(js);
      }
    }
  }

  // Create barriers for each group
  for (auto &[sync_id, join_states] : join_groups) {
    if (join_states.size() < 2) {
      throw std::runtime_error("JoinState sync_id '" + sync_id +
                               "' has < 2 participants; "
                               "each sync point needs at least 2 regions");
    }
    auto barrier =
        std::make_shared<RegionBarrier>(static_cast<int>(join_states.size()));
    for (auto *js : join_states) {
      js->set_barrier(barrier);
    }
    this->barriers_[sync_id] = barrier;
    YASMIN_LOG_DEBUG("Created barrier '%s' with %zu participants",
                     sync_id.c_str(), join_states.size());
  }

  // Build region name -> index map for O(1) lookups
  this->region_name_to_index_.clear();
  this->region_name_to_index_.reserve(this->regions_.size());
  for (size_t i = 0; i < this->regions_.size(); i++) {
    this->region_name_to_index_[this->regions_[i].name] = i;
  }

  // Configure all regions
  for (auto &region : this->regions_) {
    region.sm->configure();
  }

  this->configured_.store(true);
}

std::string OrthogonalState::execute(Blackboard::SharedPtr blackboard) {
  this->configure();

  if (this->regions_.empty()) {
    return this->default_outcome_;
  }

  std::vector<std::thread> threads;
  std::vector<std::string> region_outcomes(this->regions_.size());
  std::vector<std::exception_ptr> exceptions(this->regions_.size(), nullptr);

  // Invoke before-fork hook (e.g., GIL release when Python states are used)
  if (before_fork_hook_) {
    before_fork_hook_();
  }

  // Fork: launch all regions
  for (size_t i = 0; i < this->regions_.size(); i++) {
    auto bb_copy = std::make_shared<Blackboard>(*blackboard);
    threads.push_back(
        std::thread([this, i, bb_copy, &region_outcomes, &exceptions]() {
          try {
            region_outcomes[i] = this->regions_[i].sm->execute(bb_copy);
          } catch (...) {
            exceptions[i] = std::current_exception();
          }
        }));
  }

  // Join: wait for all regions to finish
  for (auto &t : threads) {
    if (t.joinable()) {
      t.join();
    }
  }

  // Invoke after-join hook (e.g., GIL re-acquire when Python states are used)
  if (after_join_hook_) {
    after_join_hook_();
  }

  // Check for exceptions
  for (size_t i = 0; i < exceptions.size(); i++) {
    if (exceptions[i]) {
      try {
        std::rethrow_exception(exceptions[i]);
      } catch (const std::exception &e) {
        YASMIN_LOG_ERROR("Region '%s' threw: %s",
                         this->regions_[i].name.c_str(), e.what());
        throw;
      } catch (...) {
        YASMIN_LOG_ERROR("Region '%s' threw unknown exception",
                         this->regions_[i].name.c_str());
        throw;
      }
    }
  }

  // Handle cancel
  if (this->is_canceled()) {
    return this->default_outcome_;
  }

  // Evaluate outcome map
  return this->evaluate_outcomes(region_outcomes);
}

void OrthogonalState::cancel_state() {
  // Unblock all barriers first
  for (auto &[id, barrier] : this->barriers_) {
    (void)id;
    barrier->cancel();
  }
  // Cancel each region's state machine
  for (auto &region : this->regions_) {
    if (region.sm->is_running()) {
      region.sm->cancel_state_machine();
    }
  }
  State::cancel_state();
}

std::string OrthogonalState::evaluate_outcomes(
    const std::vector<std::string> &region_outcomes) const {
  Outcomes satisfied_outcomes = yasmin::evaluate_satisfied_outcomes(
      this->outcome_map_,
      [this, &region_outcomes](const std::string &state_name) -> std::string {
        auto it = this->region_name_to_index_.find(state_name);
        if (it != this->region_name_to_index_.end()) {
          return region_outcomes[it->second];
        }
        return "";
      });

  return yasmin::resolve_outcome(satisfied_outcomes, this->default_outcome_,
                                 "orthogonal state execution (" +
                                     this->to_string() + ")");
}

void OrthogonalState::validate(bool strict_mode) {
  for (auto &region : this->regions_) {
    region.sm->validate(strict_mode);
  }
}

std::string OrthogonalState::to_string() const {
  return "OrthogonalState [" +
         yasmin::join(this->regions_, ", ",
                      [](const auto &r) {
                        return r.name + " (" + r.sm->get_name() + ")";
                      }) +
         "]";
}

} // namespace yasmin
