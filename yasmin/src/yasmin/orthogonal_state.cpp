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

#include "yasmin/orthogonal_state.hpp"

#include <algorithm>
#include <exception>
#include <set>
#include <stdexcept>

#include "yasmin/logs.hpp"

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
    : State(generate_possible_outcomes(outcome_map, default_outcome)),
      outcome_map_(outcome_map), default_outcome_(default_outcome) {}

void OrthogonalState::add_region(const std::string &name,
                                 StateMachine::SharedPtr sm) {
  if (!sm) {
    throw std::invalid_argument("Region state machine cannot be null");
  }
  for (const auto &r : regions_) {
    if (r.name == name) {
      throw std::invalid_argument("Region '" + name + "' already exists");
    }
  }
  regions_.push_back({name, std::move(sm)});
  configured_.store(false);
}

const std::vector<OrthogonalState::RegionDescriptor> &
OrthogonalState::get_regions() const noexcept {
  return regions_;
}

const OutcomeMap &OrthogonalState::get_outcome_map() const noexcept {
  return outcome_map_;
}

const std::string &OrthogonalState::get_default_outcome() const noexcept {
  return default_outcome_;
}

void OrthogonalState::configure() {
  if (configured_.load()) {
    YASMIN_LOG_DEBUG("OrthogonalState '%s' has already been configured",
                     this->to_string().c_str());
    return;
  }

  YASMIN_LOG_DEBUG("Configuring orthogonal state '%s'",
                   this->to_string().c_str());

  // Collect all JoinStates grouped by sync_id
  std::unordered_map<std::string, std::vector<JoinState *>> join_groups;

  for (auto &region : regions_) {
    for (const auto &[state_name, state] : region.sm->get_states()) {
      (void)state_name;
      JoinState *js = dynamic_cast<JoinState *>(state.get());
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
    barriers_[sync_id] = barrier;
    YASMIN_LOG_DEBUG("Created barrier '%s' with %zu participants",
                     sync_id.c_str(), join_states.size());
  }

  // Configure all regions
  for (auto &region : regions_) {
    region.sm->configure();
  }

  configured_.store(true);
}

std::string OrthogonalState::execute(Blackboard::SharedPtr blackboard) {
  this->configure();

  if (regions_.empty()) {
    return default_outcome_;
  }

  std::vector<std::thread> threads;
  std::vector<std::string> region_outcomes(regions_.size());
  std::vector<std::exception_ptr> exceptions(regions_.size(), nullptr);

  // Invoke before-fork hook (e.g., GIL release when Python states are used)
  if (before_fork_hook_) {
    before_fork_hook_();
  }

  // Fork: launch all regions
  for (size_t i = 0; i < regions_.size(); i++) {
    auto bb_copy = std::make_shared<Blackboard>(*blackboard);
    threads.push_back(
        std::thread([this, i, bb_copy, &region_outcomes, &exceptions]() {
          try {
            region_outcomes[i] = regions_[i].sm->execute(bb_copy);
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
        YASMIN_LOG_ERROR("Region '%s' threw: %s", regions_[i].name.c_str(),
                         e.what());
        throw;
      } catch (...) {
        YASMIN_LOG_ERROR("Region '%s' threw unknown exception",
                         regions_[i].name.c_str());
        throw;
      }
    }
  }

  // Handle cancel
  if (is_canceled()) {
    return default_outcome_;
  }

  // Evaluate outcome map
  return evaluate_outcomes(region_outcomes);
}

void OrthogonalState::cancel_state() {
  // Unblock all barriers first
  for (auto &[id, barrier] : barriers_) {
    (void)id;
    barrier->cancel();
  }
  // Cancel each region's state machine
  for (auto &region : regions_) {
    if (region.sm->is_running()) {
      region.sm->cancel_state_machine();
    }
  }
  State::cancel_state();
}

std::string OrthogonalState::evaluate_outcomes(
    const std::vector<std::string> &region_outcomes) const {
  std::set<std::string> satisfied_outcomes;

  for (const auto &[outcome, requirements] : outcome_map_) {
    bool satisfied = true;
    for (const auto &[state_name, expected_outcome] : requirements) {
      // Find the region by name
      bool found = false;
      for (size_t i = 0; i < regions_.size(); i++) {
        if (regions_[i].name == state_name) {
          if (region_outcomes[i] != expected_outcome) {
            satisfied = false;
          }
          found = true;
          break;
        }
      }
      if (!found) {
        satisfied = false;
      }
    }
    if (satisfied) {
      satisfied_outcomes.insert(outcome);
    }
  }

  if (satisfied_outcomes.empty()) {
    return default_outcome_;
  } else if (satisfied_outcomes.size() > 1) {
    std::string outcomes_string;
    for (auto it = satisfied_outcomes.begin(); it != satisfied_outcomes.end();
         ++it) {
      outcomes_string += *it;
      if (std::next(it) != satisfied_outcomes.end()) {
        outcomes_string += ", ";
      }
    }
    throw std::logic_error(
        "More than one satisfied outcomes (" + outcomes_string +
        ") after orthogonal state execution (" + this->to_string() + ")");
  }

  return *satisfied_outcomes.begin();
}

Outcomes OrthogonalState::generate_possible_outcomes(
    const OutcomeMap &outcome_map, const std::string &default_outcome) {
  Outcomes possible_outcomes;
  possible_outcomes.insert(default_outcome);
  for (const auto &[outcome, requirements] : outcome_map) {
    (void)requirements;
    possible_outcomes.insert(outcome);
  }
  return possible_outcomes;
}

std::string OrthogonalState::to_string() const {
  std::string name = "OrthogonalState [";

  for (auto it = regions_.begin(); it != regions_.end(); ++it) {
    name += it->name + " (" + it->sm->get_name() + ")";
    if (std::next(it) != regions_.end()) {
      name += ", ";
    }
  }

  name += "]";
  return name;
}

} // namespace yasmin
