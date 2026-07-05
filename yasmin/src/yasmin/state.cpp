// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#include "yasmin/state.hpp"

#include <algorithm>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "yasmin/demangle.hpp"
#include "yasmin/logs.hpp"
#include "yasmin/state_utils.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

// ---------------------------------------------------------------------------
// External state storage — preserves ABI with plugins compiled against the
// older State class layout that did not include metadata or parameter storage.
// ---------------------------------------------------------------------------
namespace {
struct StateStorage {
  StateMetadata metadata;
  Blackboard::SharedPtr parameters = Blackboard::make_shared();
};

std::recursive_mutex &state_storage_mutex() {
  static std::recursive_mutex mtx;
  return mtx;
}

std::unordered_map<const State *, StateStorage> &state_storage_map() {
  static std::unordered_map<const State *, StateStorage> map;
  return map;
}
} // namespace

StateMetadata &State::get_metadata_ref() const {
  std::lock_guard<std::recursive_mutex> lock(state_storage_mutex());
  return state_storage_map()[this].metadata;
}

Blackboard::SharedPtr State::get_parameters_blackboard() const {
  std::lock_guard<std::recursive_mutex> lock(state_storage_mutex());
  return state_storage_map()[this].parameters;
}

State::State(Outcomes outcomes) {
  if (outcomes.empty()) {
    throw std::logic_error("A state must have at least one possible outcome.");
  }
  this->outcomes = std::move(outcomes);
}

State::~State() {
  std::lock_guard<std::recursive_mutex> lock(state_storage_mutex());
  state_storage_map().erase(this);
}

void State::set_status(StateStatus new_status) {
  this->status.store(new_status);
}

StateStatus State::get_status() const { return this->status.load(); }

bool State::is_idle() const noexcept {
  return this->get_status() == StateStatus::IDLE;
}

bool State::is_running() const noexcept {
  return this->get_status() == StateStatus::RUNNING;
}

bool State::is_canceled() const noexcept {
  return this->get_status() == StateStatus::CANCELED;
}

bool State::is_completed() const noexcept {
  return this->get_status() == StateStatus::COMPLETED;
}

std::string State::operator()(Blackboard::SharedPtr blackboard) {

  YASMIN_LOG_DEBUG("Executing state '%s'", this->to_string().c_str());
  this->set_status(StateStatus::RUNNING);

  // Inject default values for input keys that are missing from the blackboard
  const auto &input_keys = this->get_metadata_ref().input_keys;
  for (const auto &key_info : input_keys) {
    if (key_info.has_default && !blackboard->contains(key_info.name)) {
      YASMIN_LOG_DEBUG(
          "Injecting default value for input key '%s' in state '%s'",
          key_info.name.c_str(), this->to_string().c_str());
      key_info.inject_default(*blackboard, key_info.name);
    }
  }

  // Execute the specific logic of the state
  std::string outcome = this->execute(blackboard);

  // Check if the outcome is valid
  const auto &outcomes = this->get_outcomes();
  if (outcomes.find(outcome) == outcomes.end()) {

    // Mark as idle before throwing exception
    this->set_status(StateStatus::IDLE);

    if (outcome.empty()) {
      throw std::logic_error("State '" + this->to_string() +
                             "' returned an empty outcome. "
                             "Did you forget to override execute()?");
    }

    // Construct a string representation of the possible outcomes
    std::string outcomes_string =
        "[" +
        yasmin::join(outcomes, ", ", [](const std::string &o) { return o; }) +
        "]";

    // Throw an exception if the outcome is not valid
    throw std::logic_error("Outcome '" + outcome +
                           "' does not belong to the outcomes of "
                           "the state '" +
                           this->to_string() +
                           "'. The possible outcomes are: " + outcomes_string);
  }

  // Mark as completed if not canceled
  if (!this->is_canceled()) {
    this->set_status(StateStatus::COMPLETED);
  }

  return outcome; // Return the valid outcome
}

Outcomes const &State::get_outcomes() const noexcept { return this->outcomes; }

void State::set_description(const std::string &description) {
  this->get_metadata_ref().description = description;
}

const std::string &State::get_description() const {
  return this->get_metadata_ref().description;
}

void State::set_outcome_description(const std::string &outcome,
                                    const std::string &description) {
  if (this->outcomes.find(outcome) == this->outcomes.end()) {
    throw std::invalid_argument("Outcome '" + outcome +
                                "' is not part of state outcomes");
  }

  this->get_metadata_ref().outcome_descriptions[outcome] = description;
}

const std::string &
State::get_outcome_description(const std::string &outcome) const {
  if (this->outcomes.find(outcome) == this->outcomes.end()) {
    throw std::invalid_argument("Outcome '" + outcome +
                                "' is not part of state outcomes");
  }

  const auto &outcome_descriptions =
      this->get_metadata_ref().outcome_descriptions;
  const auto it = outcome_descriptions.find(outcome);

  static const std::string empty_description;
  if (it == outcome_descriptions.end()) {
    return empty_description;
  }

  return it->second;
}

const std::unordered_map<std::string, std::string> &
State::get_outcome_descriptions() const {
  return this->get_metadata_ref().outcome_descriptions;
}

void State::add_input_key(const BlackboardKeyInfo &key_info) {
  this->get_metadata_ref().input_keys.push_back(key_info);
}

void State::add_input_key(const std::string &key_name) {
  this->get_metadata_ref().input_keys.emplace_back(key_name);
}

void State::add_input_key(const std::string &key_name,
                          const std::string &description) {
  BlackboardKeyInfo info(key_name);
  info.description = description;
  this->get_metadata_ref().input_keys.push_back(info);
}

void State::add_output_key(const BlackboardKeyInfo &key_info) {
  BlackboardKeyInfo info = key_info;
  info.has_default = false;
  info.default_value.reset();
  info.default_value_type.clear();
  info.inject_default = {};
  this->get_metadata_ref().output_keys.push_back(info);
}

void State::add_output_key(const std::string &key_name) {
  this->get_metadata_ref().output_keys.emplace_back(key_name);
}

void State::add_output_key(const std::string &key_name,
                           const std::string &description) {
  BlackboardKeyInfo info(key_name);
  info.description = description;
  this->get_metadata_ref().output_keys.push_back(info);
}

const std::vector<BlackboardKeyInfo> &State::get_input_keys() const {
  return this->get_metadata_ref().input_keys;
}

const std::vector<BlackboardKeyInfo> &State::get_output_keys() const {
  return this->get_metadata_ref().output_keys;
}

void State::declare_parameter(const BlackboardKeyInfo &parameter_info) {
  auto &parameters = this->get_metadata_ref().parameters;

  auto it = std::find_if(parameters.begin(), parameters.end(),
                         [&](const BlackboardKeyInfo &info) {
                           return info.name == parameter_info.name;
                         });

  if (it == parameters.end()) {
    parameters.push_back(parameter_info);
  } else {
    *it = parameter_info;
  }

  if (parameter_info.has_default) {
    parameter_info.inject_default(*this->get_parameters_blackboard(),
                                  parameter_info.name);
  }
}

void State::declare_parameter(const std::string &parameter_name) {
  this->declare_parameter(BlackboardKeyInfo(parameter_name));
}

void State::declare_parameter(const std::string &parameter_name,
                              const std::string &description) {
  this->declare_parameter(BlackboardKeyInfo(parameter_name, description));
}

bool State::has_parameter(const std::string &parameter_name) const {
  return this->get_parameters_blackboard()->contains(parameter_name);
}

bool State::is_parameter_declared(const std::string &parameter_name) const {
  const auto &parameters = this->get_metadata_ref().parameters;
  return std::any_of(parameters.begin(), parameters.end(),
                     [&](const BlackboardKeyInfo &info) {
                       return info.name == parameter_name;
                     });
}

void State::copy_parameter_from(const State &source_state,
                                const std::string &source_parameter_name,
                                const std::string &target_parameter_name) {
  this->get_parameters_blackboard()->copy_value_from(
      *source_state.get_parameters_blackboard(), source_parameter_name,
      target_parameter_name);
}

const std::vector<BlackboardKeyInfo> &State::get_parameters() const {
  return this->get_metadata_ref().parameters;
}

const StateMetadata &State::get_metadata() const {
  return this->get_metadata_ref();
}

std::string State::to_string() const {
  return demangle_type(typeid(*this).name());
}
