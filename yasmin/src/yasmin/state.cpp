// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#include <algorithm>
#include <exception>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#ifdef __GNUG__     // If using GCC/G++
#include <cxxabi.h> // For abi::__cxa_demangle
#endif

#include "yasmin/logs.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

// ---------------------------------------------------------------------------
// External metadata storage — preserves ABI with plugins compiled against the
// older State class layout that did not include a StateMetadata member.
// ---------------------------------------------------------------------------
namespace {
std::mutex &metadata_mutex() {
  static std::mutex mtx;
  return mtx;
}
std::unordered_map<const State *, StateMetadata> &metadata_map() {
  static std::unordered_map<const State *, StateMetadata> map;
  return map;
}
} // namespace

StateMetadata &State::get_metadata_ref() const {
  std::lock_guard<std::mutex> lock(metadata_mutex());
  return metadata_map()[this];
}

State::State(const Outcomes &outcomes) : outcomes(outcomes) {
  if (outcomes.empty()) {
    throw std::logic_error("A state must have at least one possible outcome.");
  }
}

State::~State() {
  std::lock_guard<std::mutex> lock(metadata_mutex());
  metadata_map().erase(this);
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
  if (this->outcomes.find(outcome) == this->outcomes.end()) {

    // Construct a string representation of the possible outcomes
    std::string outcomes_string = "[";
    const auto &outcomes = this->get_outcomes();

    for (auto it = outcomes.begin(); it != outcomes.end(); ++it) {
      const auto &s = *it;
      outcomes_string += s;

      // Add a comma if this is not the last element
      if (std::next(it) != outcomes.end()) {
        outcomes_string += ", ";
      }
    }

    outcomes_string += "]";

    // Mark as idle before throwing exception
    this->set_status(StateStatus::IDLE);

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

const std::string &State::get_outcome_description(
    const std::string &outcome) const {
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

const StateMetadata &State::get_metadata() const {
  return this->get_metadata_ref();
}

std::string State::to_string() const {
  std::string name = typeid(*this).name();

#ifdef __GNUG__ // If using GCC/G++
  int status;
  // Demangle the name using GCC's demangling function
  char *demangled =
      abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status);
  if (status == 0) {
    name = demangled;
  }
  free(demangled);
#endif

  return name; // Return the demangled class name
}
