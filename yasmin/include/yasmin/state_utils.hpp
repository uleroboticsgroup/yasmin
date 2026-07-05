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

#ifndef YASMIN__STATE_UTILS_HPP_
#define YASMIN__STATE_UTILS_HPP_

#include <atomic>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

namespace yasmin {

/**
 * @brief Generate the set of possible outcomes from an outcome map and default.
 * @param outcome_map The outcome map of conditional outcomes.
 * @param default_outcome The default outcome string.
 * @return A set containing the default outcome and all outcome map keys.
 */
inline Outcomes generate_possible_outcomes(const OutcomeMap &outcome_map,
                                           const std::string &default_outcome) {
  Outcomes possible_outcomes;
  possible_outcomes.insert(default_outcome);

  for (const auto &[outcome, requirements] : outcome_map) {
    (void)requirements;
    possible_outcomes.insert(outcome);
  }

  return possible_outcomes;
}

/**
 * @brief Join outcome strings into a single string separated by a delimiter.
 * @param items The set of outcome strings.
 * @param separator The separator between outcomes (default ", ").
 * @return The joined string.
 */
inline std::string join_outcomes(const Outcomes &items,
                                 const std::string &separator = ", ") {
  std::string result;
  for (auto it = items.begin(); it != items.end(); ++it) {
    if (it != items.begin()) {
      result += separator;
    }
    result += *it;
  }
  return result;
}

/**
 * @brief Join elements of a container into a string using a delimiter.
 * @tparam Container The container type.
 * @tparam StringFn A callable that converts an element to a string.
 * @param container The container to join.
 * @param delim The delimiter between elements.
 * @param fn The function that converts each element to a string.
 * @return The joined string.
 */
template <typename Container, typename StringFn>
std::string join(const Container &container, const std::string &delim,
                 StringFn fn) {
  std::ostringstream oss;
  for (auto it = container.begin(); it != container.end(); ++it) {
    if (it != container.begin())
      oss << delim;
    oss << fn(*it);
  }
  return oss.str();
}

/**
 * @brief Check if an object has already been configured, logging as a side
 * effect.
 * @param configured The atomic flag to check.
 * @param type_name The human-readable type name.
 * @param instance_name The stringified instance name.
 * @return true if already configured, false otherwise.
 */
inline bool check_already_configured(std::atomic<bool> &configured,
                                     const char *type_name,
                                     const char *instance_name) {
  if (configured.load()) {
    YASMIN_LOG_DEBUG("%s '%s' has already been configured", type_name,
                     instance_name);
    return true;
  }

  YASMIN_LOG_DEBUG("Configuring %s '%s'", type_name, instance_name);
  return false;
}

/**
 * @brief Evaluate which outcomes have all their requirements satisfied.
 * @tparam T The callable type used to look up actual outcomes.
 * @param outcome_map The map of outcomes to their requirements.
 * @param intermediate_outcome_map A callable that returns the actual outcome
 * for a state name.
 * @return A set of outcomes whose requirements are all satisfied.
 */
template <typename LookupFn>
Outcomes evaluate_satisfied_outcomes(const OutcomeMap &outcome_map,
                                     LookupFn get_actual) {
  Outcomes satisfied_outcomes;

  for (const auto &[outcome, requirements] : outcome_map) {
    bool satisfied = true;

    for (const auto &[state_name, expected_outcome] : requirements) {
      if (get_actual(state_name) != expected_outcome) {
        satisfied = false;
        break;
      }
    }

    if (satisfied) {
      satisfied_outcomes.insert(outcome);
    }
  }

  return satisfied_outcomes;
}

/**
 * @brief Resolve the final outcome from a set of satisfied outcomes.
 * @param outcome The set of satisfied outcomes.
 * @param transition_map The default outcome if no outcomes are satisfied.
 * @param current_state A context name used in error messages.
 * @return The single satisfied outcome, or the default if none are satisfied.
 * @throws std::logic_error If more than one outcome is satisfied.
 */
inline std::string resolve_outcome(const Outcomes &satisfied_outcomes,
                                   const std::string &default_outcome,
                                   const std::string &context_name = "") {
  if (satisfied_outcomes.empty()) {
    return default_outcome;
  }

  if (satisfied_outcomes.size() > 1) {
    throw std::logic_error("More than one satisfied outcomes (" +
                           join_outcomes(satisfied_outcomes) + ") after " +
                           context_name);
  }

  return *satisfied_outcomes.begin();
}

/**
 * @brief Apply parameter mappings from a parent state to a child state.
 * @param blackboard The container type string used in error messages.
 * @param param_mappings The map of parameter mappings from child to parent.
 * @param state_name The name of the child state.
 */
inline void
apply_parameter_mappings(const std::string &container_type,
                         const ParameterMappingsMap &parameter_mappings,
                         const std::string &state_name, State &parent,
                         const State::SharedPtr &child) {

  const auto mappings_it = parameter_mappings.find(state_name);
  if (mappings_it == parameter_mappings.end()) {
    return;
  }

  for (const auto &[child_parameter, parent_parameter] : mappings_it->second) {
    if (!parent.is_parameter_declared(parent_parameter)) {
      throw std::runtime_error(
          container_type + " parameter '" + parent_parameter +
          "' is not declared while configuring child state '" + state_name +
          "'");
    }

    if (!parent.has_parameter(parent_parameter)) {
      throw std::runtime_error(
          container_type + " parameter '" + parent_parameter +
          "' has no value while configuring child state '" + state_name + "'");
    }

    if (!child->is_parameter_declared(child_parameter)) {
      throw std::runtime_error("Child state '" + state_name +
                               "' does not declare parameter '" +
                               child_parameter + "'");
    }

    child->copy_parameter_from(parent, parent_parameter, child_parameter);
  }
}

} // namespace yasmin

#endif // YASMIN__STATE_UTILS_HPP_
