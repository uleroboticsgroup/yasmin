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

#ifndef YASMIN__STATE_UTILS_HPP_
#define YASMIN__STATE_UTILS_HPP_

#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

namespace yasmin {

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
