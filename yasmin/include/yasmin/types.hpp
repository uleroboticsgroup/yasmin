// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#ifndef YASMIN__TYPES_HPP
#define YASMIN__TYPES_HPP

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

namespace yasmin {

/** @brief Macro to define a SharedPtr alias for a class */
#define YASMIN_SHARED_PTR_ALIAS(ClassName)                                     \
  using SharedPtr = std::shared_ptr<ClassName>;

/** @brief Macro to define a UniquePtr alias for a class */
#define YASMIN_UNIQUE_PTR_ALIAS(ClassName)                                     \
  using UniquePtr = std::unique_ptr<ClassName>;

/** @brief Macro to define a WeakPtr alias for a class */
#define YASMIN_WEAK_PTR_ALIAS(ClassName)                                       \
  using WeakPtr = std::weak_ptr<ClassName>;

/** @brief Macro to define all pointer aliases for a class */
#define YASMIN_PTR_ALIASES(ClassName)                                          \
  /**                                                                          \
   * @brief Shared pointer type for ClassName.                                 \
   */                                                                          \
  YASMIN_SHARED_PTR_ALIAS(ClassName)                                           \
                                                                               \
  /**                                                                          \
   * @brief Unique pointer type for ClassName.                                 \
   */                                                                          \
  YASMIN_UNIQUE_PTR_ALIAS(ClassName)                                           \
                                                                               \
  /**                                                                          \
   * @brief Weak pointer type for ClassName.                                   \
   */                                                                          \
  YASMIN_WEAK_PTR_ALIAS(ClassName)

/** @brief Forward declaration for State class */
class State;
/** @brief Forward declaration for Blackboard class */
class Blackboard;
/** @brief Forward declaration for StateMachine class */
class StateMachine;
/** @brief Forward declaration for Concurrence class */
class Concurrence;

/** @brief Set of strings */
using StringSet = std::set<std::string>;
/** @brief Map from string to string */
using StringMap = std::unordered_map<std::string, std::string>;

/** @brief Set of possible outcomes for states */
using Outcomes = StringSet;
/** @brief Map of state names to their outcomes */
using StateOutcomeMap = StringMap;
/** @brief Map of outcomes to state outcome maps */
using OutcomeMap = std::unordered_map<std::string, StateOutcomeMap>;

/** @brief Map of transitions (string to string) */
using Transitions = StringMap;
/** @brief Map of state names to transitions */
using TransitionsMap = std::unordered_map<std::string, Transitions>;
/** @brief Map of remappings (string to string) */
using Remappings = StringMap;
/** @brief Map of keys to remappings */
using RemappingsMap = std::unordered_map<std::string, Remappings>;
/** @brief Registry for type information */
using TypeRegistry = StringMap;

/** @brief Shared pointer to Blackboard */
using BlackboardPtr = std::shared_ptr<Blackboard>;
/** @brief Shared pointer to State */
using StatePtr = std::shared_ptr<State>;
/** @brief Shared pointer to StateMachine */
using StateMachinePtr = std::shared_ptr<StateMachine>;
/** @brief Shared pointer to Concurrence */
using ConcurrencePtr = std::shared_ptr<Concurrence>;

/** @brief Map of state names to state pointers */
using StateMap = std::unordered_map<std::string, std::shared_ptr<State>>;

/** @brief Callback function type for CbState */
using CbStateCallback = std::function<std::string(std::shared_ptr<Blackboard>)>;

} // namespace yasmin

#endif // YASMIN__TYPES_HPP
