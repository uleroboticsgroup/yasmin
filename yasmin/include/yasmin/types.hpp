// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#ifndef YASMIN__TYPES_HPP_
#define YASMIN__TYPES_HPP_

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace yasmin {

/** @brief Macro to define a SharedPtr alias for a class */
#define YASMIN_SHARED_PTR_ALIAS(...)                                           \
  using SharedPtr = std::shared_ptr<__VA_ARGS__>;                              \
  using ConstSharedPtr = std::shared_ptr<const __VA_ARGS__>;                   \
  template <typename T = __VA_ARGS__,                                          \
            typename = std::enable_if_t<std::is_class_v<T>>, typename... Args> \
  static std::shared_ptr<T> make_shared(Args &&...args) {                      \
    return std::make_shared<T>(std::forward<Args>(args)...);                   \
  }

/** @brief Macro to define a UniquePtr alias for a class */
#define YASMIN_UNIQUE_PTR_ALIAS(...)                                           \
  using UniquePtr = std::unique_ptr<__VA_ARGS__>;                              \
  template <typename T = __VA_ARGS__,                                          \
            typename = std::enable_if_t<std::is_class_v<T>>, typename... Args> \
  static std::unique_ptr<T> make_unique(Args &&...args) {                      \
    return std::unique_ptr<T>(new T(std::forward<Args>(args)...));             \
  }

/** @brief Macro to define a WeakPtr alias for a class */
#define YASMIN_WEAK_PTR_ALIAS(...)                                             \
  using WeakPtr = std::weak_ptr<__VA_ARGS__>;                                  \
  using ConstWeakPtr = std::weak_ptr<const __VA_ARGS__>;

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
/** @brief Forward declaration for RegionBarrier class */
class RegionBarrier;
/** @brief Forward declaration for JoinState class */
class JoinState;
/** @brief Forward declaration for OrthogonalState class */
class OrthogonalState;

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
/** @brief Map of child parameter names to parent parameter names */
using ParameterMappings = StringMap;
/** @brief Map of state names to parameter mappings */
using ParameterMappingsMap = std::unordered_map<std::string, ParameterMappings>;
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
/** @brief Shared pointer to RegionBarrier */
using RegionBarrierPtr = std::shared_ptr<RegionBarrier>;
/** @brief Shared pointer to JoinState */
using JoinStatePtr = std::shared_ptr<JoinState>;
/** @brief Shared pointer to OrthogonalState */
using OrthogonalStatePtr = std::shared_ptr<OrthogonalState>;

/** @brief Map of state names to state pointers */
using StateMap = std::unordered_map<std::string, std::shared_ptr<State>>;

/** @brief Callback function type for CbState */
using CbStateCallback = std::function<std::string(std::shared_ptr<Blackboard>)>;

} // namespace yasmin

#endif // YASMIN__TYPES_HPP_
