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

#ifndef YASMIN__STATE_MACHINE_HPP_
#define YASMIN__STATE_MACHINE_HPP_

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine_cancel_exception.hpp"

namespace yasmin {

/**
 * @class StateMachine
 * @brief A class that implements a state machine with a set of states,
 *        transitions, and callback mechanisms for state changes.
 *
 * The StateMachine class inherits from the State class and allows the
 * registration of states with their respective transitions and callbacks
 * for start, transition, and end events.
 */
class StateMachine : public State {

public:
  /// Alias for a callback function executed before running the state machine.
  using StartCallbackType =
      std::function<void(const Blackboard::SharedPtr &, const std::string &)>;
  /// Alias for a callback function executed before changing the state.
  using TransitionCallbackType =
      std::function<void(const Blackboard::SharedPtr &, const std::string &,
                         const std::string &, const std::string &)>;
  /// Alias for a callback function executed after running the state machine.
  using EndCallbackType =
      std::function<void(const Blackboard::SharedPtr &, const std::string &)>;

  /**
   * @brief Shared pointer type for StateMachine.
   */
  YASMIN_PTR_ALIASES(StateMachine)

  /**
   * @brief Construct a new StateMachine object.
   *
   * @param outcomes A set of possible outcomes for the state machine.
   * @param handle_sigint Whether to handle SIGINT for canceling the state
   * machine.
   */
  StateMachine(const Outcomes &outcomes, bool handle_sigint = false);

  /**
   * @brief Construct a new StateMachine object.
   *
   * @param name The name of the state machine.
   * @param outcomes A set of possible outcomes for the state machine.
   * @param handle_sigint Whether to handle SIGINT for canceling the state
   * machine.
   */
  StateMachine(const std::string &name, const Outcomes &outcomes,
               bool handle_sigint = false);

  /**
   * @brief Destroy the StateMachine object.
   */
  ~StateMachine();

  /**
   * @brief Adds a state to the state machine with specified transitions.
   *
   * @param name The name of the state.
   * @param state A shared pointer to the State object representing the new
   * state.
   * @param transitions A map of transitions where the key is the outcome
   *                    and the value is the target state name.
   * @param remappings A map of remappings keys for the blackboard.
   * @param parameter_mappings Per-child parameter mappings.
   * @throws std::logic_error If the state is already registered or is an
   * outcome.
   * @throws std::invalid_argument If any transition has empty source or target,
   *                               or references unregistered outcomes.
   */
  void add_state(const std::string &name, State::SharedPtr state,
                 const Transitions &transitions = {},
                 const Remappings &remappings = {},
                 const ParameterMappings &parameter_mappings = {});

  /**
   * @brief Sets the name of the state machine.
   *
   * @param name The name to set for the state machine.
   */
  void set_name(const std::string &name) { this->name = name; }

  /**
   * @brief Gets the name of the state machine.
   *
   * @return The name of the state machine.
   */
  std::string get_name() const noexcept { return this->name; }

  /**
   * @brief Sets the start state for the state machine.
   *
   * @param state_name The name of the state to set as the start state.
   * @throws std::invalid_argument If the state name is empty or not registered.
   */
  void set_start_state(const std::string &state_name);

  /**
   * @brief Retrieves the name of the start state.
   *
   * @return The name of the start state.
   */
  std::string const &get_start_state() const noexcept;

  /**
   * @brief Gets a constant reference to the map of states.
   *
   * @return A constant reference to the map of states.
   */
  StateMap const &get_states() const noexcept;

  /**
   * @brief Gets a constant reference to the map of transitions.
   *
   * @return A constant reference to the map of transitions.
   */
  TransitionsMap const &get_transitions() const noexcept;

  /**
   * @brief Retrieves the current state name.
   *
   * @return The name of the current state.
   */
  std::string get_current_state() const;

  /**
   * @brief Adds a callback function to be called when the state machine starts.
   *
   * @param cb The callback function to execute.
   */
  void add_start_cb(StartCallbackType cb);

  /**
   * @brief Adds a callback function for state transitions.
   *
   * @param cb The callback function to execute.
   */
  void add_transition_cb(TransitionCallbackType cb);

  /**
   * @brief Adds a callback function to be called when the state machine ends.
   *
   * @param cb The callback function to execute.
   */
  void add_end_cb(EndCallbackType cb);

  /**
   * @brief Sets parameter mappings for a child state.
   *
   * Each mapping entry is interpreted as:
   *   child_parameter -> parent_parameter
   *
   * @param state_name The child state name.
   * @param parameter_mappings The parameter mappings to apply.
   */
  void set_parameter_mappings(const std::string &state_name,
                              const ParameterMappings &parameter_mappings);

  /**
   * @brief Gets all parameter mappings.
   * @return A constant reference to the parameter mappings.
   */
  const ParameterMappingsMap &get_parameter_mappings() const noexcept;

  /**
   * @brief Validates the state machine configuration.
   *
   * @param strict_mode Whether the validation is strict, which means checking
   * if all state outcomes are used and all state machine outcomes are reached.
   * @throws std::runtime_error If the state machine is misconfigured.
   */
  void validate(bool strict_mode = false);

  /**
   * @brief Configures the state machine and all child states.
   */
  void configure() override;

  /**
   * @brief Executes the state machine.
   *
   * @param blackboard A shared pointer to the blackboard used during execution.
   * @return The outcome of the state machine execution.
   * @throws std::runtime_error If the execution cannot be completed due to
   *                            invalid states or transitions.
   */
  std::string execute(Blackboard::SharedPtr blackboard) override;

  /**
   * @brief Executes the state machine using a default blackboard.
   *
   * @return The outcome of the state machine execution.
   */
  std::string execute();

  /**
   * @brief Invokes the state machine using a default blackboard.
   *
   * @return The outcome of the state machine execution.
   */
  std::string operator()();

  // Use the base class operator()
  using State::operator();

  /**
   * @brief Cancels the currently active child state.
   *
   * This lightweight cancel request is forwarded to the active child state. If
   * the state machine continues executing afterwards, its running status is
   * restored inside the execution loop so later cancel requests can still be
   * propagated.
   */
  void cancel_state() override;

  /**
   * @brief Cancels the complete state machine execution.
   *
   * This requests a hard stop of the current state machine. The request is
   * propagated recursively to an active child state machine and causes the
   * execution loop to abort by throwing StateMachineCancelException.
   */
  void cancel_state_machine();

  /**
   * @brief Sets whether the state machine should handle SIGINT for cancel.
   * @param handle True to handle SIGINT, false to ignore or reset the handler.
   */
  void set_sigint_handler(bool handle = true);

  /**
   * @brief Converts the state machine to a string representation.
   *
   * @return A string describing the state machine and its states.
   */
  std::string to_string() const override;

private:
  // Name of the state machine (used if this is the root state machine)
  std::string name;

  /// Map of states
  StateMap states;
  /// Map of transitions
  TransitionsMap transitions;
  /// A dictionary of remappings to set in the blackboard in each transition
  RemappingsMap remappings;
  /// Per-child parameter mappings applied during configure()
  ParameterMappingsMap parameter_mappings;

  /// Name of the start state
  std::string start_state;
  /// Name of the current state
  std::string current_state;
  /// Mutex for current state access
  std::unique_ptr<std::mutex> current_state_mutex;
  /// Condition variable for current state changes
  std::condition_variable current_state_cond;

  /// Flag to indicate if the state machine has been validated
  std::atomic_bool validated{false};
  /// Flag to indicate if the state machine has been configured
  std::atomic_bool configured{false};
  /// Flag to indicate whether the execute loop is currently active
  std::atomic_bool execution_active{false};
  /// Flag to indicate that a hard state machine cancel was requested
  std::atomic_bool cancel_state_machine_requested{false};

  /// Start callbacks executed before the state machine
  std::vector<StartCallbackType> start_cbs;
  /// Transition callbacks executed before changing the state
  std::vector<TransitionCallbackType> transition_cbs;
  /// End callbacks executed after the state machine finishes
  std::vector<EndCallbackType> end_cbs;

  /**
   * @brief Sets the current state name.
   *
   * @param state_name The name of the state to set as the current state.
   */
  void set_current_state(const std::string &state_name);

  /**
   * @brief Applies this container's parameter mappings to a direct child.
   * @param state_name The child state name.
   * @param state The child state instance.
   */
  void apply_parameter_mappings(const std::string &state_name,
                                const State::SharedPtr &state);

  /**
   * @brief Calls start callbacks with the given blackboard and start state.
   *
   * @param blackboard A shared pointer to the blackboard.
   * @param start_state The name of the start state.
   */
  void call_start_cbs(Blackboard::SharedPtr blackboard,
                      const std::string &start_state);

  /**
   * @brief Calls transition callbacks when transitioning between states.
   *
   * @param blackboard A shared pointer to the blackboard.
   * @param from_state The state being transitioned from.
   * @param to_state The state being transitioned to.
   * @param outcome The outcome that triggered the transition.
   */
  void call_transition_cbs(Blackboard::SharedPtr blackboard,
                           const std::string &from_state,
                           const std::string &to_state,
                           const std::string &outcome);

  /**
   * @brief Calls end callbacks with the given blackboard and outcome.
   *
   * @param blackboard A shared pointer to the blackboard.
   * @param outcome The outcome when the state machine ends.
   */
  void call_end_cbs(Blackboard::SharedPtr blackboard,
                    const std::string &outcome);

  /**
   * @brief Compose parent and state blackboard remappings.
   *
   * The state remappings are resolved against the currently active parent
   * remappings. If a state target already points to a remapped parent key,
   * the resulting mapping is flattened to the final parent target.
   *
   * @param parent_remappings Remappings currently active in the parent scope.
   * @param state_remappings Remappings defined by the current state.
   * @return Combined remapping table with state entries resolved through the
   * parent scope.
   */
  static Remappings compose_remappings(const Remappings &parent_remappings,
                                       const Remappings &state_remappings);

  /**
   * @brief Waits until a current state is available while execution is active.
   * @return The current state name, or an empty string if execution is no
   * longer active.
   */
  std::string wait_for_current_state();

  /**
   * @brief Throws if a hard state machine cancel was requested.
   */
  void throw_if_cancel_state_machine_requested();

  /**
   * @brief Executes a single state transition step.
   *
   * Runs the child state specified by @p current_state, applies remappings,
   * and resolves the resulting outcome (either a child state name or a
   * state-machine-level terminal outcome).
   *
   * @param blackboard The shared blackboard.
   * @param current_state The name of the state to execute.
   * @param state_machine_ends Set to true if the outcome is terminal.
   * @return The translated outcome following this transition.
   */
  std::string execute_step(Blackboard::SharedPtr blackboard,
                           const std::string &current_state,
                           bool &state_machine_ends);
};

} // namespace yasmin

#endif // YASMIN__STATE_MACHINE_HPP_
