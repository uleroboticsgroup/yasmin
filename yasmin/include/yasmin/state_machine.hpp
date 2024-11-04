// Copyright (C) 2023  Miguel Ángel González Santamarta
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

#ifndef YASMIN_STATE_MACHINE_HPP
#define YASMIN_STATE_MACHINE_HPP

#include <atomic>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"

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

  /// Alias for a callback function executed before running the state machine.
  using StartCallbackType = std::function<void(
      std::shared_ptr<yasmin::blackboard::Blackboard>, const std::string &,
      const std::vector<std::string> &)>;
  /// Alias for a callback function executed before changing the state.
  using TransitionCallbackType = std::function<void(
      std::shared_ptr<yasmin::blackboard::Blackboard>, const std::string &,
      const std::string &, const std::string &,
      const std::vector<std::string> &)>;
  /// Alias for a callback function executed after running the state machine.
  using EndCallbackType = std::function<void(
      std::shared_ptr<yasmin::blackboard::Blackboard>, const std::string &,
      const std::vector<std::string> &)>;

public:
  /**
   * @brief Construct a new StateMachine object.
   *
   * @param outcomes A set of possible outcomes for the state machine.
   */
  StateMachine(std::set<std::string> outcomes);

  /**
   * @brief Adds a state to the state machine with specified transitions.
   *
   * @param name The name of the state.
   * @param state A shared pointer to the State object representing the new
   * state.
   * @param transitions A map of transitions where the key is the outcome
   *                    and the value is the target state name.
   * @throws std::logic_error If the state is already registered.
   * @throws std::invalid_argument If any transition has empty source or target,
   *                               or references unregistered outcomes.
   */
  void add_state(std::string name, std::shared_ptr<State> state,
                 std::map<std::string, std::string> transitions);

  /**
   * @brief Adds a state to the state machine without transitions.
   *
   * @param name The name of the state.
   * @param state A shared pointer to the State object representing the new
   * state.
   */
  void add_state(std::string name, std::shared_ptr<State> state);

  /**
   * @brief Sets the start state for the state machine.
   *
   * @param state_name The name of the state to set as the start state.
   * @throws std::invalid_argument If the state name is empty or not registered.
   */
  void set_start_state(std::string state_name);

  /**
   * @brief Retrieves the name of the start state.
   *
   * @return The name of the start state.
   */
  std::string get_start_state();

  /**
   * @brief Gets a constant reference to the map of states.
   *
   * @return A constant reference to the map of states.
   */
  std::map<std::string, std::shared_ptr<State>> const &get_states();

  /**
   * @brief Gets a constant reference to the map of transitions.
   *
   * @return A constant reference to the map of transitions.
   */
  std::map<std::string, std::map<std::string, std::string>> const &
  get_transitions();

  /**
   * @brief Retrieves the current state name.
   *
   * @return The name of the current state.
   */
  std::string get_current_state();

  /**
   * @brief Adds a callback function to be called when the state machine starts.
   *
   * @param cb The callback function to execute.
   * @param args Optional arguments to pass to the callback.
   */
  void add_start_cb(StartCallbackType cb, std::vector<std::string> args = {});

  /**
   * @brief Adds a callback function for state transitions.
   *
   * @param cb The callback function to execute.
   * @param args Optional arguments to pass to the callback.
   */
  void add_transition_cb(TransitionCallbackType cb,
                         std::vector<std::string> args = {});

  /**
   * @brief Adds a callback function to be called when the state machine ends.
   *
   * @param cb The callback function to execute.
   * @param args Optional arguments to pass to the callback.
   */
  void add_end_cb(EndCallbackType cb, std::vector<std::string> args = {});

  /**
   * @brief Calls start callbacks with the given blackboard and start state.
   *
   * @param blackboard A shared pointer to the blackboard.
   * @param start_state The name of the start state.
   */
  void
  call_start_cbs(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                 const std::string &start_state);

  /**
   * @brief Calls transition callbacks when transitioning between states.
   *
   * @param blackboard A shared pointer to the blackboard.
   * @param from_state The state being transitioned from.
   * @param to_state The state being transitioned to.
   * @param outcome The outcome that triggered the transition.
   */
  void call_transition_cbs(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
      const std::string &from_state, const std::string &to_state,
      const std::string &outcome);

  /**
   * @brief Calls end callbacks with the given blackboard and outcome.
   *
   * @param blackboard A shared pointer to the blackboard.
   * @param outcome The outcome when the state machine ends.
   */
  void call_end_cbs(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                    const std::string &outcome);

  /**
   * @brief Validates the state machine configuration.
   *
   * @param strict Whether the validation is strict, which means checking if all
   * state outcomes are used and all state machine outcomes are reached.
   * @throws std::runtime_error If the state machine is misconfigured.
   */
  void validate(bool strict_mode = false);

  /**
   * @brief Executes the state machine.
   *
   * @param blackboard A shared pointer to the blackboard used during execution.
   * @return The outcome of the state machine execution.
   * @throws std::runtime_error If the execution cannot be completed due to
   *                            invalid states or transitions.
   */
  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override;

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

  using State::operator();

  /**
   * @brief Cancels the current state execution.
   */
  void cancel_state() override;

  /**
   * @brief Converts the state machine to a string representation.
   *
   * @return A string describing the state machine and its states.
   */
  std::string to_string();

private:
  /// Map of states
  std::map<std::string, std::shared_ptr<State>> states;
  /// Map of transitions
  std::map<std::string, std::map<std::string, std::string>> transitions;
  /// Name of the start state
  std::string start_state;
  /// Name of the current state
  std::string current_state;
  /// Mutex for current state access
  std::unique_ptr<std::mutex> current_state_mutex;

  /// Flag to indicate if the state machine has been validated
  std::atomic_bool validated{false};

  /// Start callbacks executed before the state machine
  std::vector<std::pair<StartCallbackType, std::vector<std::string>>> start_cbs;
  /// Transition callbacks executed before changing the state
  std::vector<std::pair<TransitionCallbackType, std::vector<std::string>>>
      transition_cbs;
  /// End callbacks executed before the state machine
  std::vector<std::pair<EndCallbackType, std::vector<std::string>>> end_cbs;
};

} // namespace yasmin

#endif
