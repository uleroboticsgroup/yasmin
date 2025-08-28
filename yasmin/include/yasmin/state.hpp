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

#ifndef YASMIN__STATE_HPP
#define YASMIN__STATE_HPP

#include <atomic>
#include <memory>
#include <set>
#include <string>
#include <vector>

#ifdef __GNUG__     // If using GCC/G++
#include <cxxabi.h> // For abi::__cxa_demangle
#endif

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/logs.hpp"

namespace yasmin {

/**
 * @enum StateStatus
 * @brief Enumeration representing the current status of a state.
 */
enum class StateStatus {
  IDLE,     ///< State is idle and ready to execute
  RUNNING,  ///< State is currently executing
  CANCELED, ///< State execution has been canceled
  COMPLETED ///< State execution has completed successfully
};

/**
 * @class State
 * @brief Represents a state in a state machine.
 *
 * The State class defines a state that can execute actions and manage
 * outcomes. It maintains information about its execution status
 * and the possible outcomes of its execution.
 */
class State {

protected:
  /// The possible outcomes of this state.
  std::set<std::string> outcomes;

private:
  /// Current status of the state
  std::atomic<StateStatus> status{StateStatus::IDLE};

public:
  /**
   * @brief Constructs a State with a set of possible outcomes.
   * @param outcomes A set of possible outcomes for this state.
   */
  State(std::set<std::string> outcomes);

  /**
   * @brief Gets the current status of the state.
   * @return The current StateStatus.
   */
  StateStatus get_status() const;

  /**
   * @brief Checks if the state is idle.
   * @return True if the state is idle, otherwise false.
   */
  bool is_idle() const;

  /**
   * @brief Checks if the state is currently running.
   * @return True if the state is running, otherwise false.
   */
  bool is_running() const;

  /**
   * @brief Checks if the state has been canceled.
   * @return True if the state is canceled, otherwise false.
   */
  bool is_canceled() const;

  /**
   * @brief Checks if the state has completed execution.
   * @return True if the state is completed, otherwise false.
   */
  bool is_completed() const;

  /**
   * @brief Executes the state and returns the outcome.
   * @param blackboard A shared pointer to the Blackboard to use during
   * execution.
   * @return A string representing the outcome of the execution.
   *
   * This function stores the state as running, invokes the execute method,
   * and checks if the returned outcome is valid. If the outcome is not
   * valid, a std::logic_error is thrown.
   * @throws std::logic_error If the outcome is not in the set of outcomes.
   */
  std::string operator()(std::shared_ptr<blackboard::Blackboard> blackboard);

  /**
   * @brief Executes the state's specific logic.
   * @param blackboard A shared pointer to the Blackboard to use during
   * execution.
   * @return A string representing the outcome of the execution.
   *
   * This method is intended to be overridden by derived classes to provide
   * specific execution logic.
   */
  virtual std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) {
    (void)blackboard; // Suppress unused parameter warning
    return "";
  }

  /**
   * @brief Cancels the current state execution.
   *
   * This method sets the status to CANCELED and logs the action.
   */
  virtual void cancel_state() {
    YASMIN_LOG_INFO("Canceling state '%s'", this->to_string().c_str());
    this->status.store(StateStatus::CANCELED);
  }

  /**
   * @brief Gets the set of possible outcomes for this state.
   * @return A constant reference to the set of possible outcomes.
   */
  std::set<std::string> const &get_outcomes();

  /**
   * @brief Converts the state to a string representation.
   * @return A string representation of the state.
   *
   * This method retrieves the demangled name of the class for a readable
   * string representation.
   */
  virtual std::string to_string() {
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
};

} // namespace yasmin

#endif // YASMIN__STATE_HPP
