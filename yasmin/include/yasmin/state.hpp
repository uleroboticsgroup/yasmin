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

#ifndef YASMIN__STATE_HPP_
#define YASMIN__STATE_HPP_

#include <atomic>
#include <functional>
#include <string>
#include <unordered_map>

#include "yasmin/blackboard.hpp"
#include "yasmin/blackboard_key_info.hpp"
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
  Outcomes outcomes;

private:
  /**
   * @brief Gets a mutable reference to this state's metadata.
   *
   * Metadata is stored externally (not as a direct member) to preserve
   * ABI compatibility with plugins compiled against earlier versions of
   * the State class.
   */
  StateMetadata &get_metadata_ref() const;
  /// Current status of the state
  std::atomic<StateStatus> status{StateStatus::IDLE};

  /**
   * @brief Sets the current status of the state.
   * @param new_status The new status to set.
   */
  void set_status(StateStatus new_status);

  /**
   * @brief Gets the current status of the state.
   * @return The current StateStatus.
   */
  StateStatus get_status() const;

public:
  /**
   * @brief Shared pointer type for State.
   */
  YASMIN_PTR_ALIASES(State)

  /**
   * @brief Constructs a State with a set of possible outcomes.
   * @param outcomes A set of possible outcomes for this state.
   */
  State(const Outcomes &outcomes);

  /**
   * @brief Virtual destructor.
   *
   * Cleans up externally-stored metadata for this instance.
   */
  virtual ~State();

  /**
   * @brief Checks if the state is idle.
   * @return True if the state is idle, otherwise false.
   */
  bool is_idle() const noexcept;

  /**
   * @brief Checks if the state is currently running.
   * @return True if the state is running, otherwise false.
   */
  bool is_running() const noexcept;

  /**
   * @brief Checks if the state has been canceled.
   * @return True if the state is canceled, otherwise false.
   */
  bool is_canceled() const noexcept;

  /**
   * @brief Checks if the state has completed execution.
   * @return True if the state is completed, otherwise false.
   */
  bool is_completed() const noexcept;

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
  std::string operator()(Blackboard::SharedPtr blackboard);

  /**
   * @brief Executes the state's specific logic.
   * @param blackboard A shared pointer to the Blackboard to use during
   * execution.
   * @return A string representing the outcome of the execution.
   *
   * This method is intended to be overridden by derived classes to provide
   * specific execution logic.
   */
  virtual std::string execute(Blackboard::SharedPtr blackboard) {
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
    this->set_status(StateStatus::CANCELED);
  }

  /**
   * @brief Gets the set of possible outcomes for this state.
   * @return A constant reference to the set of possible outcomes.
   */
  Outcomes const &get_outcomes() const noexcept;

  /**
   * @brief Sets the description for this state.
   * @param description The description of the state.
   */
  void set_description(const std::string &description);

  /**
   * @brief Gets the description of this state.
   * @return The description of the state.
   */
  const std::string &get_description() const;

  /**
   * @brief Sets a human-readable description for an outcome.
   * @param outcome The outcome name.
   * @param description The description of the outcome.
   * @throws std::invalid_argument If the outcome is not part of this state.
   */
  void set_outcome_description(const std::string &outcome,
                               const std::string &description);

  /**
   * @brief Gets the human-readable description for an outcome.
   * @param outcome The outcome name.
   * @return The description of the outcome, or an empty string if none exists.
   * @throws std::invalid_argument If the outcome is not part of this state.
   */
  const std::string &get_outcome_description(const std::string &outcome) const;

  /**
   * @brief Gets all outcome descriptions.
   * @return A constant reference to the outcome description map.
   */
  const std::unordered_map<std::string, std::string> &
  get_outcome_descriptions() const;

  /**
   * @brief Adds an input key to the state's metadata.
   * @param key_info Information about the input key.
   */
  void add_input_key(const BlackboardKeyInfo &key_info);

  /**
   * @brief Adds an input key with a name only.
   * @param key_name The name of the input key.
   */
  void add_input_key(const std::string &key_name);

  /**
   * @brief Adds an input key with a name and description.
   * @param key_name The name of the input key.
   * @param description Human-readable description of the input key.
   */
  void add_input_key(const std::string &key_name,
                     const std::string &description);

  /**
   * @brief Adds an input key with a name, default value and description.
   * @tparam T The type of the default value.
   * @param key_name The name of the input key.
   * @param description Human-readable description of the input key.
   * @param default_value The default value for the key.
   */
  template <typename T>
  void add_input_key(const std::string &key_name,
                     const std::string &description, T default_value) {
    this->add_input_key(
        BlackboardKeyInfo(key_name, description, default_value));
  }

  /**
   * @brief Adds an output key to the state's metadata.
   * @param key_info Information about the output key.
   */
  void add_output_key(const BlackboardKeyInfo &key_info);

  /**
   * @brief Adds an output key with a name only.
   * @param key_name The name of the output key.
   */
  void add_output_key(const std::string &key_name);

  /**
   * @brief Adds an output key with a name and description.
   * @param key_name The name of the output key.
   * @param description Human-readable description of the output key.
   */
  void add_output_key(const std::string &key_name,
                      const std::string &description);

  /**
   * @brief Gets the input keys metadata.
   * @return A constant reference to the vector of input key information.
   */
  const std::vector<BlackboardKeyInfo> &get_input_keys() const;

  /**
   * @brief Gets the output keys metadata.
   * @return A constant reference to the vector of output key information.
   */
  const std::vector<BlackboardKeyInfo> &get_output_keys() const;

  /**
   * @brief Gets the complete state metadata.
   * @return A constant reference to the state metadata.
   */
  const StateMetadata &get_metadata() const;

  /**
   * @brief Converts the state to a string representation.
   * @return A string representation of the state.
   *
   * This method retrieves the demangled name of the class for a readable
   * string representation.
   */
  virtual std::string to_string() const;
};

} // namespace yasmin

#endif // YASMIN__STATE_HPP_
