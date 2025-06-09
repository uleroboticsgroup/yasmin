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

#ifndef YASMIN__CB_STATE_HPP
#define YASMIN__CB_STATE_HPP

#include <functional>
#include <memory>
#include <set>
#include <string>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin {

/**
 * @class CbState
 * @brief Represents a state that executes a callback function.
 *
 * The CbState class inherits from the State class and is designed to
 * execute a user-defined callback function, utilizing a shared pointer
 * to a Blackboard object to obtain necessary data.
 */
class CbState : public State {

private:
  /// Pointer to the callback function to be executed.
  std::function<std::string(std::shared_ptr<blackboard::Blackboard>)> callback;

public:
  /**
   * @brief Constructs a CbState object.
   *
   * @param outcomes A set of possible outcomes for this state.
   * @param callback A function pointer to the callback function that
   *                 will be executed when this state is activated.
   *
   * @throw std::invalid_argument If the outcomes set is empty.
   */
  CbState(std::set<std::string> outcomes,
          std::function<std::string(std::shared_ptr<blackboard::Blackboard>)>
              callback);

  /**
   * @brief Executes the callback function.
   *
   * This function is called to execute the callback and retrieve
   * the result. It may use the provided Blackboard for additional data.
   *
   * @param blackboard A shared pointer to the Blackboard object
   *                   used during execution.
   *
   * @return The result of the callback function execution as a string.
   *
   * @throw std::runtime_error If the callback execution fails.
   */
  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override;
};

} // namespace yasmin

#endif // YASMIN__CB_STATE_HPP
