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

#ifndef YASMIN__CB_STATE_HPP_
#define YASMIN__CB_STATE_HPP_

#include <functional>
#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

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
  CbStateCallback callback;

public:
  /**
   * @brief Shared pointer type for CbState.
   */
  YASMIN_PTR_ALIASES(CbState)

  /**
   * @brief Constructs a CbState object.
   *
   * @param outcomes A set of possible outcomes for this state.
   * @param callback A function pointer to the callback function that
   *                 will be executed when this state is activated.
   *
   * @throws std::invalid_argument If the outcomes set is empty.
   */
  CbState(const Outcomes &outcomes, CbStateCallback callback);

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
   * @throws std::runtime_error If the callback execution fails.
   */
  std::string execute(Blackboard::SharedPtr blackboard) override;
};

} // namespace yasmin

#endif // YASMIN__CB_STATE_HPP_
