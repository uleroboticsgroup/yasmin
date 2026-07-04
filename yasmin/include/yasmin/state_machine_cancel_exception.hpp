// Copyright (C) 2026 Maik Knof
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

#ifndef YASMIN__STATE_MACHINE_CANCEL_EXCEPTION_HPP_
#define YASMIN__STATE_MACHINE_CANCEL_EXCEPTION_HPP_

#include <stdexcept>
#include <string>

namespace yasmin {

/**
 * @class StateMachineCancelException
 * @brief Exception thrown when a state machine is canceled as a whole.
 *
 * This exception is used internally to abort the current execution loop after
 * cancel_state_machine() has been requested. It can also be observed by callers
 * that execute the state machine directly.
 */
class StateMachineCancelException : public std::runtime_error {
public:
  /**
   * @brief Construct a new cancellation exception.
   * @param state_machine_name Human-readable state machine name.
   */
  explicit StateMachineCancelException(const std::string &state_machine_name)
      : std::runtime_error("State machine canceled: " + state_machine_name) {}
};

} // namespace yasmin

#endif // YASMIN__STATE_MACHINE_CANCEL_EXCEPTION_HPP_
