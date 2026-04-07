// Copyright (C) 2026 Maik Knof
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
