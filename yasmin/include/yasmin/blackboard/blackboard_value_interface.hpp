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

#ifndef YASMIN_BLACKBOARD_VAL_IFACE_HPP
#define YASMIN_BLACKBOARD_VAL_IFACE_HPP

#include <string>

namespace yasmin {
namespace blackboard {

/**
 * @class BlackboardValueInterface
 * @brief Interface for blackboard value types.
 *
 * This interface defines the contract for value types that can be
 * stored in a blackboard. It requires implementing a method to
 * represent the value as a string.
 */
class BlackboardValueInterface {
public:
  /** @brief Virtual destructor for the interface. */
  virtual ~BlackboardValueInterface(){};

  /**
   * @brief Convert the value to a string representation.
   * @return A string that represents the value.
   *
   * This method should be implemented by derived classes to provide
   * an appropriate string representation of the value they encapsulate.
   */
  virtual std::string to_string() { return ""; };
};

} // namespace blackboard
} // namespace yasmin

#endif // YASMIN_BLACKBOARD_VAL_IFACE_HPP
