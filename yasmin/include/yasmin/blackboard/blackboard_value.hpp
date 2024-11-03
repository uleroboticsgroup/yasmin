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

#ifndef YASMIN_BLACKBOARD_VAL_HPP
#define YASMIN_BLACKBOARD_VAL_HPP

#include <string>
#include <typeinfo>

#ifdef __GNUG__     // If using GCC/G++
#include <cxxabi.h> // For abi::__cxa_demangle
#endif

#include "yasmin/blackboard/blackboard_value_interface.hpp"

namespace yasmin {
namespace blackboard {

/**
 * @class BlackboardValue
 * @brief A template class that wraps a value of type T.
 *
 * The BlackboardValue class is a template that stores a value of
 * any type T. It provides methods to get and set the value, as well as
 * to retrieve the type information of the value in a human-readable format.
 *
 * @tparam T The type of the value to be stored.
 */
template <class T> class BlackboardValue : public BlackboardValueInterface {
private:
  T value; ///< The stored value of type T.

public:
  /**
   * @brief Constructs a BlackboardValue with the specified value.
   * @param value The initial value to store.
   */
  BlackboardValue(T value) : value(value) {}

  /**
   * @brief Retrieve the stored value.
   * @return The stored value of type T.
   */
  T get() { return this->value; }

  /**
   * @brief Set a new value.
   * @param value The new value to store.
   */
  void set(T value) { this->value = value; }

  /**
   * @brief Get the type of the stored value as a string.
   * @return A string representation of the type of the stored value.
   *
   * This method uses RTTI to get the mangled name of the type
   * and demangles it for readability (if using GCC).
   */
  std::string get_type() {
    std::string name = typeid(T).name(); // Get the mangled name of the type

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

    return name; // Return the demangled type name
  }

  /**
   * @brief Convert the stored value's type information to a string.
   * @return A string representation of the type of the stored value.
   *
   * This method overrides the to_string method from the
   * BlackboardValueInterface to provide the type of the value.
   */
  std::string to_string() { return this->get_type(); }
};

} // namespace blackboard
} // namespace yasmin

#endif // YASMIN_BLACKBOARD_VAL_HPP
