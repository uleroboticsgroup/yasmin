// Copyright (C) 2023  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef YASMIN_BLACKBOARD_VAL_HPP
#define YASMIN_BLACKBOARD_VAL_HPP

#include <string>
#include <typeinfo>

#ifdef __GNUG__     // if using GCC/G++
#include <cxxabi.h> // for abi::__cxa_demangle
#endif

#include "yasmin/blackboard/blackboard_value_interface.hpp"

namespace yasmin {
namespace blackboard {

template <class T> class BlackboardValue : public BlackboardValueInterface {
private:
  T value;

public:
  BlackboardValue(T value) { this->value = value; }

  T get() { return this->value; }
  void set(T value) { this->value = value; }

  std::string get_type() {
    std::string name = typeid(T).name(); // get the mangled name of the type

#ifdef __GNUG__ // if using GCC/G++
    int status;
    // demangle the name using GCC's demangling function
    char *demangled =
        abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status);
    if (status == 0) {
      name = demangled;
    }
    free(demangled);
#endif

    return name;
  }
  std::string to_string() { return this->get_type(); }
};

} // namespace blackboard
} // namespace yasmin

#endif
