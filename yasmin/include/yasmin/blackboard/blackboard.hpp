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

#ifndef YASMIN_BLACKBOARD_HPP
#define YASMIN_BLACKBOARD_HPP

#include <map>
#include <string>

#include "yasmin/blackboard/blackboard_value.hpp"
#include "yasmin/blackboard/blackboard_value_interface.hpp"

namespace yasmin {
namespace blackboard {

class Blackboard {

private:
  std::map<std::string, BlackboardValueInterface *> values;

public:
  Blackboard();
  Blackboard(const Blackboard &other);
  ~Blackboard();

  template <class T> void set(std::string name, T value) {
    if (!this->contains(name)) {
      BlackboardValue<T> *b_value = new BlackboardValue<T>(value);
      this->values.insert({name, b_value});

    } else {
      ((BlackboardValue<T> *)this->values.at(name))->set(value);
    }
  }

  template <class T> T get(std::string name) {
    if (!this->contains(name)) {
      throw "Element " + name + " does not exist in the blackboard";
    }

    BlackboardValue<T> *b_value = (BlackboardValue<T> *)this->values.at(name);
    return b_value->get();
  }

  bool contains(std::string name);

  std::string to_string();
};

} // namespace blackboard
} // namespace yasmin

#endif
