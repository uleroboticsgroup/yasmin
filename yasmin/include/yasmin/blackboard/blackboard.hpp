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
#include <mutex>
#include <string>

#include "yasmin/blackboard/blackboard_value.hpp"
#include "yasmin/blackboard/blackboard_value_interface.hpp"

namespace yasmin {
namespace blackboard {

class Blackboard {

private:
  std::recursive_mutex mutex;
  std::map<std::string, BlackboardValueInterface *> values;

public:
  Blackboard();
  Blackboard(const Blackboard &other);
  ~Blackboard();

  template <class T> T get(std::string name) {

    std::lock_guard<std::recursive_mutex> lk(this->mutex);

    if (!this->contains(name)) {
      throw "Element " + name + " does not exist in the blackboard";
    }

    BlackboardValue<T> *b_value = (BlackboardValue<T> *)this->values.at(name);
    return b_value->get();
  }

  template <class T> void set(std::string name, T value) {

    std::lock_guard<std::recursive_mutex> lk(this->mutex);

    if (!this->contains(name)) {
      BlackboardValue<T> *b_value = new BlackboardValue<T>(value);
      this->values.insert({name, b_value});

    } else {
      ((BlackboardValue<T> *)this->values.at(name))->set(value);
    }
  }

  void remove(std::string name);
  bool contains(std::string name);
  int size();
  std::string to_string();
};

} // namespace blackboard
} // namespace yasmin

#endif
