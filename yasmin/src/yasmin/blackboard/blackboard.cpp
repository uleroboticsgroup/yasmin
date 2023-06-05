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

#include <map>
#include <string>

#include "yasmin/blackboard/blackboard.hpp"

using namespace yasmin::blackboard;

Blackboard::Blackboard() {}

Blackboard::Blackboard(const Blackboard &other) {
  for (const auto &ele : other.values) {
    this->values.insert({ele.first, ele.second});
  }
}

Blackboard::~Blackboard() {
  for (const auto &ele : this->values) {
    delete this->values.at(ele.first);
  }
}

bool Blackboard::contains(std::string name) {
  return (this->values.find(name) != this->values.end());
}

std::string Blackboard::to_string() {

  std::string result = "Blackboard\n";

  for (const auto &ele : this->values) {
    result += "\t" + ele.first + " (" + ele.second->to_string() + ")\n";
  }

  return result;
}
