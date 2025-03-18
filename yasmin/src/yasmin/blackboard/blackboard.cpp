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
  // Deallocate memory for each value stored in the blackboard
  for (const auto &ele : this->values) {
    delete this->values.at(ele.first);
  }
}

void Blackboard::remove(const std::string &key) {
  YASMIN_LOG_DEBUG("Removing '%s' from the blackboard", key.c_str());

  std::lock_guard<std::recursive_mutex> lk(this->mutex);
  auto remapped_key = this->remap(key);
  delete this->values.at(remapped_key); // Free memory of the value
  this->values.erase(remapped_key);     // Remove the entry from the map
}

bool Blackboard::contains(const std::string &key) {
  YASMIN_LOG_DEBUG("Checking if '%s' is in the blackboard", key.c_str());

  std::lock_guard<std::recursive_mutex> lk(this->mutex);
  auto remapped_key = this->remap(key);
  return (this->values.find(remapped_key) !=
          this->values.end()); // Check if key exists
}

int Blackboard::size() {
  std::lock_guard<std::recursive_mutex> lk(this->mutex);
  return this->values.size(); // Return the number of key-value pairs
}

std::string Blackboard::to_string() {
  std::lock_guard<std::recursive_mutex> lk(this->mutex);

  std::string result = "Blackboard\n";

  // Iterate through each value and append its string representation
  for (const auto &ele : this->values) {
    result += "\t" + ele.first + " (" + ele.second->to_string() + ")\n";
  }

  return result; // Return the complete string representation
}

const std::string &Blackboard::remap(const std::string &key) {
  if (this->remapping.find(key) != this->remapping.end()) {
    return this->remapping.at(key);
  }

  return key;
}

void Blackboard::set_remapping(
    const std::map<std::string, std::string> &remapping) {
  this->remapping = remapping;
}

const std::map<std::string, std::string> &Blackboard::get_remapping() {
  return this->remapping;
}
