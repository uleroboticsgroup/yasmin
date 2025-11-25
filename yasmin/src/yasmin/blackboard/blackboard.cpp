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
  this->type_registry = other.type_registry;
}

void Blackboard::remove(const std::string &key) {
  YASMIN_LOG_DEBUG("Removing '%s' from the blackboard", key.c_str());

  std::lock_guard<std::recursive_mutex> lk(this->mutex);
  auto remapped_key = this->remap(key);
  this->values.erase(remapped_key);
  this->type_registry.erase(remapped_key);
}

bool Blackboard::contains(const std::string &key) {
  YASMIN_LOG_DEBUG("Checking if '%s' is in the blackboard", key.c_str());

  std::lock_guard<std::recursive_mutex> lk(this->mutex);
  auto remapped_key = this->remap(key);
  return (this->values.find(remapped_key) != this->values.end());
}

int Blackboard::size() {
  std::lock_guard<std::recursive_mutex> lk(this->mutex);
  return this->values.size();
}

std::string Blackboard::get_type(const std::string &key) {
  YASMIN_LOG_DEBUG("Getting type of '%s' from the blackboard", key.c_str());

  std::lock_guard<std::recursive_mutex> lk(this->mutex);
  auto remapped_key = this->remap(key);

  // Check if the key exists
  if (!this->contains(key)) {
    throw std::runtime_error("Element '" + key +
                             "' does not exist in the blackboard");
  }

  return this->type_registry.at(remapped_key);
}

std::string Blackboard::to_string() {
  std::lock_guard<std::recursive_mutex> lk(this->mutex);

  std::string result = "Blackboard\n";

  // Iterate through all key-value pairs and append to the result string
  for (const auto &ele : this->values) {
    result +=
        "\t" + ele.first + " (" + this->type_registry.at(ele.first) + ")\n";
  }

  return result;
}

const std::string &Blackboard::remap(const std::string &key) {

  // Check if the key has a remapping
  if (this->remappings.find(key) != this->remappings.end()) {
    return this->remappings.at(key);
  }

  return key;
}

void Blackboard::set_remappings(
    const std::map<std::string, std::string> &remappings) {
  this->remappings = remappings;
}

const std::map<std::string, std::string> &Blackboard::get_remappings() {
  return this->remappings;
}
