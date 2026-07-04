// Copyright (C) 2023 Miguel Ángel González Santamarta
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

#include "yasmin/blackboard.hpp"

#include <algorithm>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "yasmin/logs.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

Blackboard::Blackboard() : storage(std::make_shared<SharedStorage>()) {}

Blackboard::Blackboard(const Blackboard &other)
    : storage(other.storage), remappings(other.remappings) {}

void Blackboard::remove(const std::string &key) {
  YASMIN_LOG_DEBUG("Removing '%s' from the blackboard", key.c_str());

  std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);
  auto remapped_key = this->remap(key);
  this->storage->values.erase(remapped_key);
  this->storage->type_registry.erase(remapped_key);
}

bool Blackboard::contains(const std::string &key) const {
  YASMIN_LOG_DEBUG("Checking if '%s' is in the blackboard", key.c_str());

  std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);
  auto remapped_key = this->remap(key);
  return (this->storage->values.find(remapped_key) !=
          this->storage->values.end());
}

void Blackboard::copy_value_from(const Blackboard &other,
                                 const std::string &source_key,
                                 const std::string &target_key) {
  YASMIN_LOG_DEBUG("Copying '%s' from blackboard into '%s'", source_key.c_str(),
                   target_key.c_str());

  auto copy_impl = [&]() {
    const std::string &remapped_source_key = other.remap(source_key);
    if (other.storage->values.find(remapped_source_key) ==
        other.storage->values.end()) {
      throw std::runtime_error("Element '" + source_key +
                               "' does not exist in the blackboard");
    }

    const std::string &remapped_target_key = this->remap(target_key);
    this->storage->values[remapped_target_key] =
        other.storage->values.at(remapped_source_key);
    this->storage->type_registry[remapped_target_key] =
        other.storage->type_registry.at(remapped_source_key);
  };

  if (this->storage == other.storage) {
    std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);
    copy_impl();
  } else {
    std::scoped_lock<std::recursive_mutex, std::recursive_mutex> lk(
        this->storage->mutex, other.storage->mutex);
    copy_impl();
  }
}

int Blackboard::size() const {
  std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);
  return this->storage->values.size();
}

std::vector<std::string> Blackboard::keys() const {
  std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);

  std::unordered_map<std::string, std::vector<std::string>>
      visible_keys_by_target;
  visible_keys_by_target.reserve(this->remappings.size());

  for (const auto &[visible_key, target_key] : this->remappings) {
    if (this->storage->values.find(target_key) != this->storage->values.end()) {
      visible_keys_by_target[target_key].push_back(visible_key);
    }
  }

  std::vector<std::string> result;
  result.reserve(this->storage->values.size() + this->remappings.size());

  std::unordered_set<std::string> inserted_keys;
  inserted_keys.reserve(this->storage->values.size() + this->remappings.size());

  for (const auto &[stored_key, _] : this->storage->values) {
    const auto visible_it = visible_keys_by_target.find(stored_key);

    if (visible_it == visible_keys_by_target.end()) {
      if (inserted_keys.insert(stored_key).second) {
        result.push_back(stored_key);
      }
      continue;
    }

    for (const auto &visible_key : visible_it->second) {
      if (inserted_keys.insert(visible_key).second) {
        result.push_back(visible_key);
      }
    }
  }

  std::sort(result.begin(), result.end());
  return result;
}

std::string Blackboard::get_type(const std::string &key) const {
  YASMIN_LOG_DEBUG("Getting type of '%s' from the blackboard", key.c_str());

  std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);
  auto remapped_key = this->remap(key);

  auto it = this->storage->type_registry.find(remapped_key);
  if (it == this->storage->type_registry.end()) {
    throw std::runtime_error("Element '" + key +
                             "' does not exist in the blackboard");
  }

  return it->second;
}

std::string Blackboard::to_string() const {
  std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);

  std::string result = "Blackboard\n";

  // Iterate through all key-value pairs and append to the result string
  for (const auto &ele : this->storage->values) {
    result += "\t" + ele.first + " (" +
              this->storage->type_registry.at(ele.first) + ")\n";
  }

  return result;
}

const std::string &Blackboard::remap(const std::string &key) const {

  // Check if the key has a remapping
  if (this->remappings.find(key) != this->remappings.end()) {
    return this->remappings.at(key);
  }

  return key;
}

void Blackboard::set_remappings(const Remappings &remappings) {
  this->remappings = remappings;
}

const Remappings &Blackboard::get_remappings() const noexcept {
  return this->remappings;
}
