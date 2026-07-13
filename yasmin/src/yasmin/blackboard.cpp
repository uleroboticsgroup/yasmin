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
#include <vector>

#include "yasmin/logs.hpp"
#include "yasmin/types.hpp"

using namespace yasmin;

Blackboard::Blackboard() : storage(std::make_shared<SharedStorage>()) {}

Blackboard::Blackboard(const Blackboard &other) : storage(other.storage) {
  std::lock_guard<std::recursive_mutex> lk(other.storage->mutex);
  this->remappings = other.remappings;
}

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

  for (const auto &[stored_key, _] : this->storage->values) {
    const auto visible_it = visible_keys_by_target.find(stored_key);

    if (visible_it == visible_keys_by_target.end()) {
      result.push_back(stored_key);
      continue;
    }

    for (const auto &visible_key : visible_it->second) {
      result.push_back(visible_key);
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
    auto type_it = this->storage->type_registry.find(ele.first);
    std::string type_name = (type_it != this->storage->type_registry.end())
                                ? type_it->second
                                : "unknown";
    result += "\t" + ele.first + " (" + type_name + ")\n";
  }

  return result;
}

const std::string &Blackboard::remap(const std::string &key) const {

  // Check if the key has a remapping
  auto it = this->remappings.find(key);
  if (it != this->remappings.end()) {
    return it->second;
  }

  return key;
}

Blackboard::RawEntry Blackboard::get_raw(const std::string &key) const {
  std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);
  const std::string &remapped_key = this->remap(key);
  auto vit = this->storage->values.find(remapped_key);
  if (vit == this->storage->values.end()) {
    throw std::runtime_error("Element '" + key +
                             "' does not exist in the blackboard");
  }
  auto tit = this->storage->type_registry.find(remapped_key);
  std::string type_name =
      (tit != this->storage->type_registry.end()) ? tit->second : "";
  return {vit->second, type_name};
}

std::vector<Blackboard::RawEntry>
Blackboard::get_raw_batch(const std::vector<std::string> &keys) const {
  std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);
  std::vector<RawEntry> result;
  result.reserve(keys.size());
  for (const auto &key : keys) {
    const std::string &remapped_key = this->remap(key);
    auto vit = this->storage->values.find(remapped_key);
    if (vit == this->storage->values.end()) {
      throw std::runtime_error("Element '" + key +
                               "' does not exist in the blackboard");
    }
    auto tit = this->storage->type_registry.find(remapped_key);
    std::string type_name =
        (tit != this->storage->type_registry.end()) ? tit->second : "";
    result.push_back({vit->second, type_name});
  }
  return result;
}

void Blackboard::set_remappings(const Remappings &remappings) {
  std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);
  this->remappings = remappings;
}

Remappings Blackboard::get_remappings() const {
  std::lock_guard<std::recursive_mutex> lk(this->storage->mutex);
  return this->remappings;
}
