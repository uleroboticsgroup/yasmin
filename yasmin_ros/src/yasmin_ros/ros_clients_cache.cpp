// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#include "yasmin_ros/ros_clients_cache.hpp"
#include <sstream>

namespace yasmin_ros {

// Static member function definitions for cache access
std::map<ROSClientsCache::ActionClientKey, std::shared_ptr<void>> &
ROSClientsCache::get_action_clients() {
  static std::map<ActionClientKey, std::shared_ptr<void>> action_clients;
  return action_clients;
}

std::map<ROSClientsCache::ServiceClientKey, std::shared_ptr<void>> &
ROSClientsCache::get_service_clients() {
  static std::map<ServiceClientKey, std::shared_ptr<void>> service_clients;
  return service_clients;
}

std::map<ROSClientsCache::PublisherKey, std::shared_ptr<void>> &
ROSClientsCache::get_publishers() {
  static std::map<PublisherKey, std::shared_ptr<void>> publishers;
  return publishers;
}

std::recursive_mutex &ROSClientsCache::get_lock() {
  static std::recursive_mutex lock;
  return lock;
}

void ROSClientsCache::clear_action_clients() {
  std::lock_guard<std::recursive_mutex> lock(get_lock());
  get_action_clients().clear();
  YASMIN_LOG_INFO("Action clients cache cleared");
}

void ROSClientsCache::clear_service_clients() {
  std::lock_guard<std::recursive_mutex> lock(get_lock());
  get_service_clients().clear();
  YASMIN_LOG_INFO("Service clients cache cleared");
}

void ROSClientsCache::clear_publishers() {
  std::lock_guard<std::recursive_mutex> lock(get_lock());
  get_publishers().clear();
  YASMIN_LOG_INFO("Publishers cache cleared");
}

void ROSClientsCache::clear_all() {
  std::lock_guard<std::recursive_mutex> lock(get_lock());
  clear_action_clients();
  clear_service_clients();
  clear_publishers();
  YASMIN_LOG_INFO("All ROS clients caches cleared");
}

size_t ROSClientsCache::get_action_clients_count() {
  std::lock_guard<std::recursive_mutex> lock(get_lock());
  return get_action_clients().size();
}

size_t ROSClientsCache::get_service_clients_count() {
  std::lock_guard<std::recursive_mutex> lock(get_lock());
  return get_service_clients().size();
}

size_t ROSClientsCache::get_publishers_count() {
  std::lock_guard<std::recursive_mutex> lock(get_lock());
  return get_publishers().size();
}

std::map<std::string, size_t> ROSClientsCache::get_cache_stats() {
  std::lock_guard<std::recursive_mutex> lock(get_lock());

  size_t action_count = get_action_clients().size();
  size_t service_count = get_service_clients().size();
  size_t publisher_count = get_publishers().size();

  return {{"action_clients", action_count},
          {"service_clients", service_count},
          {"publishers", publisher_count},
          {"total", action_count + service_count + publisher_count}};
}

std::string ROSClientsCache::get_callback_group_name(
    rclcpp::CallbackGroup::SharedPtr callback_group) {
  if (callback_group == nullptr) {
    return "none";
  }

  // Use the address of the callback group for uniqueness
  std::ostringstream oss;
  oss << "CallbackGroup_" << callback_group.get();
  return oss.str();
}

std::string ROSClientsCache::hash_qos_profile(const rclcpp::QoS &qos_profile) {
  std::ostringstream oss;

  auto rmw_qos = qos_profile.get_rmw_qos_profile();

  // Hash based on key QoS attributes
  oss << static_cast<int>(rmw_qos.history) << "_" << rmw_qos.depth << "_"
      << static_cast<int>(rmw_qos.reliability) << "_"
      << static_cast<int>(rmw_qos.durability) << "_" << rmw_qos.deadline.sec
      << "_" << rmw_qos.deadline.nsec << "_" << rmw_qos.lifespan.sec << "_"
      << rmw_qos.lifespan.nsec << "_" << static_cast<int>(rmw_qos.liveliness);

  return oss.str();
}

} // namespace yasmin_ros
