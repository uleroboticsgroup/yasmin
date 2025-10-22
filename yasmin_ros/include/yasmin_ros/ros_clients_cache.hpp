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

#ifndef YASMIN_ROS__ROS_CLIENTS_CACHE_HPP
#define YASMIN_ROS__ROS_CLIENTS_CACHE_HPP

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <tuple>
#include <typeindex>
#include <typeinfo>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "yasmin/logs.hpp"

namespace yasmin_ros {

/**
 * @brief Centralized cache for managing ROS 2 client objects.
 *
 * This class provides a thread-safe cache for storing and reusing ROS 2
 * client objects such as publishers, service clients, and
 * action clients. This helps reduce resource usage and improves performance by
 * avoiding duplicate creation of client objects.
 *
 * The cache is organized by client type and uses unique keys based on:
 * - Node name
 * - Message/Service/Action type
 * - Topic/Service/Action name
 * - Callback group name
 * - QoS profile
 *
 * All methods are thread-safe using internal locking mechanisms.
 */
class ROSClientsCache {
public:
  /**
   * @brief Get an existing action client from the cache or create a new one.
   *
   * @tparam ActionT The type of the action.
   * @param node The ROS 2 node to use.
   * @param action_name The name of the action.
   * @param callback_group The callback group for the action client (optional).
   * @return A shared pointer to the cached or newly created action client.
   */
  template <typename ActionT>
  static typename rclcpp_action::Client<ActionT>::SharedPtr
  get_or_create_action_client(
      rclcpp::Node::SharedPtr node, const std::string &action_name,
      rclcpp::CallbackGroup::SharedPtr callback_group = nullptr) {

    // Create a unique key
    std::string node_name = node->get_name();
    std::string action_type_name = get_type_name<ActionT>();
    std::string callback_group_name = get_callback_group_name(callback_group);
    auto cache_key =
        std::make_tuple(node_name, action_type_name, action_name,
                        callback_group_name, std::type_index(typeid(ActionT)));

    std::lock_guard<std::recursive_mutex> lock(get_lock());

    // Check if action client already exists in cache
    auto &action_clients = get_action_clients();
    auto it = action_clients.find(cache_key);

    if (it != action_clients.end()) {
      YASMIN_LOG_INFO("Reusing existing action client for '%s' of type '%s'",
                      action_name.c_str(), action_type_name.c_str());
      return std::static_pointer_cast<rclcpp_action::Client<ActionT>>(
          it->second);
    }

    // Create new action client if not in cache
    YASMIN_LOG_INFO("Creating new action client for '%s' of type '%s'",
                    action_name.c_str(), action_type_name.c_str());

    auto action_client = rclcpp_action::create_client<ActionT>(
        node, action_name, callback_group);

    // Store in cache (as void pointer for type erasure)
    action_clients[cache_key] = std::static_pointer_cast<void>(action_client);

    return action_client;
  }

  /**
   * @brief Get an existing service client from the cache or create a new one.
   *
   * @tparam ServiceT The type of the service.
   * @param node The ROS 2 node to use.
   * @param service_name The name of the service.
   * @param callback_group The callback group for the service client (optional).
   * @return A shared pointer to the cached or newly created service client.
   */
  template <typename ServiceT>
  static typename rclcpp::Client<ServiceT>::SharedPtr
  get_or_create_service_client(
      rclcpp::Node::SharedPtr node, const std::string &service_name,
      rclcpp::CallbackGroup::SharedPtr callback_group = nullptr) {

    // Create a unique key
    std::string node_name = node->get_name();
    std::string service_type_name = get_type_name<ServiceT>();
    std::string callback_group_name = get_callback_group_name(callback_group);
    auto cache_key =
        std::make_tuple(node_name, service_type_name, service_name,
                        callback_group_name, std::type_index(typeid(ServiceT)));

    std::lock_guard<std::recursive_mutex> lock(get_lock());

    // Check if service client already exists in cache
    auto &service_clients = get_service_clients();
    auto it = service_clients.find(cache_key);

    if (it != service_clients.end()) {
      YASMIN_LOG_INFO("Reusing existing service client for '%s' of type '%s'",
                      service_name.c_str(), service_type_name.c_str());
      return std::static_pointer_cast<rclcpp::Client<ServiceT>>(it->second);
    }

    // Create new service client if not in cache
    YASMIN_LOG_INFO("Creating new service client for '%s' of type '%s'",
                    service_name.c_str(), service_type_name.c_str());

#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(28, 1, 9)
    auto qos = rclcpp::QoS(
        rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_services_default));
#else
    auto qos = rmw_qos_profile_services_default;
#endif
#else
    auto qos = rmw_qos_profile_services_default;
#endif

    auto service_client =
        node->create_client<ServiceT>(service_name, qos, callback_group);

    // Store in cache (as void pointer for type erasure)
    service_clients[cache_key] = std::static_pointer_cast<void>(service_client);

    return service_client;
  }

  /**
   * @brief Get an existing publisher from the cache or create a new one.
   *
   * @tparam MsgT The type of the message.
   * @param node The ROS 2 node to use.
   * @param topic_name The name of the topic.
   * @param qos_profile The QoS profile for the publisher.
   * @param callback_group The callback group for the publisher (optional).
   * @return A shared pointer to the cached or newly created publisher.
   */
  template <typename MsgT>
  static typename rclcpp::Publisher<MsgT>::SharedPtr get_or_create_publisher(
      rclcpp::Node::SharedPtr node, const std::string &topic_name,
      const rclcpp::QoS &qos_profile = rclcpp::QoS(10),
      rclcpp::CallbackGroup::SharedPtr callback_group = nullptr) {

    // Create a unique key
    std::string node_name = node->get_name();
    std::string msg_type_name = get_type_name<MsgT>();
    std::string qos_hash = hash_qos_profile(qos_profile);
    auto cache_key = std::make_tuple(node_name, msg_type_name, topic_name,
                                     qos_hash, std::type_index(typeid(MsgT)));

    std::lock_guard<std::recursive_mutex> lock(get_lock());

    // Check if publisher already exists in cache
    auto &publishers = get_publishers();
    auto it = publishers.find(cache_key);

    if (it != publishers.end()) {
      YASMIN_LOG_INFO("Reusing existing publisher for topic '%s' of type '%s'",
                      topic_name.c_str(), msg_type_name.c_str());
      return std::static_pointer_cast<rclcpp::Publisher<MsgT>>(it->second);
    }

    // Create new publisher if not in cache
    YASMIN_LOG_INFO("Creating new publisher for topic '%s' of type '%s'",
                    topic_name.c_str(), msg_type_name.c_str());

    rclcpp::PublisherOptions options;
    options.callback_group = callback_group;

    auto publisher =
        node->create_publisher<MsgT>(topic_name, qos_profile, options);

    // Store in cache (as void pointer for type erasure)
    publishers[cache_key] = std::static_pointer_cast<void>(publisher);

    return publisher;
  }

  /**
   * @brief Clear the action clients cache.
   */
  static void clear_action_clients();

  /**
   * @brief Clear the service clients cache.
   */
  static void clear_service_clients();

  /**
   * @brief Clear the publishers cache.
   */
  static void clear_publishers();

  /**
   * @brief Clear all clients caches.
   */
  static void clear_all();

  /**
   * @brief Get the number of cached action clients.
   *
   * @return The number of cached action clients.
   */
  static size_t get_action_clients_count();

  /**
   * @brief Get the number of cached service clients.
   *
   * @return The number of cached service clients.
   */
  static size_t get_service_clients_count();

  /**
   * @brief Get the number of cached publishers.
   *
   * @return The number of cached publishers.
   */
  static size_t get_publishers_count();

  /**
   * @brief Get statistics about all caches.
   *
   * @return A map with cache statistics.
   */
  static std::map<std::string, size_t> get_cache_stats();

private:
  // Type alias for cache keys
  using ActionClientKey = std::tuple<std::string, std::string, std::string,
                                     std::string, std::type_index>;
  using ServiceClientKey = std::tuple<std::string, std::string, std::string,
                                      std::string, std::type_index>;
  using PublisherKey = std::tuple<std::string, std::string, std::string,
                                  std::string, std::type_index>;

  // Static cache maps
  static std::map<ActionClientKey, std::shared_ptr<void>> &get_action_clients();
  static std::map<ServiceClientKey, std::shared_ptr<void>> &
  get_service_clients();
  static std::map<PublisherKey, std::shared_ptr<void>> &get_publishers();

  // Static lock for thread-safe access
  static std::recursive_mutex &get_lock();

  /**
   * @brief Get a string representation of a type.
   *
   * @tparam T The type to get the name from.
   * @return A string representation of the type.
   */
  template <typename T> static std::string get_type_name() {
    return typeid(T).name();
  }

  /**
   * @brief Get a string representation of a callback group.
   *
   * @param callback_group The callback group to get the name from.
   * @return A string representation of the callback group.
   */
  static std::string
  get_callback_group_name(rclcpp::CallbackGroup::SharedPtr callback_group);

  /**
   * @brief Get a string representation of a callback function.
   *
   * @tparam CallbackT The type of the callback function.
   * @param callback The callback function to get the name from.
   * @return A string representation of the callback.
   */
  template <typename CallbackT>
  static std::string get_callback_name(const CallbackT &callback) {
    // Use the address of the callback for uniqueness
    std::ostringstream oss;
    oss << typeid(CallbackT).name() << "_"
        << reinterpret_cast<const void *>(&callback);
    return oss.str();
  }

  /**
   * @brief Create a hash for a QoS profile.
   *
   * @param qos_profile The QoS profile to hash.
   * @return A string hash value for the QoS profile.
   */
  static std::string hash_qos_profile(const rclcpp::QoS &qos_profile);
};

} // namespace yasmin_ros

#endif // YASMIN_ROS__ROS_CLIENTS_CACHE_HPP
