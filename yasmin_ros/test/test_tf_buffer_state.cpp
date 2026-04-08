// Copyright (C) 2026 Maik Knof
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

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#if __has_include(<tf2/time.hpp>)
#include <tf2/time.hpp>
#else
#include <tf2/time.h>
#endif

#if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/ros_clients_cache.hpp"
#include "yasmin_ros/tf_buffer_state.hpp"
#include "yasmin_ros/yasmin_node.hpp"

using namespace std::chrono_literals;
using namespace yasmin_ros;
using namespace yasmin_ros::basic_outcomes;

namespace {

class LookupTfState : public yasmin::State {
public:
  LookupTfState() : yasmin::State({SUCCEED, ABORT}) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    auto buffer =
        blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
    const auto deadline = std::chrono::steady_clock::now() + 2s;

    while (std::chrono::steady_clock::now() < deadline) {
      try {
        const auto transform =
            buffer->lookupTransform("map", "base_link", tf2::TimePointZero);
        if (std::abs(transform.transform.translation.x - 1.23) < 1e-6) {
          return SUCCEED;
        }
      } catch (const std::exception &) {
      }
      std::this_thread::sleep_for(50ms);
    }

    return ABORT;
  }
};

void publish_tf_loop(const rclcpp::Node::SharedPtr &node,
                     std::atomic<bool> &running) {
  tf2_ros::TransformBroadcaster broadcaster(node);

  while (running.load()) {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node->now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "base_link";
    transform.transform.translation.x = 1.23;
    transform.transform.translation.y = 0.0;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;
    broadcaster.sendTransform(transform);
    std::this_thread::sleep_for(50ms);
  }
}

} // namespace

class TestTfBufferState : public ::testing::Test {
protected:
  static void SetUpTestCase() { rclcpp::init(0, nullptr); }

  static void TearDownTestCase() {
    yasmin_ros::ROSClientsCache::clear_all();
    yasmin_ros::YasminNode::destroy_instance();
    rclcpp::shutdown();
  }
};

TEST_F(TestTfBufferState, TestCreatesBufferAndListenerInBlackboard) {
  auto blackboard = yasmin::Blackboard::make_shared();
  auto create_buffer_state = std::make_shared<TfBufferState>();
  create_buffer_state->configure();

  EXPECT_EQ((*create_buffer_state)(blackboard), std::string(SUCCEED));
  EXPECT_TRUE(blackboard->contains("tf_buffer"));
  EXPECT_TRUE(blackboard->contains("tf_listener"));
}

TEST_F(TestTfBufferState, TestLookupFromFollowingState) {
  auto broadcaster_node = std::make_shared<rclcpp::Node>("tf_buffer_cpp_test");
  std::atomic<bool> running{true};
  std::thread publisher_thread(publish_tf_loop, broadcaster_node,
                               std::ref(running));

  auto blackboard = yasmin::Blackboard::make_shared();
  auto create_buffer_state = std::make_shared<TfBufferState>();
  create_buffer_state->configure();
  auto lookup_state = std::make_shared<LookupTfState>();

  EXPECT_EQ((*create_buffer_state)(blackboard), std::string(SUCCEED));
  EXPECT_EQ((*lookup_state)(blackboard), std::string(SUCCEED));

  running.store(false);
  publisher_thread.join();
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
