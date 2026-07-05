// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#include <gtest/gtest.h>

#include <string>

#include "yasmin/blackboard.hpp"

using namespace yasmin;

class TestBlackboard : public ::testing::Test {
protected:
  Blackboard blackboard;

  void TearDown() override { blackboard.set_remappings({}); }
};

TEST_F(TestBlackboard, TestGet) {
  blackboard.set<std::string>("foo", "foo");
  EXPECT_EQ(blackboard.get<std::string>("foo"), "foo");
}

TEST_F(TestBlackboard, TestDelete) {
  blackboard.set<std::string>("foo", "foo");
  blackboard.remove("foo");
  EXPECT_FALSE(blackboard.contains("foo"));
}

TEST_F(TestBlackboard, TestContains) {
  blackboard.set<std::string>("foo", "foo");
  EXPECT_TRUE(blackboard.contains("foo"));
}

TEST_F(TestBlackboard, TestLen) {
  blackboard.set<std::string>("foo", "foo");
  EXPECT_EQ(blackboard.size(), 1);
}

TEST_F(TestBlackboard, TestType) {
  blackboard.set<int>("foo", 10);
  EXPECT_EQ(blackboard.get_type("foo"), "int");
}

TEST_F(TestBlackboard, TestRemappings) {
  blackboard.set<std::string>("bar", "foo");
  blackboard.set_remappings({{"foo", "bar"}});
  EXPECT_EQ(blackboard.get<std::string>("bar"), "foo");
}

TEST_F(TestBlackboard, TestKeysWithoutRemappings) {
  blackboard.set<std::string>("foo", "foo");
  blackboard.set<int>("bar", 1);

  const auto keys = blackboard.keys();

  ASSERT_EQ(keys.size(), 2U);
  EXPECT_EQ(keys.at(0), "bar");
  EXPECT_EQ(keys.at(1), "foo");
}

TEST_F(TestBlackboard, TestKeysWithRemappingsExposeVisibleScope) {
  blackboard.set<std::string>("shared", "value");
  blackboard.set<int>("plain", 7);
  blackboard.set_remappings({{"first", "shared"}, {"second", "shared"}});

  const auto keys = blackboard.keys();

  ASSERT_EQ(keys.size(), 3U);
  EXPECT_EQ(keys.at(0), "first");
  EXPECT_EQ(keys.at(1), "plain");
  EXPECT_EQ(keys.at(2), "second");
}

TEST_F(TestBlackboard,
       TestCopiedBlackboardSharesStorageAndKeepsRemappingsLocal) {
  blackboard.set<int>("shared", 1);
  blackboard.set_remappings({{"from_original", "shared"}});

  Blackboard copied_blackboard(blackboard);
  copied_blackboard.set_remappings({{"from_copy", "shared"}});

  EXPECT_TRUE(blackboard.contains("from_original"));
  EXPECT_FALSE(blackboard.contains("from_copy"));
  EXPECT_TRUE(copied_blackboard.contains("from_copy"));
  EXPECT_FALSE(copied_blackboard.contains("from_original"));

  copied_blackboard.set<int>("from_copy", 42);

  EXPECT_EQ(blackboard.get<int>("from_original"), 42);
  EXPECT_EQ(blackboard.get<int>("shared"), 42);
  EXPECT_EQ(copied_blackboard.get<int>("from_copy"), 42);
}

TEST_F(TestBlackboard, TestCopyValueFromWorksAcrossCopiedBlackboards) {
  blackboard.set<std::string>("source", "value");
  Blackboard copied_blackboard(blackboard);
  copied_blackboard.set_remappings({{"copy_target", "target"}});

  copied_blackboard.copy_value_from(blackboard, "source", "copy_target");

  EXPECT_EQ(blackboard.get<std::string>("target"), "value");
  EXPECT_EQ(copied_blackboard.get<std::string>("copy_target"), "value");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
