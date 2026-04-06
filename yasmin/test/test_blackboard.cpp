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
