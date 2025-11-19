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

#include "yasmin/blackboard/blackboard.hpp"

using namespace yasmin::blackboard;

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

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
