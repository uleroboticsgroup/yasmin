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
#include <memory>
#include <string>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/cb_state.hpp"

using namespace yasmin;

class TestCbState : public ::testing::Test {
protected:
  std::shared_ptr<CbState> state;
  std::shared_ptr<blackboard::Blackboard> blackboard;

  void SetUp() override {
    blackboard = std::make_shared<blackboard::Blackboard>();

    auto execute = [](std::shared_ptr<blackboard::Blackboard> bb) {
      return std::string("outcome1");
    };

    state =
        std::make_shared<CbState>(std::set<std::string>{"outcome1"}, execute);
  }
};

TEST_F(TestCbState, TestCbStateCall) {
  EXPECT_EQ((*state)(blackboard), "outcome1");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
