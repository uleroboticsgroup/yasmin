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

#include <memory>
#include <string>

#include "yasmin/blackboard.hpp"
#include "yasmin/cb_state.hpp"

using namespace yasmin;

class TestCbState : public ::testing::Test {
protected:
  CbState::SharedPtr state;
  yasmin::Blackboard::SharedPtr blackboard;

  void SetUp() override {
    blackboard = yasmin::Blackboard::make_shared();

    auto execute = [](yasmin::Blackboard::SharedPtr bb) {
      return std::string("outcome1");
    };

    state = CbState::make_shared(std::set<std::string>{"outcome1"}, execute);
  }
};

TEST_F(TestCbState, TestCbStateCall) {
  EXPECT_EQ((*state)(blackboard), "outcome1");
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
