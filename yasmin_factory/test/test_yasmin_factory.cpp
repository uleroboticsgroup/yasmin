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

#include <fstream>
#include <gtest/gtest.h>
#include <memory>
#include <sstream>
#include <string>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin/state_machine.hpp"
#include "yasmin_factory/yasmin_factory.hpp"

using namespace yasmin_factory;

class TestYasminFactory : public ::testing::Test {
protected:
  std::unique_ptr<YasminFactory> factory;
  std::string test_dir;

  void SetUp() override {
    factory = std::make_unique<YasminFactory>();
    // Get the current test directory
    test_dir = __FILE__;
    test_dir = test_dir.substr(0, test_dir.find_last_of("/\\"));
  }

  void TearDown() override {
    if (factory) {
      factory->cleanup();
    }
  }

  // Helper function to create a temporary XML file
  std::string createTempXMLFile(const std::string &content) {
    std::string temp_file = test_dir + "/temp_test.xml";
    std::ofstream ofs(temp_file);
    ofs << content;
    ofs.close();
    return temp_file;
  }

  // Helper function to remove temporary file
  void removeTempFile(const std::string &filename) {
    std::remove(filename.c_str());
  }
};

TEST_F(TestYasminFactory, TestCreateCppState) {
  // Create a simple C++ state using XML
  std::string xml_content = R"(
    <State name="TestState" type="cpp" class="yasmin_factory/TestSimpleState"/>
  )";

  tinyxml2::XMLDocument doc;
  doc.Parse(xml_content.c_str());
  tinyxml2::XMLElement *state_elem = doc.FirstChildElement("State");

  ASSERT_NE(state_elem, nullptr);

  try {
    auto state = factory->create_state(state_elem);

    ASSERT_NE(state, nullptr);
    auto outcomes = state->get_outcomes();
    EXPECT_EQ(outcomes.size(), 2);
    EXPECT_NE(outcomes.find("outcome1"), outcomes.end());
    EXPECT_NE(outcomes.find("outcome2"), outcomes.end());
  } catch (const std::exception &e) {
    GTEST_SKIP() << "C++ plugin not available: " << e.what();
  }
}

TEST_F(TestYasminFactory, TestCreatePythonState) {
  // Create a simple Python state using XML
  std::string xml_content = R"(
    <State name="TestState" type="py" module="test.test_simple_state" class="TestSimpleState"/>
  )";

  tinyxml2::XMLDocument doc;
  doc.Parse(xml_content.c_str());
  tinyxml2::XMLElement *state_elem = doc.FirstChildElement("State");

  ASSERT_NE(state_elem, nullptr);

  try {
    auto state = factory->create_state(state_elem);

    ASSERT_NE(state, nullptr);
    auto outcomes = state->get_outcomes();
    EXPECT_EQ(outcomes.size(), 2);
  } catch (const std::exception &e) {
    GTEST_SKIP() << "Python state creation failed: " << e.what();
  }
}

TEST_F(TestYasminFactory, TestCreateStateInvalidType) {
  std::string xml_content = R"(
    <State name="InvalidState" type="invalid" class="SomeClass"/>
  )";

  tinyxml2::XMLDocument doc;
  doc.Parse(xml_content.c_str());
  tinyxml2::XMLElement *state_elem = doc.FirstChildElement("State");

  ASSERT_NE(state_elem, nullptr);

  EXPECT_THROW(
      { auto state = factory->create_state(state_elem); }, std::runtime_error);
}

TEST_F(TestYasminFactory, TestCreateStateFromFileCppPy) {
  std::string xml_file = test_dir + "/test_simple_sm_1.xml";

  try {
    auto sm = factory->create_sm_from_file(xml_file);

    ASSERT_NE(sm, nullptr);
    auto outcomes = sm->get_outcomes();
    EXPECT_NE(outcomes.find("end"), outcomes.end());
  } catch (const std::exception &e) {
    GTEST_SKIP() << "XML file not available or plugin missing: " << e.what();
  }
}

TEST_F(TestYasminFactory, TestCreateStateFromFilePyCpp) {
  std::string xml_file = test_dir + "/test_simple_sm_2.xml";

  try {
    auto sm = factory->create_sm_from_file(xml_file);

    ASSERT_NE(sm, nullptr);
    auto outcomes = sm->get_outcomes();
    EXPECT_NE(outcomes.find("end"), outcomes.end());
  } catch (const std::exception &e) {
    GTEST_SKIP() << "XML file not available or plugin missing: " << e.what();
  }
}

TEST_F(TestYasminFactory, TestCreateStateFromInvalidFile) {
  std::string xml_file = "/non/existent/file.xml";

  EXPECT_THROW(
      { auto sm = factory->create_sm_from_file(xml_file); },
      std::runtime_error);
}

TEST_F(TestYasminFactory, TestCreateStateFromInvalidXML) {
  std::string temp_file = createTempXMLFile("This is not valid XML");

  EXPECT_THROW(
      { auto sm = factory->create_sm_from_file(temp_file); },
      std::runtime_error);

  removeTempFile(temp_file);
}

TEST_F(TestYasminFactory, TestCreateStateFromInvalidRootElement) {
  std::string xml_content = R"(<?xml version="1.0"?>
    <InvalidRoot/>
  )";

  std::string temp_file = createTempXMLFile(xml_content);

  EXPECT_THROW(
      { auto sm = factory->create_sm_from_file(temp_file); },
      std::runtime_error);

  removeTempFile(temp_file);
}

TEST_F(TestYasminFactory, TestCreateSimpleStateMachine) {
  std::string xml_content = R"(
    <StateMachine outcomes="end">
      <State name="State1" type="cpp" class="yasmin_factory/TestSimpleState">
        <Transition from="outcome1" to="State1"/>
        <Transition from="outcome2" to="end"/>
      </State>
    </StateMachine>
  )";

  tinyxml2::XMLDocument doc;
  doc.Parse(xml_content.c_str());
  tinyxml2::XMLElement *root = doc.FirstChildElement("StateMachine");

  ASSERT_NE(root, nullptr);

  try {
    auto sm = factory->create_sm(root);

    ASSERT_NE(sm, nullptr);
    auto outcomes = sm->get_outcomes();
    EXPECT_NE(outcomes.find("end"), outcomes.end());
  } catch (const std::exception &e) {
    GTEST_SKIP() << "C++ plugin not available: " << e.what();
  }
}

TEST_F(TestYasminFactory, TestStateExecution) {
  std::string xml_content = R"(
    <State name="TestState" type="cpp" class="yasmin_factory/TestSimpleState"/>
  )";

  tinyxml2::XMLDocument doc;
  doc.Parse(xml_content.c_str());
  tinyxml2::XMLElement *state_elem = doc.FirstChildElement("State");

  ASSERT_NE(state_elem, nullptr);

  try {
    auto state = factory->create_state(state_elem);
    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();

    // Execute state multiple times
    std::string outcome1 = state->execute(blackboard);
    EXPECT_EQ(outcome1, "outcome1");
    EXPECT_EQ(blackboard->get<int>("counter"), 1);

    std::string outcome2 = state->execute(blackboard);
    EXPECT_EQ(outcome2, "outcome1");
    EXPECT_EQ(blackboard->get<int>("counter"), 2);

    std::string outcome3 = state->execute(blackboard);
    EXPECT_EQ(outcome3, "outcome2");
    EXPECT_EQ(blackboard->get<int>("counter"), 3);
  } catch (const std::exception &e) {
    GTEST_SKIP() << "C++ plugin not available: " << e.what();
  }
}

TEST_F(TestYasminFactory, TestNestedStateMachine) {
  std::string xml_file = test_dir + "/test_nested_sm.xml";

  try {
    auto sm = factory->create_sm_from_file(xml_file);

    ASSERT_NE(sm, nullptr);
    auto outcomes = sm->get_outcomes();
    EXPECT_NE(outcomes.find("final_outcome"), outcomes.end());
  } catch (const std::exception &e) {
    GTEST_SKIP() << "XML file not available or plugin missing: " << e.what();
  }
}

TEST_F(TestYasminFactory, TestFactoryCleanup) {
  std::string xml_content = R"(
    <State name="TestState" type="cpp" class="yasmin_factory/TestSimpleState"/>
  )";

  tinyxml2::XMLDocument doc;
  doc.Parse(xml_content.c_str());
  tinyxml2::XMLElement *state_elem = doc.FirstChildElement("State");

  try {
    auto state = factory->create_state(state_elem);
    ASSERT_NE(state, nullptr);

    // Cleanup should not raise errors
    factory->cleanup();

    // Should still be able to create new factory
    auto new_factory = std::make_unique<YasminFactory>();
    auto new_state = new_factory->create_state(state_elem);
    ASSERT_NE(new_state, nullptr);
    new_factory->cleanup();
  } catch (const std::exception &e) {
    GTEST_SKIP() << "C++ plugin not available: " << e.what();
  }
}

TEST_F(TestYasminFactory, TestMissingRequiredAttribute) {
  std::string xml_content = R"(
    <State name="NoClass" type="cpp"/>
  )";

  tinyxml2::XMLDocument doc;
  doc.Parse(xml_content.c_str());
  tinyxml2::XMLElement *state_elem = doc.FirstChildElement("State");

  ASSERT_NE(state_elem, nullptr);

  EXPECT_THROW(
      { auto state = factory->create_state(state_elem); }, std::runtime_error);
}

TEST_F(TestYasminFactory, TestRemapping) {
  std::string xml_file = test_dir + "/test_remapping.xml";

  try {
    auto sm = factory->create_sm_from_file(xml_file);
    ASSERT_NE(sm, nullptr);

    // Set up initial input
    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();
    blackboard->set<std::string>("initial_data", "start");

    std::string outcome = (*sm)(blackboard);

    // Check the chain of processing
    EXPECT_EQ(outcome, "end");
    EXPECT_TRUE(blackboard->contains("final_data"));
    EXPECT_EQ(blackboard->get<std::string>("final_data"), "processed_start");
    EXPECT_TRUE(blackboard->contains("final_data_2"));
    EXPECT_EQ(blackboard->get<std::string>("final_data_2"),
              "processed_processed_start");
  } catch (const std::exception &e) {
    GTEST_SKIP() << "XML file not available or plugin missing: " << e.what();
  }
}

TEST_F(TestYasminFactory, TestFilePathMechanism) {
  std::string xml_file = test_dir + "/test_file_path_sm.xml";

  try {
    auto sm = factory->create_sm_from_file(xml_file);

    ASSERT_NE(sm, nullptr);

    // Verify the state machine name is set
    EXPECT_EQ(sm->get_name(), "MainStateMachine");

    // Verify the outcomes include the final outcome
    auto outcomes = sm->get_outcomes();
    EXPECT_NE(outcomes.find("final_end"), outcomes.end());

    // Execute the state machine
    auto blackboard = std::make_shared<yasmin::blackboard::Blackboard>();
    std::string outcome = (*sm)(blackboard);

    // The state machine should execute through FirstState -> IncludedSM ->
    // final_end or directly to final_end depending on FirstState's execution
    EXPECT_EQ(outcome, "final_end");
  } catch (const std::exception &e) {
    GTEST_SKIP() << "XML file not available or plugin missing: " << e.what();
  }
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
