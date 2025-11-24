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

#include "yasmin_factory/yasmin_factory.hpp"

#include "yasmin/blackboard/blackboard_pywrapper.hpp"
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace yasmin_factory {

// Static member initialization
std::unique_ptr<py::scoped_interpreter> YasminFactory::py_interpreter_ =
    nullptr;
bool YasminFactory::py_initialized_ = false;

// PythonStateHolder implementation
PythonStateHolder::PythonStateHolder(std::shared_ptr<yasmin::State> cpp_state,
                                     py::object py_state)
    : yasmin::State(cpp_state->get_outcomes()), cpp_state_(cpp_state),
      py_state_(py_state) {}

std::string PythonStateHolder::execute(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  return this->cpp_state_->execute(blackboard);
}

void PythonStateHolder::cancel_state() { this->cpp_state_->cancel_state(); }

std::string PythonStateHolder::to_string() {
  return this->cpp_state_->to_string();
}

// YasminFactory implementation
YasminFactory::YasminFactory()
    : state_loader_(std::make_unique<pluginlib::ClassLoader<yasmin::State>>(
          "yasmin", "yasmin::State")) {
  this->initialize_python();
}

YasminFactory::~YasminFactory() { this->cleanup(); }

void YasminFactory::cleanup() {
  if (this->state_loader_) {
    this->state_loader_.reset();
  }
}

void YasminFactory::initialize_python() {
  if (!py_initialized_) {
    // Check if Python is already initialized (e.g., by ROS or another module)
    if (!Py_IsInitialized()) {
      py_interpreter_ = std::make_unique<py::scoped_interpreter>();
    }
    py_initialized_ = true;

    // Import sys to ensure Python path is set up
    try {
      py::gil_scoped_acquire acquire;
#if __has_include("rclcpp/version.h")
      py::module_::import("sys");
#else
      py::module::import("sys");
#endif
    } catch (const py::error_already_set &e) {
      throw std::runtime_error("Failed to initialize Python: " +
                               std::string(e.what()));
    }
  }
}

std::shared_ptr<yasmin::State>
YasminFactory::create_python_state(const std::string &module_name,
                                   const std::string &class_name) {
  try {
    py::gil_scoped_acquire acquire;
    // Import the yasmin.state module to ensure the State class is registered
#if __has_include("rclcpp/version.h")
    py::module_::import("yasmin.state");
    py::module_ module = py::module_::import(module_name.c_str());
#else
    py::module::import("yasmin.state");
    py::module module = py::module::import(module_name.c_str());
#endif

    // Get the class
    py::object state_class = module.attr(class_name.c_str());

    // Create instance
    py::object py_state = state_class();

    // Extract the C++ pointer from the Python object
    auto cpp_state_ptr = py_state.cast<std::shared_ptr<yasmin::State>>();

    // Wrap it in a holder that keeps the Python object alive
    return std::make_shared<PythonStateHolder>(cpp_state_ptr, py_state);

  } catch (const py::error_already_set &e) {
    throw std::runtime_error("Failed to create Python state '" + module_name +
                             "." + class_name + "': " + std::string(e.what()));
  }
}

std::string YasminFactory::trim(const std::string &str) const {
  auto start = std::find_if_not(str.begin(), str.end(), [](unsigned char ch) {
    return std::isspace(ch);
  });
  auto end = std::find_if_not(str.rbegin(), str.rend(), [](unsigned char ch) {
               return std::isspace(ch);
             }).base();
  return (start < end) ? std::string(start, end) : std::string();
}

std::vector<std::string> YasminFactory::split_string(const std::string &str,
                                                     char delimiter) const {
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;

  while (std::getline(ss, token, delimiter)) {
    token = this->trim(token);
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }

  return tokens;
}

std::string
YasminFactory::get_required_attribute(tinyxml2::XMLElement *element,
                                      const std::string &attr_name) const {
  const char *attr = element->Attribute(attr_name.c_str());
  if (!attr) {
    throw std::runtime_error("Missing required attribute '" + attr_name +
                             "' in element '" + element->Name() + "'");
  }
  return std::string(attr);
}

std::string
YasminFactory::get_optional_attribute(tinyxml2::XMLElement *element,
                                      const std::string &attr_name,
                                      const std::string &default_value) const {
  const char *attr = element->Attribute(attr_name.c_str());
  return attr ? std::string(attr) : default_value;
}

std::shared_ptr<yasmin::State>
YasminFactory::create_state(tinyxml2::XMLElement *state_elem) {
  std::string state_type =
      this->get_optional_attribute(state_elem, "type", "cpp");
  std::string class_name = this->get_required_attribute(state_elem, "class");

  if (state_type == "cpp") {
    try {
      return state_loader_->createSharedInstance(class_name);
    } catch (const pluginlib::PluginlibException &ex) {
      throw std::runtime_error("Failed to load C++ state class '" + class_name +
                               "': " + ex.what());
    }
  } else if (state_type == "py") {
    std::string module_name =
        this->get_required_attribute(state_elem, "module");
    return this->create_python_state(module_name, class_name);
  } else {
    throw std::runtime_error("Unknown state type: " + state_type);
  }
}

std::shared_ptr<yasmin::Concurrence>
YasminFactory::create_concurrence(tinyxml2::XMLElement *conc_elem) {
  std::string default_outcome =
      this->get_optional_attribute(conc_elem, "default_outcome", "");

  std::map<std::string, std::shared_ptr<yasmin::State>> states;
  yasmin::Concurrence::OutcomeMap outcome_map;

  // Parse the concurrence structure
  for (tinyxml2::XMLElement *child = conc_elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {

    std::string child_name = child->Name();

    if (child_name == "State") {
      std::string name = this->get_required_attribute(child, "name");
      states[name] = this->create_state(child);
    } else if (child_name == "Concurrence") {
      std::string name = this->get_required_attribute(child, "name");
      states[name] = this->create_concurrence(child);
    } else if (child_name == "StateMachine") {
      std::string name = this->get_required_attribute(child, "name");
      states[name] = this->create_sm(child);
    }
  }

  // Parse outcome map
  for (tinyxml2::XMLElement *child = conc_elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {

    if (std::string(child->Name()) == "Outcome") {
      std::string outcome_to = this->get_required_attribute(child, "to");
      outcome_map[outcome_to] = {};

      for (tinyxml2::XMLElement *transition =
               child->FirstChildElement("Transition");
           transition;
           transition = transition->NextSiblingElement("Transition")) {
        std::string state_name =
            this->get_required_attribute(transition, "state");
        std::string outcome =
            this->get_required_attribute(transition, "outcome");

        if (states.find(state_name) != states.end()) {
          outcome_map[outcome_to][states[state_name]->to_string()] = outcome;
        }
      }
    }
  }

  return std::make_shared<yasmin::Concurrence>(states, default_outcome,
                                               outcome_map);
}

std::shared_ptr<yasmin::StateMachine>
YasminFactory::create_sm(tinyxml2::XMLElement *root) {

  std::string file_path = this->get_optional_attribute(root, "file_path", "");

  if (file_path.empty()) {
    std::string file_name = this->get_optional_attribute(root, "file_name", "");
    std::string package = this->get_optional_attribute(root, "package", "");

    if (!file_name.empty() && !package.empty()) {
      std::string package_path;
      try {
        package_path = ament_index_cpp::get_package_share_directory(package);
        file_path = "";
        for (const auto &entry :
             std::filesystem::recursive_directory_iterator(package_path)) {
          if (entry.is_regular_file() && entry.path().filename() == file_name) {
            file_path = entry.path().string();
            break;
          }
        }
      } catch (const ament_index_cpp::PackageNotFoundError &e) {
        file_path = "";
      } catch (const std::filesystem::filesystem_error &e) {
        file_path = "";
      }
    }
  }

  // Check if StateMachine is an included XML file
  if (!file_path.empty()) {
    if (!std::filesystem::path(file_path).is_absolute()) {
      file_path =
          (std::filesystem::path(this->xml_path_).parent_path() / file_path)
              .string();
    }
    return this->create_sm_from_file(file_path);
  }

  std::string outcomes_str = this->get_optional_attribute(root, "outcomes", "");
  std::string set_start_state =
      this->get_optional_attribute(root, "start_state", "");
  std::vector<std::string> outcomes_vec = this->split_string(outcomes_str, ' ');
  std::set<std::string> outcomes(outcomes_vec.begin(), outcomes_vec.end());

  auto sm = std::make_shared<yasmin::StateMachine>(outcomes);

  // Parse all child elements
  for (tinyxml2::XMLElement *child = root->FirstChildElement(); child;
       child = child->NextSiblingElement()) {

    std::string child_name = child->Name();
    if (child_name != "State" && child_name != "Concurrence" &&
        child_name != "StateMachine") {
      continue;
    }

    std::string name = this->get_required_attribute(child, "name");

    // Parse transitions
    std::map<std::string, std::string> transitions;
    for (tinyxml2::XMLElement *transition =
             child->FirstChildElement("Transition");
         transition;
         transition = transition->NextSiblingElement("Transition")) {
      std::string from = this->get_required_attribute(transition, "from");
      std::string to = this->get_required_attribute(transition, "to");
      transitions[from] = to;
    }

    // Parse remappings
    std::map<std::string, std::string> remappings;
    for (tinyxml2::XMLElement *transition = child->FirstChildElement("Remap");
         transition; transition = transition->NextSiblingElement("Remap")) {
      std::string from = this->get_required_attribute(transition, "old");
      std::string to = this->get_required_attribute(transition, "new");
      remappings[from] = to;
    }

    // Create the state
    std::shared_ptr<yasmin::State> state;
    if (child_name == "State") {
      state = this->create_state(child);
    } else if (child_name == "Concurrence") {
      state = this->create_concurrence(child);
    } else if (child_name == "StateMachine") {
      state = this->create_sm(child);
    } else {
      continue;
    }

    // Add state to state machine
    sm->add_state(name, state, transitions, remappings);
  }

  // Set initial state if specified
  if (!set_start_state.empty()) {
    sm->set_start_state(set_start_state);
  }

  return sm;
}

std::shared_ptr<yasmin::StateMachine>
YasminFactory::create_sm_from_file(const std::string &xml_file) {
  this->xml_path_ = xml_file;
  tinyxml2::XMLDocument doc;
  tinyxml2::XMLError error = doc.LoadFile(xml_file.c_str());

  if (error != tinyxml2::XML_SUCCESS) {
    throw std::runtime_error("Failed to load XML file: " + xml_file +
                             " (Error code: " + std::to_string(error) + ")");
  }

  tinyxml2::XMLElement *root = doc.RootElement();
  if (!root) {
    throw std::runtime_error("XML file has no root element: " + xml_file);
  }

  if (std::string(root->Name()) != "StateMachine") {
    throw std::runtime_error("Root element must be 'StateMachine', found: " +
                             std::string(root->Name()));
  }

  // Read the name of the state machine root if exists
  std::string sm_name = this->get_optional_attribute(root, "name", "");

  // Create the state machine
  auto sm = this->create_sm(root);
  sm->set_name(sm_name);
  return sm;
}

} // namespace yasmin_factory
