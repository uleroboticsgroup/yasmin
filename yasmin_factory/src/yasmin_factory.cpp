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

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>

#include "yasmin/blackboard/blackboard_pywrapper.hpp"

namespace yasmin_factory {

// Static member initialization
std::unique_ptr<py::scoped_interpreter> YasminFactory::py_interpreter_ =
    nullptr;
bool YasminFactory::py_initialized_ = false;

// PythonStateAdapter implementation
PythonStateAdapter::PythonStateAdapter(py::object py_state,
                                       std::set<std::string> outcomes)
    : yasmin::State(outcomes), py_state_(py_state) {}

std::string PythonStateAdapter::execute(
    std::shared_ptr<yasmin::blackboard::Blackboard> blackboard) {
  try {
    py::gil_scoped_acquire acquire;

    // Wrap the C++ blackboard for Python
    yasmin::blackboard::BlackboardPyWrapper bb_wrapper(blackboard);

    // Call the Python state's execute method
    py::object result = py_state_.attr("execute")(bb_wrapper);

    // Convert result to string
    return result.cast<std::string>();
  } catch (const py::error_already_set &e) {
    throw std::runtime_error("Python state execution error: " +
                             std::string(e.what()));
  }
}

std::string PythonStateAdapter::to_string() {
  try {
    py::gil_scoped_acquire acquire;
    py::object str_repr = py_state_.attr("__str__")();
    return str_repr.cast<std::string>();
  } catch (const py::error_already_set &e) {
    return "PythonState(error getting string representation)";
  }
}

// YasminFactory implementation
YasminFactory::YasminFactory()
    : state_loader_(std::make_unique<pluginlib::ClassLoader<yasmin::State>>(
          "yasmin", "yasmin::State")) {
  this->initialize_python();
}

YasminFactory::~YasminFactory() { this->cleanup(); }

void YasminFactory::cleanup() {
  if (state_loader_) {
    state_loader_.reset();
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
      py::module_::import("sys");
    } catch (const py::error_already_set &e) {
      throw std::runtime_error("Failed to initialize Python: " +
                               std::string(e.what()));
    }
  }
}

std::shared_ptr<yasmin::State>
YasminFactory::create_python_state(const std::string &module_name,
                                   const std::string &class_name,
                                   const std::vector<std::string> &parameters) {
  try {
    py::gil_scoped_acquire acquire;

    // Import the module
    py::module_ module = py::module_::import(module_name.c_str());

    // Get the class
    py::object state_class = module.attr(class_name.c_str());

    // Create instance with or without parameters
    py::object py_state;
    if (!parameters.empty()) {
      py::list py_params;
      for (const auto &param : parameters) {
        py_params.append(param);
      }
      py_state = state_class(*py_params);
    } else {
      py_state = state_class();
    }

    // Get the outcomes from the Python state
    py::object py_outcomes = py_state.attr("get_outcomes")();
    std::set<std::string> outcomes = py_outcomes.cast<std::set<std::string>>();

    // Create and return the adapter
    return std::make_shared<PythonStateAdapter>(py_state, outcomes);

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
    // Get required module name for Python states
    std::string module_name =
        this->get_required_attribute(state_elem, "module");

    // Get optional parameters
    std::string params_str =
        this->get_optional_attribute(state_elem, "parameters", "");
    std::vector<std::string> parameters;
    if (!params_str.empty()) {
      parameters = this->split_string(params_str, ',');
    }

    return this->create_python_state(module_name, class_name, parameters);
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
  std::string outcomes_str = this->get_optional_attribute(root, "outcomes", "");
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

    // Create the state
    std::shared_ptr<yasmin::State> state;
    if (child_name == "State") {
      state = this->create_state(child);
    } else if (child_name == "Concurrence") {
      state = this->create_concurrence(child);
    } else if (child_name == "StateMachine") {
      state = this->create_sm(child);
    }

    // Add state to state machine
    sm->add_state(name, state, transitions);
  }

  return sm;
}

std::shared_ptr<yasmin::StateMachine>
YasminFactory::create_sm_from_file(const std::string &xml_file) {
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

  return this->create_sm(root);
}

} // namespace yasmin_factory
