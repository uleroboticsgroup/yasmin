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

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <vector>

#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(32, 0, 0)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

#include "yasmin/blackboard_pywrapper.hpp"
#include "yasmin/types.hpp"

#include "yasmin_factory/yasmin_factory.hpp"

namespace yasmin_factory {

namespace {

using StringVector = std::vector<std::string>;
using IntVector = std::vector<int>;
using FloatVector = std::vector<double>;
using BoolVector = std::vector<bool>;

using StringDict = std::unordered_map<std::string, std::string>;
using IntDict = std::unordered_map<std::string, int>;
using FloatDict = std::unordered_map<std::string, double>;
using BoolDict = std::unordered_map<std::string, bool>;

std::string normalize_xml_value_type(const std::string &type_name) {
  std::string normalized;
  normalized.reserve(type_name.size());

  for (unsigned char ch : type_name) {
    if (!std::isspace(ch)) {
      normalized.push_back(static_cast<char>(std::tolower(ch)));
    }
  }

  if (normalized == "string") {
    return "str";
  }
  if (normalized == "integer") {
    return "int";
  }
  if (normalized == "double") {
    return "float";
  }
  if (normalized == "boolean") {
    return "bool";
  }
  if (normalized == "list[string]") {
    return "list[str]";
  }
  if (normalized == "list[double]") {
    return "list[float]";
  }
  if (normalized == "list[boolean]") {
    return "list[bool]";
  }
  if (normalized == "dict[string,str]" || normalized == "dict[str,string]" ||
      normalized == "dict[string,string]") {
    return "dict[str,str]";
  }
  if (normalized == "dict[string,int]" ||
      normalized == "dict[string,integer]" ||
      normalized == "dict[str,integer]") {
    return "dict[str,int]";
  }
  if (normalized == "dict[string,float]" ||
      normalized == "dict[string,double]" || normalized == "dict[str,double]") {
    return "dict[str,float]";
  }
  if (normalized == "dict[string,bool]" ||
      normalized == "dict[string,boolean]" ||
      normalized == "dict[str,boolean]") {
    return "dict[str,bool]";
  }

  return normalized.empty() ? "str" : normalized;
}

bool parse_bool_value(const std::string &value_str) {
  std::string normalized;
  normalized.reserve(value_str.size());

  for (unsigned char ch : value_str) {
    normalized.push_back(static_cast<char>(std::tolower(ch)));
  }

  if (normalized == "true" || normalized == "1" || normalized == "yes" ||
      normalized == "on") {
    return true;
  }
  if (normalized == "false" || normalized == "0" || normalized == "no" ||
      normalized == "off") {
    return false;
  }

  throw std::runtime_error("Invalid boolean default value '" + value_str + "'");
}

py::object load_json_value(const std::string &value_str) {
#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
  py::module_ json_module = py::module_::import("json");
#else
  py::module json_module = py::module::import("json");
#endif

  try {
    return json_module.attr("loads")(value_str);
  } catch (const py::error_already_set &e) {
    throw std::runtime_error("Invalid JSON default value '" + value_str +
                             "': " + std::string(e.what()));
  }
}

bool dict_has_only_string_keys(const py::dict &dict) {
  for (auto item : dict) {
    if (!py::isinstance<py::str>(item.first)) {
      return false;
    }
  }
  return true;
}

template <typename Predicate>
bool sequence_matches(const py::sequence &seq, Predicate pred) {
  for (auto item : seq) {
    if (!pred(item)) {
      return false;
    }
  }
  return true;
}

template <typename Predicate>
bool dict_values_match(const py::dict &dict, Predicate pred) {
  for (auto item : dict) {
    if (!pred(item.second)) {
      return false;
    }
  }
  return true;
}

template <typename T>
std::vector<T> sequence_to_vector(const py::sequence &seq) {
  std::vector<T> result;
  result.reserve(static_cast<std::size_t>(py::len(seq)));

  for (auto item : seq) {
    result.push_back(py::cast<T>(item));
  }

  return result;
}

template <typename T>
std::unordered_map<std::string, T> dict_to_unordered_map(const py::dict &dict) {
  std::unordered_map<std::string, T> result;
  result.reserve(static_cast<std::size_t>(py::len(dict)));

  for (auto item : dict) {
    result.emplace(py::cast<std::string>(item.first), py::cast<T>(item.second));
  }

  return result;
}

template <typename Callback>
void with_typed_xml_value(const std::string &value_str,
                          const std::string &type_str, Callback &&callback) {
  const std::string normalized_type = normalize_xml_value_type(type_str);

  if (normalized_type == "str") {
    callback(value_str);
    return;
  }

  if (normalized_type == "int") {
    std::size_t pos = 0;
    int result = std::stoi(value_str, &pos);
    if (pos != value_str.size()) {
      throw std::runtime_error("Trailing characters in integer value '" +
                               value_str + "'");
    }
    callback(result);
    return;
  }

  if (normalized_type == "float") {
    std::size_t pos = 0;
    double result = std::stod(value_str, &pos);
    if (pos != value_str.size()) {
      throw std::runtime_error("Trailing characters in float value '" +
                               value_str + "'");
    }
    callback(result);
    return;
  }

  if (normalized_type == "bool") {
    callback(parse_bool_value(value_str));
    return;
  }

  py::gil_scoped_acquire acquire;
  py::object parsed = load_json_value(value_str);

  if (normalized_type == "list[str]") {
    if (!py::isinstance<py::list>(parsed)) {
      throw std::runtime_error("Type list[str] expects a JSON array");
    }
    py::sequence seq = parsed.cast<py::sequence>();
    if (!sequence_matches(seq, [](const py::handle &item) {
          return py::isinstance<py::str>(item);
        })) {
      throw std::runtime_error("Type list[str] expects only string entries");
    }
    callback(sequence_to_vector<std::string>(seq));
    return;
  }

  if (normalized_type == "list[int]") {
    if (!py::isinstance<py::list>(parsed)) {
      throw std::runtime_error("Type list[int] expects a JSON array");
    }
    py::sequence seq = parsed.cast<py::sequence>();
    if (!sequence_matches(seq, [](const py::handle &item) {
          return py::isinstance<py::int_>(item) &&
                 !py::isinstance<py::bool_>(item);
        })) {
      throw std::runtime_error("Type list[int] expects only integer entries");
    }
    callback(sequence_to_vector<int>(seq));
    return;
  }

  if (normalized_type == "list[float]") {
    if (!py::isinstance<py::list>(parsed)) {
      throw std::runtime_error("Type list[float] expects a JSON array");
    }
    py::sequence seq = parsed.cast<py::sequence>();
    if (!sequence_matches(seq, [](const py::handle &item) {
          return (py::isinstance<py::int_>(item) &&
                  !py::isinstance<py::bool_>(item)) ||
                 py::isinstance<py::float_>(item);
        })) {
      throw std::runtime_error("Type list[float] expects only numeric entries");
    }
    callback(sequence_to_vector<double>(seq));
    return;
  }

  if (normalized_type == "list[bool]") {
    if (!py::isinstance<py::list>(parsed)) {
      throw std::runtime_error("Type list[bool] expects a JSON array");
    }
    py::sequence seq = parsed.cast<py::sequence>();
    if (!sequence_matches(seq, [](const py::handle &item) {
          return py::isinstance<py::bool_>(item);
        })) {
      throw std::runtime_error("Type list[bool] expects only boolean entries");
    }
    callback(sequence_to_vector<bool>(seq));
    return;
  }

  if (normalized_type == "dict[str,str]") {
    if (!py::isinstance<py::dict>(parsed)) {
      throw std::runtime_error("Type dict[str,str] expects a JSON object");
    }
    py::dict dict = parsed.cast<py::dict>();
    if (!dict_has_only_string_keys(dict) ||
        !dict_values_match(dict, [](const py::handle &item) {
          return py::isinstance<py::str>(item);
        })) {
      throw std::runtime_error(
          "Type dict[str,str] expects string keys and values");
    }
    callback(dict_to_unordered_map<std::string>(dict));
    return;
  }

  if (normalized_type == "dict[str,int]") {
    if (!py::isinstance<py::dict>(parsed)) {
      throw std::runtime_error("Type dict[str,int] expects a JSON object");
    }
    py::dict dict = parsed.cast<py::dict>();
    if (!dict_has_only_string_keys(dict) ||
        !dict_values_match(dict, [](const py::handle &item) {
          return py::isinstance<py::int_>(item) &&
                 !py::isinstance<py::bool_>(item);
        })) {
      throw std::runtime_error(
          "Type dict[str,int] expects string keys and integer values");
    }
    callback(dict_to_unordered_map<int>(dict));
    return;
  }

  if (normalized_type == "dict[str,float]") {
    if (!py::isinstance<py::dict>(parsed)) {
      throw std::runtime_error("Type dict[str,float] expects a JSON object");
    }
    py::dict dict = parsed.cast<py::dict>();
    if (!dict_has_only_string_keys(dict) ||
        !dict_values_match(dict, [](const py::handle &item) {
          return (py::isinstance<py::int_>(item) &&
                  !py::isinstance<py::bool_>(item)) ||
                 py::isinstance<py::float_>(item);
        })) {
      throw std::runtime_error(
          "Type dict[str,float] expects string keys and numeric values");
    }
    callback(dict_to_unordered_map<double>(dict));
    return;
  }

  if (normalized_type == "dict[str,bool]") {
    if (!py::isinstance<py::dict>(parsed)) {
      throw std::runtime_error("Type dict[str,bool] expects a JSON object");
    }
    py::dict dict = parsed.cast<py::dict>();
    if (!dict_has_only_string_keys(dict) ||
        !dict_values_match(dict, [](const py::handle &item) {
          return py::isinstance<py::bool_>(item);
        })) {
      throw std::runtime_error(
          "Type dict[str,bool] expects string keys and boolean values");
    }
    callback(dict_to_unordered_map<bool>(dict));
    return;
  }

  throw std::runtime_error("Unsupported default_type '" + type_str + "'");
}

} // namespace

// Static member initialization
std::unique_ptr<py::scoped_interpreter> YasminFactory::py_interpreter_ =
    nullptr;
bool YasminFactory::py_initialized_ = false;
std::once_flag YasminFactory::py_init_once_;

// PythonStateHolder implementation
PythonStateHolder::PythonStateHolder(yasmin::State::SharedPtr cpp_state,
                                     py::object py_state)
    : yasmin::State(cpp_state->get_outcomes()), cpp_state_(cpp_state),
      py_state_(py_state) {
  this->set_description(cpp_state->get_description());

  for (const auto &[outcome, description] :
       cpp_state->get_outcome_descriptions()) {
    this->set_outcome_description(outcome, description);
  }

  {
    auto existing_inputs = this->get_input_keys();
    const auto key_name_match = [&existing_inputs](const std::string &name) {
      return std::any_of(existing_inputs.begin(), existing_inputs.end(),
                         [&name](const yasmin::BlackboardKeyInfo &k) {
                           return k.name == name;
                         });
    };
    for (const auto &input_key : cpp_state->get_input_keys()) {
      if (!key_name_match(input_key.name)) {
        this->add_input_key(input_key);
      }
    }
  }

  {
    auto existing_outputs = this->get_output_keys();
    const auto key_name_match = [&existing_outputs](const std::string &name) {
      return std::any_of(existing_outputs.begin(), existing_outputs.end(),
                         [&name](const yasmin::BlackboardKeyInfo &k) {
                           return k.name == name;
                         });
    };
    for (const auto &output_key : cpp_state->get_output_keys()) {
      if (!key_name_match(output_key.name)) {
        this->add_output_key(output_key);
      }
    }
  }

  for (const auto &parameter : cpp_state->get_parameters()) {
    if (!this->is_parameter_declared(parameter.name)) {
      this->declare_parameter(parameter);
    }
  }
}

yasmin::State *PythonStateHolder::get_inner_state() {
  return this->cpp_state_.get();
}

void PythonStateHolder::configure() {
  for (const auto &parameter : this->get_parameters()) {
    if (!this->cpp_state_->is_parameter_declared(parameter.name)) {
      this->cpp_state_->declare_parameter(parameter);
    }

    if (this->has_parameter(parameter.name)) {
      this->cpp_state_->copy_parameter_from(*this, parameter.name,
                                            parameter.name);
    }
  }

  this->cpp_state_->configure();
}

std::string
PythonStateHolder::execute(yasmin::Blackboard::SharedPtr blackboard) {
  return this->cpp_state_->execute(blackboard);
}

void PythonStateHolder::cancel_state() { this->cpp_state_->cancel_state(); }

std::string PythonStateHolder::to_string() const {
  return this->cpp_state_->to_string();
}

// YasminFactory implementation
YasminFactory::YasminFactory()
    : state_loader_(std::make_shared<pluginlib::ClassLoader<yasmin::State>>(
          "yasmin", "yasmin::State")) {
  this->initialize_python();
}

YasminFactory::~YasminFactory() { this->cleanup(); }

void YasminFactory::cleanup() {
  if (this->state_loader_) {
    this->state_loader_.reset();
  }
}

namespace {
struct GilSavedState {
  PyGILState_STATE gstate;
  PyThreadState *tstate;
};
thread_local GilSavedState gil_saved_state{};

void gil_before_fork() {
  if (Py_IsInitialized()) {
    gil_saved_state.gstate = PyGILState_Ensure();
    gil_saved_state.tstate = PyEval_SaveThread();
  }
}

void gil_after_join() {
  if (gil_saved_state.tstate) {
    PyEval_RestoreThread(gil_saved_state.tstate);
    PyGILState_Release(gil_saved_state.gstate);
    gil_saved_state.tstate = nullptr;
  }
}
} // namespace

void YasminFactory::initialize_python() {
  // Set GIL hooks for OrthogonalState region threads (once only)
  static bool hooks_set = false;
  if (!hooks_set) {
    yasmin::OrthogonalState::set_thread_hooks(gil_before_fork, gil_after_join);
    hooks_set = true;
  }

  std::call_once(py_init_once_, [this]() {
    if (!Py_IsInitialized()) {
      py_interpreter_ = std::make_unique<py::scoped_interpreter>();
    }
    py_initialized_ = true;

    // Import sys to set up sys.path for module discovery
    try {
      py::gil_scoped_acquire acquire;
#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
      py::module_::import("sys");
#else
      py::module::import("sys");
#endif
    } catch (const py::error_already_set &e) {
      throw std::runtime_error("Failed to initialize Python: " +
                               std::string(e.what()));
    }
  });
}

yasmin::State::SharedPtr
YasminFactory::create_python_state(const std::string &module_name,
                                   const std::string &class_name) const {
  try {
    py::gil_scoped_acquire acquire;
    // Import the yasmin.state module to ensure the State class is registered
#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
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
    auto cpp_state_ptr = py_state.cast<yasmin::State::SharedPtr>();

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
      tokens.emplace_back(std::move(token));
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

void YasminFactory::add_blackboard_keys(yasmin::State::SharedPtr owner,
                                        tinyxml2::XMLElement *parent) const {

  for (tinyxml2::XMLElement *key_elem = parent->FirstChildElement("Key");
       key_elem; key_elem = key_elem->NextSiblingElement("Key")) {
    const std::string key_name = this->get_required_attribute(key_elem, "name");
    std::string key_usage =
        this->get_optional_attribute(key_elem, "type", "in");
    const std::string key_description =
        this->get_optional_attribute(key_elem, "description", "");
    const std::string default_type =
        this->get_optional_attribute(key_elem, "default_type", "str");

    std::transform(key_usage.begin(), key_usage.end(), key_usage.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    const char *default_value_attr = key_elem->Attribute("default_value");

    if (key_usage == "in" || key_usage == "in/out") {
      const auto &existing_inputs = owner->get_input_keys();
      bool already_exists =
          std::any_of(existing_inputs.begin(), existing_inputs.end(),
                      [&key_name](const yasmin::BlackboardKeyInfo &k) {
                        return k.name == key_name;
                      });
      if (!already_exists) {
        if (default_value_attr) {
          with_typed_xml_value(std::string(default_value_attr), default_type,
                               [&](const auto &value) {
                                 owner->add_input_key(yasmin::BlackboardKeyInfo(
                                     key_name, key_description, value));
                               });
        } else {
          owner->add_input_key(
              yasmin::BlackboardKeyInfo(key_name, key_description));
        }
      }
    }

    if (key_usage == "out" || key_usage == "in/out") {
      const auto &existing_outputs = owner->get_output_keys();
      bool already_exists =
          std::any_of(existing_outputs.begin(), existing_outputs.end(),
                      [&key_name](const yasmin::BlackboardKeyInfo &k) {
                        return k.name == key_name;
                      });
      if (!already_exists) {
        owner->add_output_key(
            yasmin::BlackboardKeyInfo(key_name, key_description));
      }
    }
  }

  for (tinyxml2::XMLElement *def_elem = parent->FirstChildElement("Default");
       def_elem; def_elem = def_elem->NextSiblingElement("Default")) {
    const std::string key_name = this->get_required_attribute(def_elem, "key");
    const std::string value_str =
        this->get_required_attribute(def_elem, "value");
    const std::string type_str =
        this->get_optional_attribute(def_elem, "type", "str");
    const std::string key_description =
        this->get_optional_attribute(def_elem, "description", "");

    with_typed_xml_value(value_str, type_str, [&](const auto &value) {
      owner->add_input_key(
          yasmin::BlackboardKeyInfo(key_name, key_description, value));
    });
  }
}

void YasminFactory::add_parameters(yasmin::State::SharedPtr owner,
                                   tinyxml2::XMLElement *parent) const {
  for (tinyxml2::XMLElement *param_elem = parent->FirstChildElement("Param");
       param_elem; param_elem = param_elem->NextSiblingElement("Param")) {
    const std::string parameter_name =
        this->get_required_attribute(param_elem, "name");
    const std::string parameter_description =
        this->get_optional_attribute(param_elem, "description", "");
    const std::string default_type =
        this->get_optional_attribute(param_elem, "default_type", "str");
    const char *default_value_attr = param_elem->Attribute("default_value");

    if (owner->is_parameter_declared(parameter_name)) {
      continue;
    }
    if (default_value_attr) {
      with_typed_xml_value(std::string(default_value_attr), default_type,
                           [&](const auto &value) {
                             owner->declare_parameter(
                                 parameter_name, parameter_description, value);
                           });
    } else if (!parameter_description.empty()) {
      owner->declare_parameter(parameter_name, parameter_description);
    } else {
      owner->declare_parameter(parameter_name);
    }
  }
}

yasmin::ParameterMappings
YasminFactory::get_parameter_mappings(tinyxml2::XMLElement *parent) const {
  yasmin::ParameterMappings parameter_mappings;

  for (tinyxml2::XMLElement *remap_elem =
           parent->FirstChildElement("ParamRemap");
       remap_elem; remap_elem = remap_elem->NextSiblingElement("ParamRemap")) {
    const std::string from = this->get_required_attribute(remap_elem, "old");
    const std::string to = this->get_required_attribute(remap_elem, "new");
    parameter_mappings[from] = to;
  }

  return parameter_mappings;
}

yasmin::State::SharedPtr
YasminFactory::create_state(tinyxml2::XMLElement *state_elem) const {
  std::string type = this->get_optional_attribute(state_elem, "type", "py");
  std::string class_name = this->get_required_attribute(state_elem, "class");

  yasmin::State::SharedPtr state;

  if (type == "py") {
    std::string module_name =
        this->get_required_attribute(state_elem, "module");
    state = this->create_python_state(module_name, class_name);
  } else if (type == "cpp") {
    auto *raw_state = this->state_loader_->createUnmanagedInstance(class_name);
    state = std::shared_ptr<yasmin::State>(
        raw_state,
        [loader = this->state_loader_](yasmin::State *ptr) { delete ptr; });
  } else {
    throw std::runtime_error("Unknown state type: " + type);
  }

  this->add_blackboard_keys(state, state_elem);
  this->add_parameters(state, state_elem);
  return state;
}

yasmin::Concurrence::SharedPtr
YasminFactory::create_concurrence(tinyxml2::XMLElement *conc_elem,
                                  const std::string &base_dir) const {
  std::string default_outcome =
      this->get_optional_attribute(conc_elem, "default_outcome", "");

  yasmin::StateMap states;
  yasmin::OutcomeMap outcome_map;
  yasmin::ParameterMappingsMap parameter_mappings;

  // Parse the concurrence structure
  for (tinyxml2::XMLElement *child = conc_elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {

    std::string child_name = child->Name();

    if (child_name == "State") {
      std::string name = this->get_required_attribute(child, "name");
      states[name] = this->create_state(child);
      parameter_mappings[name] = this->get_parameter_mappings(child);
    } else if (child_name == "Concurrence") {
      std::string name = this->get_required_attribute(child, "name");
      states[name] = this->create_concurrence(child, base_dir);
      parameter_mappings[name] = this->get_parameter_mappings(child);
    } else if (child_name == "StateMachine") {
      std::string name = this->get_required_attribute(child, "name");
      states[name] = this->create_sm(child, base_dir);
      parameter_mappings[name] = this->get_parameter_mappings(child);
    } else if (child_name == "OrthogonalState") {
      std::string name = this->get_required_attribute(child, "name");
      states[name] = this->create_orthogonal_state(child, base_dir);
      parameter_mappings[name] = this->get_parameter_mappings(child);
    } else if (child_name == "JoinState") {
      std::string name = this->get_required_attribute(child, "name");
      states[name] = this->create_join_state(child);
      parameter_mappings[name] = this->get_parameter_mappings(child);
    } else if (child_name != "OutcomeMap" && child_name != "Outcome" &&
               child_name != "FinalOutcome" && child_name != "Key" &&
               child_name != "Default" && child_name != "Param" &&
               child_name != "ParamRemap") {
      YASMIN_LOG_WARN("Unknown element '%s' in Concurrence",
                      child_name.c_str());
    }
  }

  // Parse outcome map
  for (tinyxml2::XMLElement *child = conc_elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {

    const std::string child_tag = child->Name();
    if (child_tag == "OutcomeMap") {
      const std::string outcome_to =
          this->get_required_attribute(child, "outcome");
      outcome_map[outcome_to] = {};

      for (tinyxml2::XMLElement *item = child->FirstChildElement("Item"); item;
           item = item->NextSiblingElement("Item")) {
        const std::string state_name =
            this->get_required_attribute(item, "state");
        const std::string outcome =
            this->get_required_attribute(item, "outcome");

        if (states.find(state_name) != states.end()) {
          outcome_map[outcome_to][state_name] = outcome;
        } else {
          YASMIN_LOG_WARN("OutcomeMap '%s' references unknown state '%s'",
                          outcome_to.c_str(), state_name.c_str());
        }
      }
    } else if (child_tag == "Outcome") {
      const std::string outcome_to = this->get_required_attribute(child, "to");
      outcome_map[outcome_to] = {};

      for (tinyxml2::XMLElement *transition =
               child->FirstChildElement("Transition");
           transition;
           transition = transition->NextSiblingElement("Transition")) {
        const std::string state_name =
            this->get_required_attribute(transition, "state");
        const std::string outcome =
            this->get_required_attribute(transition, "outcome");

        if (states.find(state_name) != states.end()) {
          outcome_map[outcome_to][state_name] = outcome;
        } else {
          YASMIN_LOG_WARN("Outcome '%s' references unknown state '%s'",
                          outcome_to.c_str(), state_name.c_str());
        }
      }
    } else {
      YASMIN_LOG_WARN("Unknown element '%s' in Concurrence outcome map",
                      child_tag.c_str());
    }
  }

  auto concurrence = yasmin::Concurrence::make_shared(
      states, default_outcome, outcome_map, parameter_mappings);

  this->add_blackboard_keys(concurrence, conc_elem);
  this->add_parameters(concurrence, conc_elem);

  for (tinyxml2::XMLElement *outcome_elem =
           conc_elem->FirstChildElement("FinalOutcome");
       outcome_elem;
       outcome_elem = outcome_elem->NextSiblingElement("FinalOutcome")) {
    std::string outcome_name =
        this->get_required_attribute(outcome_elem, "name");
    std::string outcome_description =
        this->get_optional_attribute(outcome_elem, "description", "");

    if (!outcome_description.empty()) {
      concurrence->set_outcome_description(outcome_name, outcome_description);
    }
  }

  return concurrence;
}

yasmin::OrthogonalState::SharedPtr
YasminFactory::create_orthogonal_state(tinyxml2::XMLElement *orth_elem,
                                       const std::string &base_dir) const {
  std::string default_outcome =
      this->get_optional_attribute(orth_elem, "default_outcome", "");

  yasmin::OutcomeMap outcome_map;

  // Parse outcome map (same schema as Concurrence)
  for (tinyxml2::XMLElement *child = orth_elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {
    const std::string child_tag = child->Name();
    if (child_tag == "OutcomeMap") {
      const std::string outcome_to =
          this->get_required_attribute(child, "outcome");
      outcome_map[outcome_to] = {};

      for (tinyxml2::XMLElement *item = child->FirstChildElement("Item"); item;
           item = item->NextSiblingElement("Item")) {
        const std::string state_name =
            this->get_required_attribute(item, "state");
        const std::string outcome =
            this->get_required_attribute(item, "outcome");
        outcome_map[outcome_to][state_name] = outcome;
      }
    } else if (child_tag == "Outcome") {
      const std::string outcome_to = this->get_required_attribute(child, "to");
      outcome_map[outcome_to] = {};

      for (tinyxml2::XMLElement *transition =
               child->FirstChildElement("Transition");
           transition;
           transition = transition->NextSiblingElement("Transition")) {
        const std::string state_name =
            this->get_required_attribute(transition, "state");
        const std::string outcome =
            this->get_required_attribute(transition, "outcome");
        outcome_map[outcome_to][state_name] = outcome;
      }
    } else if (child_tag != "Region" && child_tag != "Key" &&
               child_tag != "Default" && child_tag != "Param" &&
               child_tag != "ParamRemap") {
      YASMIN_LOG_WARN("Unknown element '%s' in OrthogonalState outcome map",
                      child_tag.c_str());
    }
  }

  auto ort = yasmin::OrthogonalState::make_shared(default_outcome, outcome_map);

  // Parse regions
  std::unordered_set<std::string> region_names;
  for (tinyxml2::XMLElement *child = orth_elem->FirstChildElement(); child;
       child = child->NextSiblingElement()) {
    if (std::string(child->Name()) != "Region") {
      continue;
    }

    std::string region_name = this->get_required_attribute(child, "name");
    auto region_sm = this->create_sm(child, base_dir);
    region_sm->set_name(region_name);
    ort->add_region(region_name, region_sm);
    region_names.insert(region_name);
  }

  // Validate outcome map items against region names
  for (const auto &[outcome_name, state_map] : outcome_map) {
    for (const auto &[state_name, unused_outcome] : state_map) {
      (void)unused_outcome;
      if (region_names.find(state_name) == region_names.end()) {
        YASMIN_LOG_WARN(
            "OutcomeMap '%s' references unknown region '%s' in OrthogonalState",
            outcome_name.c_str(), state_name.c_str());
      }
    }
  }

  this->add_blackboard_keys(ort, orth_elem);
  this->add_parameters(ort, orth_elem);

  return ort;
}

yasmin::JoinState::SharedPtr
YasminFactory::create_join_state(tinyxml2::XMLElement *join_elem) const {
  const std::string sync_id =
      this->get_optional_attribute(join_elem, "sync_id", "");
  const std::string outcome =
      this->get_optional_attribute(join_elem, "outcome", "joined");

  return std::make_shared<yasmin::JoinState>(sync_id, outcome);
}

yasmin::StateMachine::SharedPtr
YasminFactory::create_sm(tinyxml2::XMLElement *root,
                         const std::string &base_dir) const {

  std::string file_path = this->get_optional_attribute(root, "file_path", "");
  bool from_file_path_attr = !file_path.empty();

  if (file_path.empty()) {
    std::string file_name = this->get_optional_attribute(root, "file_name", "");
    std::string package = this->get_optional_attribute(root, "package", "");

    if (!file_name.empty() && !package.empty()) {
      std::string package_path;
      try {

#if __has_include("rclcpp/version.h")
#if RCLCPP_VERSION_GTE(32, 0, 0)
        package_path =
            ament_index_cpp::get_package_share_path(package).string();
#else
        package_path = ament_index_cpp::get_package_share_directory(package);
#endif
#else
        package_path = ament_index_cpp::get_package_share_directory(package);
#endif

        file_path = "";
        std::vector<std::string> matches;
        for (const auto &entry :
             std::filesystem::recursive_directory_iterator(package_path)) {
          if (entry.is_regular_file() && entry.path().filename() == file_name) {
            matches.push_back(entry.path().string());
          }
        }
        if (matches.size() > 1) {
          YASMIN_LOG_WARN(
              "Found %zu matches for file '%s' in package '%s', using '%s'",
              matches.size(), file_name.c_str(), package.c_str(),
              matches[0].c_str());
        }
        if (!matches.empty()) {
          file_path = matches[0];
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
      if (base_dir.empty()) {
        throw std::runtime_error(
            "Cannot resolve relative file_path without a base directory");
      }
      file_path = (std::filesystem::path(base_dir) / file_path).string();
    }
    // Prevent path traversal outside the base directory, but only for
    // user-supplied file_path attributes. Paths resolved via package +
    // file_name are trusted (they come from the ament package registry)
    // and may legitimately point to another package's share directory.
    if (from_file_path_attr && !base_dir.empty()) {
      std::string xml_dir =
          std::filesystem::weakly_canonical(base_dir).string();
      std::string resolved =
          std::filesystem::weakly_canonical(file_path).string();
      if (resolved != xml_dir &&
          resolved.find(xml_dir + std::filesystem::path::preferred_separator) !=
              0) {
        throw std::runtime_error("File path '" + file_path +
                                 "' resolves outside the XML directory");
      }
      file_path = resolved;
    }
    return this->create_sm_from_file(file_path);
  }

  std::string outcomes_str = this->get_optional_attribute(root, "outcomes", "");
  std::string set_start_state =
      this->get_optional_attribute(root, "start_state", "");
  std::vector<std::string> outcomes_vec;
  {
    std::istringstream iss(outcomes_str);
    std::string token;
    while (iss >> token) {
      outcomes_vec.push_back(token);
    }
  }
  yasmin::Outcomes outcomes(outcomes_vec.begin(), outcomes_vec.end());

  auto sm = yasmin::StateMachine::make_shared(outcomes);

  // Parse all child elements
  for (tinyxml2::XMLElement *child = root->FirstChildElement(); child;
       child = child->NextSiblingElement()) {

    std::string child_name = child->Name();
    if (child_name != "State" && child_name != "Concurrence" &&
        child_name != "StateMachine" && child_name != "OrthogonalState" &&
        child_name != "JoinState") {
      if (child_name != "FinalOutcome" && child_name != "Key" &&
          child_name != "Default" && child_name != "Param" &&
          child_name != "ParamRemap") {
        YASMIN_LOG_WARN("Unknown element '%s' in StateMachine",
                        child_name.c_str());
      }
      continue;
    }

    std::string name = this->get_required_attribute(child, "name");

    // Parse transitions
    yasmin::Transitions transitions;
    for (tinyxml2::XMLElement *transition =
             child->FirstChildElement("Transition");
         transition;
         transition = transition->NextSiblingElement("Transition")) {
      std::string from = this->get_required_attribute(transition, "from");
      std::string to = this->get_required_attribute(transition, "to");
      transitions[from] = to;
    }

    // Parse remappings
    yasmin::Remappings remappings;
    for (tinyxml2::XMLElement *transition = child->FirstChildElement("Remap");
         transition; transition = transition->NextSiblingElement("Remap")) {
      std::string from = this->get_required_attribute(transition, "old");
      std::string to = this->get_required_attribute(transition, "new");
      remappings[from] = to;
    }

    const yasmin::ParameterMappings parameter_mappings =
        this->get_parameter_mappings(child);

    // Create the state
    yasmin::State::SharedPtr state;
    if (child_name == "State") {
      state = this->create_state(child);
    } else if (child_name == "Concurrence") {
      state = this->create_concurrence(child, base_dir);
    } else if (child_name == "StateMachine") {
      state = this->create_sm(child, base_dir);
    } else if (child_name == "OrthogonalState") {
      state = this->create_orthogonal_state(child, base_dir);
    } else if (child_name == "JoinState") {
      state = this->create_join_state(child);
    } else {
      continue;
    }

    // Add state to state machine
    sm->add_state(name, state, transitions, remappings, parameter_mappings);
  }

  // Set initial state if specified
  if (!set_start_state.empty()) {
    sm->set_start_state(set_start_state);
  }

  // Parse description attribute
  std::string description =
      this->get_optional_attribute(root, "description", "");
  if (!description.empty()) {
    sm->set_description(description);
  }

  // Parse outcome descriptions
  for (tinyxml2::XMLElement *outcome_elem =
           root->FirstChildElement("FinalOutcome");
       outcome_elem;
       outcome_elem = outcome_elem->NextSiblingElement("FinalOutcome")) {
    std::string outcome_name =
        this->get_required_attribute(outcome_elem, "name");
    std::string outcome_description =
        this->get_optional_attribute(outcome_elem, "description", "");

    if (!outcome_description.empty()) {
      sm->set_outcome_description(outcome_name, outcome_description);
    }
  }

  this->add_blackboard_keys(sm, root);
  this->add_parameters(sm, root);
  return sm;
}

yasmin::StateMachine::SharedPtr
YasminFactory::create_sm_from_file(const std::string &xml_file) const {
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

  // Use the XML file's parent directory as the base for relative includes
  std::string base_dir = std::filesystem::path(xml_file).parent_path().string();

  // Create the state machine
  auto sm = this->create_sm(root, base_dir);
  sm->set_name(sm_name);
  return sm;
}

} // namespace yasmin_factory
