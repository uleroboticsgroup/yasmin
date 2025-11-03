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

#include <pluginlib/class_loader.hpp>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/blackboard/blackboard_pywrapper.hpp"
#include "yasmin/state.hpp"

namespace py = pybind11;

/**
 * @class CppStateWrapper
 * @brief A wrapper class to hold a C++ State instance and expose it to Python.
 *
 * This class manages a shared pointer to a yasmin::State instance and provides
 * methods to interact with it from Python.
 */
class CppStateWrapper {
public:
  /**
   * @brief Constructs a CppStateWrapper with a shared pointer to a
   * yasmin::State.
   * @param impl A shared pointer to a yasmin::State instance.
   */
  explicit CppStateWrapper(std::shared_ptr<yasmin::State> impl) : impl_(impl) {}

  /**
   * @brief Retrieves the possible outcomes of the state.
   * @return A set of strings representing the possible outcomes.
   */
  std::set<std::string> get_outcomes() const { return impl_->get_outcomes(); }

  /**
   * @brief Calls the underlying C++ State instance with a Blackboard.
   * @param bb A shared pointer to a yasmin::Blackboard instance.
   * @return The outcome of the state execution as a string.
   */
  std::string operator()(std::shared_ptr<yasmin::blackboard::Blackboard> bb) {
    return (*impl_)(bb);
  }

  /**
   * @brief Calls the underlying C++ State instance with a BlackboardPyWrapper.
   * @param bb_wrapper A reference to a BlackboardPyWrapper instance.
   * @return The outcome of the state execution as a string.
   */
  std::string operator()(yasmin::blackboard::BlackboardPyWrapper &bb_wrapper) {
    return (*impl_)(bb_wrapper.get_cpp_blackboard());
  }

  /**
   * @brief Converts the state to a string representation.
   * @return A string representation of the state.
   */
  std::string to_string() const { return impl_->to_string(); }

private:
  /// The underlying C++ State instance.
  std::shared_ptr<yasmin::State> impl_;
};

/**
 * @class CppStateFactory
 * @brief A factory class to create CppStateWrapper instances from available C++
 * State classes.
 *
 * This class uses pluginlib to load and instantiate C++ State classes
 * dynamically.
 */
class CppStateFactory {
public:
  /**
   * @brief Constructs a CppStateFactory and initializes the pluginlib
   * ClassLoader.
   */
  CppStateFactory() : loader_("yasmin", "yasmin::State") {}

  /**
   * @brief Retrieves a list of available C++ State class names.
   * @return A vector of strings representing the available class names.
   */
  std::vector<std::string> available_classes() {
    return loader_.getDeclaredClasses();
  }

  /**
   * @brief Creates a CppStateWrapper instance for the specified C++ State
   * class.
   * @param class_name The name of the C++ State class to instantiate.
   * @return A CppStateWrapper instance wrapping the created C++ State instance.
   * @throws pluginlib::LibraryLoadException if the class cannot be loaded.
   * @throws pluginlib::CreateClassException if the class cannot be
   * instantiated.
   */
  CppStateWrapper create(const std::string &class_name) {
    auto instance = loader_.createSharedInstance(class_name);
    return CppStateWrapper(instance);
  }

private:
  /// The pluginlib ClassLoader for yasmin::State classes.
  pluginlib::ClassLoader<yasmin::State> loader_;
};

PYBIND11_MODULE(yasmin_pybind_bridge, m) {
  py::class_<CppStateWrapper>(m, "CppState")
      .def("__call__",
           py::overload_cast<yasmin::blackboard::BlackboardPyWrapper &>(
               &CppStateWrapper::operator()))
      .def("__call__",
           py::overload_cast<std::shared_ptr<yasmin::blackboard::Blackboard>>(
               &CppStateWrapper::operator()))
      .def("get_outcomes", &CppStateWrapper::get_outcomes)
      .def("__str__", &CppStateWrapper::to_string);

  py::class_<CppStateFactory>(m, "CppStateFactory")
      .def(py::init<>())
      .def("available_classes", &CppStateFactory::available_classes)
      .def("create", &CppStateFactory::create);
}