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

#include "yasmin/state.hpp"

namespace py = pybind11;

/**
 * @class CppStateFactory
 * @brief A factory class to create C++ State instances from available
 * State classes using pluginlib.
 *
 * This class uses pluginlib to load and instantiate C++ State classes
 * dynamically. The returned states are pybind11-wrapped yasmin::State
 * instances that can be used directly in Python.
 */
class CppStateFactory {
public:
  /**
   * @brief Constructs a CppStateFactory and initializes the pluginlib
   * ClassLoader.
   */
  CppStateFactory()
      : loader_(std::make_shared<pluginlib::ClassLoader<yasmin::State>>(
            "yasmin", "yasmin::State")) {}

  /**
   * @brief Destructor that destroys the ClassLoader.
   */
  ~CppStateFactory() = default;

  /**
   * @brief Retrieves a list of available C++ State class names.
   * @return A vector of strings representing the available class names.
   */
  std::vector<std::string> available_classes() {
    return this->loader_->getDeclaredClasses();
  }

  /**
   * @brief Creates a yasmin::State instance for the specified C++ State class.
   * @param class_name The name of the C++ State class to instantiate.
   * @return A shared pointer to the created yasmin::State instance.
   * @throws pluginlib::LibraryLoadException if the class cannot be loaded.
   * @throws pluginlib::CreateClassException if the class cannot be
   * instantiated.
   */
  std::shared_ptr<yasmin::State> create(const std::string &class_name) {
    return this->loader_->createSharedInstance(class_name);
  }

private:
  /// The pluginlib ClassLoader for yasmin::State classes (as shared_ptr for
  /// controlled lifetime).
  std::shared_ptr<pluginlib::ClassLoader<yasmin::State>> loader_;
};

PYBIND11_MODULE(yasmin_pybind_bridge, m) {
  m.doc() = "Python bindings for yasmin factory C++ state loading";

// Import the State class from yasmin.state module
// This ensures pybind11 knows how to handle yasmin::State objects
#if __has_include("rclcpp/version.h")
  auto state_module = py::module_::import("yasmin.state");
#else
  auto state_module = py::module::import("yasmin.state");
#endif

  // Get the State class that's already registered in yasmin.state
  auto state_class = state_module.attr("State");

  py::class_<CppStateFactory>(m, "CppStateFactory")
      .def(py::init<>())
      .def("available_classes", &CppStateFactory::available_classes,
           "Get list of available C++ State class names")
      .def(
          "create",
          [state_class](CppStateFactory &self,
                        const std::string &class_name) -> py::object {
            auto cpp_state = self.create(class_name);
            return py::cast(cpp_state);
          },
          py::arg("class_name"), "Create a C++ State instance by class name");
}