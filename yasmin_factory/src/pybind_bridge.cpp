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

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <pluginlib/class_loader.hpp>

#include "yasmin/state.hpp"
#include "yasmin/types.hpp"

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
   * @brief Destructor.
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
  yasmin::State::SharedPtr create(const std::string &class_name) {
    // Create an unmanaged instance of the specified class
    // Python will manage the lifetime via shared_ptr
    auto state = this->loader_->createUnmanagedInstance(class_name);

    // Wrap the raw pointer in a shared_ptr (Python will manage the lifetime)
    yasmin::State::SharedPtr state_ptr(state);

    // Return the shared pointer to the created state
    return state_ptr;
  }

private:
  /// The pluginlib ClassLoader for yasmin::State classes
  std::shared_ptr<pluginlib::ClassLoader<yasmin::State>> loader_;
};

PYBIND11_MODULE(yasmin_pybind_bridge, m) {
  m.doc() = "Python bindings for yasmin factory C++ state loading";

// Import the State class from yasmin.state module
// This ensures pybind11 knows how to handle yasmin::State objects
#if PYBIND11_VERSION_MAJOR > 2 ||                                              \
    (PYBIND11_VERSION_MAJOR == 2 && PYBIND11_VERSION_MINOR >= 6)
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