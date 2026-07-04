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

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>

#include "yasmin/logs.hpp"

namespace py = pybind11;

PYBIND11_MODULE(logs, m) {
  m.doc() = "Python bindings for yasmin logging";

  // Export LogLevel enum
  py::enum_<yasmin::LogLevel>(m, "LogLevel")
      .value("ERROR", yasmin::LogLevel::ERROR,
             "Log level for error messages. Only critical errors should be "
             "logged.")
      .value("WARN", yasmin::LogLevel::WARN,
             "Log level for warning messages. Indicate potential issues that "
             "are not critical.")
      .value("INFO", yasmin::LogLevel::INFO,
             "Log level for informational messages. General runtime "
             "information about the system's state.")
      .value("DEBUG", yasmin::LogLevel::DEBUG,
             "Log level for debug messages. Used for detailed information, "
             "mainly for developers.")
      .export_values();

  // Export log_level as a module attribute (read/write)
  m.def(
      "get_log_level", []() { return yasmin::log_level; },
      "Get the current log level");

  m.def(
      "set_log_level",
      [](yasmin::LogLevel level) { yasmin::set_log_level(level); },
      py::arg("level"), "Set the log level for the YASMIN framework");

  // Export log_level_to_name
  m.def(
      "log_level_to_name",
      [](yasmin::LogLevel level) { return yasmin::log_level_to_name(level); },
      py::arg("level"), "Convert a log level to its string name");

  // Export log helper functions that can be called from Python
  m.def(
      "log_error",
      [](const std::string &file, const std::string &function, int line,
         const std::string &text) {
        if (yasmin::log_level >= yasmin::LogLevel::ERROR) {
          yasmin::log_message(yasmin::LogLevel::ERROR, file.c_str(),
                              function.c_str(), line, text.c_str());
        }
      },
      py::arg("file"), py::arg("function"), py::arg("line"), py::arg("text"),
      "Log an error message");

  m.def(
      "log_warn",
      [](const std::string &file, const std::string &function, int line,
         const std::string &text) {
        if (yasmin::log_level >= yasmin::LogLevel::WARN) {
          yasmin::log_message(yasmin::LogLevel::WARN, file.c_str(),
                              function.c_str(), line, text.c_str());
        }
      },
      py::arg("file"), py::arg("function"), py::arg("line"), py::arg("text"),
      "Log a warning message");

  m.def(
      "log_info",
      [](const std::string &file, const std::string &function, int line,
         const std::string &text) {
        if (yasmin::log_level >= yasmin::LogLevel::INFO) {
          yasmin::log_message(yasmin::LogLevel::INFO, file.c_str(),
                              function.c_str(), line, text.c_str());
        }
      },
      py::arg("file"), py::arg("function"), py::arg("line"), py::arg("text"),
      "Log an info message");

  m.def(
      "log_debug",
      [](const std::string &file, const std::string &function, int line,
         const std::string &text) {
        if (yasmin::log_level >= yasmin::LogLevel::DEBUG) {
          yasmin::log_message(yasmin::LogLevel::DEBUG, file.c_str(),
                              function.c_str(), line, text.c_str());
        }
      },
      py::arg("file"), py::arg("function"), py::arg("line"), py::arg("text"),
      "Log a debug message");

  // Export set_loggers with Python callback support
  m.def(
      "set_loggers",
      [](py::function py_log_func) {
        // Use a raw pointer to avoid static destruction issues
        static py::function *py_log_func_ptr = nullptr;

        // Clean up old function if it exists
        if (py_log_func_ptr) {
          delete py_log_func_ptr;
        }

        // Create new function pointer
        py_log_func_ptr = new py::function(py_log_func);

        // Create a C++ wrapper for the Python logging function
        static std::function<void(yasmin::LogLevel, const char *, const char *,
                                  int, const char *)>
            cpp_log_func = [](yasmin::LogLevel level, const char *file,
                              const char *function, int line,
                              const char *text) {
              // Check if Python is still initialized before trying to acquire
              // GIL This prevents crashes during Python finalization
              if (Py_IsInitialized() && py_log_func_ptr && *py_log_func_ptr) {
                try {
                  py::gil_scoped_acquire acquire;
                  (*py_log_func_ptr)(level, file, function, line, text);
                } catch (...) {
                  // Silently ignore exceptions during finalization
                }
              }
            };

        // Set the logger using a static C-style function pointer
        yasmin::set_loggers([](yasmin::LogLevel level, const char *file,
                               const char *function, int line,
                               const char *text) {
          cpp_log_func(level, file, function, line, text);
        });
      },
      py::arg("log_function"),
      "Set a custom logging function. The function should accept (LogLevel, "
      "str, str, int, str) parameters");

  // Export set_default_loggers
  m.def("set_default_loggers", &yasmin::set_default_loggers,
        "Reset to the default logging function");
}
