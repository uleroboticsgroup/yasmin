// Copyright (C) 2024 Miguel Ángel González Santamarta
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

#include "yasmin/logs.hpp"

#include <string>

namespace yasmin {

void default_log_message(LogLevel level, const char *file, const char *function,
                         int line, const char *text) {
  fprintf(stderr, "[%s] [%s:%s:%d] %s\n", log_level_to_name(level), file,
          function, line, text);
}

// Default to INFO so debug/spam messages are hidden unless explicitly enabled
LogLevel log_level = LogLevel::INFO;

LogFunction log_message = default_log_message;

void set_log_level(LogLevel new_log_level) { log_level = new_log_level; }

const char *log_level_to_name(LogLevel log_level) {
  switch (log_level) {

  case LogLevel::ERROR:
    return "ERROR";
  case LogLevel::WARN:
    return "WARN";
  case LogLevel::INFO:
    return "INFO";
  case LogLevel::DEBUG:
    return "DEBUG";

  default:
    return "";
  }

  return "";
}

void set_loggers(LogFunction new_log_message) { log_message = new_log_message; }

void set_default_loggers() { set_loggers(default_log_message); }

} // namespace yasmin
