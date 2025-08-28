// Copyright (C) 2024 Miguel Ángel González Santamarta
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

#include <string>

#include "yasmin/logs.hpp"

namespace yasmin {

/**
 * @brief Default logging function.
 *
 * @param level The log level as a string (e.g., "ERROR", "WARN", "INFO",
 * "DEBUG").
 * @param file The source file where the log function is called.
 * @param function The function where the log function is called.
 * @param line The line number in the source file.
 * @param text The format string for the log message.
 * @param args Additional arguments for the format string.
 */
void default_log_message(LogLevel level, const char *file, const char *function,
                         int line, const char *text) {

  fprintf(stderr, "[%s] [%s:%s:%d] %s\n", log_level_to_name(level), file,
          function, line, text);
}

// Initialize the log level to INFO
LogLevel log_level = INFO;

LogFunction log_message = default_log_message;

void set_log_level(LogLevel new_log_level) { log_level = new_log_level; }

const char *log_level_to_name(LogLevel log_level) {
  switch (log_level) {

  case LogLevel::ERROR:
    return "ERROR";
    break;

  case LogLevel::WARN:
    return "WARN";
    break;

  case LogLevel::INFO:
    return "INFO";
    break;

  case LogLevel::DEBUG:
    return "DEBUG";
    break;
  }

  return "";
}

void set_loggers(LogFunction new_log_message) { log_message = new_log_message; }

void set_default_loggers() { set_loggers(default_log_message); }

} // namespace yasmin
