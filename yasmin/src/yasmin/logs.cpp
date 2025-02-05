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

#include "yasmin/logs.hpp"

namespace yasmin {

/**
 * @brief Generalized logging function.
 *
 * @param level The log level as a string (e.g., "ERROR", "WARN", "INFO",
 * "DEBUG").
 * @param file The source file where the log function is called.
 * @param function The function where the log function is called.
 * @param line The line number in the source file.
 * @param text The format string for the log message.
 * @param args Additional arguments for the format string.
 */
void log_message(const char *level, const char *file, const char *function,
                 int line, const char *text, va_list args) {
  fprintf(stderr, "[%s] [%s:%s:%d] ", level, file, function, line);
  vfprintf(stderr, text, args);
  fprintf(stderr, "\n");
}

/**
 * @brief Variadic template function to log messages at different levels.
 *
 * This function wraps log_message and allows logging messages with different
 * log levels while reducing redundant code. It provides a consistent logging
 * format across all levels.
 *
 * @tparam LEVEL The log level string (e.g., "ERROR", "WARN", "INFO", "DEBUG").
 * @param file The source file where the log function is called.
 * @param function The function where the log function is called.
 * @param line The line number in the source file.
 * @param text The format string for the log message.
 * @param ... Additional arguments for the format string.
 */
template <const char *LEVEL>
void log_helper(const char *file, const char *function, int line,
                const char *text, ...) {
  va_list args;
  va_start(args, text);
  log_message(LEVEL, file, function, line, text, args);
  va_end(args);
}

/**
 * @brief Log level strings used in logging functions.
 */
constexpr const char ERROR_LEVEL[] = "ERROR"; ///< Error log level
constexpr const char WARN_LEVEL[] = "WARN";   ///< Warning log level
constexpr const char INFO_LEVEL[] = "INFO";   ///< Info log level
constexpr const char DEBUG_LEVEL[] = "DEBUG"; ///< Debug log level

/**
 * @brief Explicit template instantiations for log_helper with different log
 * levels.
 *
 * These instantiations ensure that log_helper functions for each log level are
 * available.
 */
template void log_helper<ERROR_LEVEL>(const char *, const char *, int,
                                      const char *, ...);
template void log_helper<WARN_LEVEL>(const char *, const char *, int,
                                     const char *, ...);
template void log_helper<INFO_LEVEL>(const char *, const char *, int,
                                     const char *, ...);
template void log_helper<DEBUG_LEVEL>(const char *, const char *, int,
                                      const char *, ...);

// Assign default logging functions
LogFunction log_error = log_helper<ERROR_LEVEL>;
LogFunction log_warn = log_helper<WARN_LEVEL>;
LogFunction log_info = log_helper<INFO_LEVEL>;
LogFunction log_debug = log_helper<DEBUG_LEVEL>;

void set_loggers(LogFunction error, LogFunction warn, LogFunction info,
                 LogFunction debug) {
  log_error = error ? error : log_helper<ERROR_LEVEL>;
  log_warn = warn ? warn : log_helper<WARN_LEVEL>;
  log_info = info ? info : log_helper<INFO_LEVEL>;
  log_debug = debug ? debug : log_helper<DEBUG_LEVEL>;
}

void set_default_loggers() {
  set_loggers(log_helper<ERROR_LEVEL>, log_helper<WARN_LEVEL>,
              log_helper<INFO_LEVEL>, log_helper<DEBUG_LEVEL>);
}

// Initialize the log level to INFO
LogLevel log_level = INFO;

void set_log_level(LogLevel log_level) { log_level = log_level; }

} // namespace yasmin
