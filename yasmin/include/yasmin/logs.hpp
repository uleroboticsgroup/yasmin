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

#ifndef YASMIN__LOGS_HPP
#define YASMIN__LOGS_HPP

#include <cstdarg>
#include <cstring>
#include <string>

namespace yasmin {

/**
 * @brief Enum representing different log levels for controlling log verbosity.
 *
 * This enum defines the severity levels of logs that can be used to control
 * which log messages should be displayed. The levels are ordered from most
 * severe to least severe. Only logs at or above the current log level will be
 * shown.
 */
enum LogLevel {
  /// Log level for error messages. Only critical errors should be logged.
  ERROR = 0,
  /// Log level for warning messages. Indicate potential issues that are not
  /// critical.
  WARN,
  /// Log level for informational messages. General runtime information about
  /// the system's state.
  INFO,
  /// Log level for debug messages. Used for detailed information, mainly for
  /// developers.
  DEBUG
};

/**
 * @brief The current log level for the application.
 *
 * This global variable holds the current log level, which determines the
 * verbosity of the logs. Logs at or above this level will be displayed. The
 * default level is set to INFO.
 */
extern LogLevel log_level;

/**
 * @brief Sets the log level for the logs.
 *
 * This function allows the user to specify the log level error, warning, info,
 * or debug.
 *
 * @param new_log_level Log level.
 */
void set_log_level(LogLevel new_log_level);

/**
 * @brief Parse LogLevel to string.
 *
 * This function returns the name of a given log level.
 *
 * @param level Log level.
 */
const char *log_level_to_name(LogLevel level);

/**
 * @brief Type definition for a logging function.
 *
 * This type represents a function pointer that takes a file name,
 * function name, line number, log message, and a variable number of
 * additional arguments for formatting the log message.
 *
 * @param level The log level of the message
 * @param file The name of the source file where the log function is called.
 * @param function The name of the function where the log function is called.
 * @param line The line number in the source file where the log function is
 * called.
 * @param text The format string for the log message, similar to printf.
 * @param ... Additional arguments for the format string.
 */
typedef void (*LogFunction)(LogLevel level, const char *file,
                            const char *function, int line, const char *text);

extern LogFunction log_message; ///< Pointer to the logging function

/**
 * @brief Variadic template function to log messages at different levels.
 *
 * This function wraps log_message and allows logging messages with different
 * log levels while reducing redundant code. It provides a consistent logging
 * format across all levels.
 *
 * @tparam LEVEL The log level LogLevel (e.g., 0 -> "ERROR", 1 -> "WARN", 2 ->
 * "INFO", 3 -> "DEBUG").
 * @param log_message Function to create the logs
 * @param file The source file where the log function is called.
 * @param function The function where the log function is called.
 * @param line The line number in the source file.
 * @param text The format string for the log message.
 * @param ... Additional arguments for the format string.
 */
template <yasmin::LogLevel LEVEL>
void log_helper(const char *file, const char *function, int line,
                const char *text, ...) {
  va_list args;
  va_start(args, text);

  // Calculate the required buffer size
  int size = vsnprintf(nullptr, 0, text, args) + 1;
  va_end(args);

  std::string buffer(size, '\0');
  va_start(args, text);
  vsnprintf(&buffer[0], buffer.size(), text, args);

  va_end(args);

  yasmin::log_message(LEVEL, file, function, line, buffer.c_str());
}

/**
 * @brief Extracts the filename from a given file path.
 *
 * This function takes a full path to a file and returns just the file name.
 *
 * @param path The full path to the file.
 * @return A pointer to the extracted filename.
 */
inline const char *extract_filename(const char *path) {
  const char *filename = std::strrchr(path, '/');
  if (!filename) {
    filename = std::strrchr(path, '\\'); // handle Windows-style paths
  }
  return filename ? filename + 1 : path;
}

// Macros for logging with automatic file and function information
#define YASMIN_LOG_ERROR(text, ...)                                            \
  if (yasmin::log_level >= yasmin::ERROR)                                      \
  yasmin::log_helper<yasmin::ERROR>(::yasmin::extract_filename(__FILE__),      \
                                    __FUNCTION__, __LINE__, text,              \
                                    ##__VA_ARGS__)
#define YASMIN_LOG_WARN(text, ...)                                             \
  if (yasmin::log_level >= yasmin::WARN)                                       \
  yasmin::log_helper<yasmin::WARN>(::yasmin::extract_filename(__FILE__),       \
                                   __FUNCTION__, __LINE__, text,               \
                                   ##__VA_ARGS__)
#define YASMIN_LOG_INFO(text, ...)                                             \
  if (yasmin::log_level >= yasmin::INFO)                                       \
  yasmin::log_helper<yasmin::INFO>(::yasmin::extract_filename(__FILE__),       \
                                   __FUNCTION__, __LINE__, text,               \
                                   ##__VA_ARGS__)
#define YASMIN_LOG_DEBUG(text, ...)                                            \
  if (yasmin::log_level >= yasmin::DEBUG)                                      \
  yasmin::log_helper<yasmin::DEBUG>(::yasmin::extract_filename(__FILE__),      \
                                    __FUNCTION__, __LINE__, text,              \
                                    ##__VA_ARGS__)

/**
 * @brief Sets custom logging functions for different log levels.
 *
 * This function allows the user to specify custom logging functions for
 * error, warning, info, and debug logs. If a null function is provided
 * for any log level, the default logging function will be used instead.
 *
 * @param error Pointer to the custom error logging function.
 * @param warn Pointer to the custom warning logging function.
 * @param info Pointer to the custom info logging function.
 * @param debug Pointer to the custom debug logging function.
 */
void set_loggers(LogFunction log_message);

/**
 * @brief Sets the default logging function for all log levels.
 *
 * This function initializes the logging function to the default
 * implementations.
 */
void set_default_loggers();

} // namespace yasmin

#endif // YASMIN__LOGS_HPP
