// Copyright (C) 2024  Miguel Ángel González Santamarta
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

#ifndef YASMIN_LOGS_HPP
#define YASMIN_LOGS_HPP

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace yasmin {

/**
 * @brief Type definition for a logging function.
 *
 * This type represents a function pointer that takes a file name,
 * function name, line number, log message, and a variable number of
 * additional arguments for formatting the log message.
 *
 * @param file The name of the source file where the log function is called.
 * @param function The name of the function where the log function is called.
 * @param line The line number in the source file where the log function is
 * called.
 * @param text The format string for the log message, similar to printf.
 * @param ... Additional arguments for the format string.
 */
typedef void (*LogFunction)(const char *file, const char *function, int line,
                            const char *text, ...);

// Declare function pointers for logging at different severity levels
extern LogFunction log_error; ///< Pointer to the error logging function
extern LogFunction log_warn;  ///< Pointer to the warning logging function
extern LogFunction log_info;  ///< Pointer to the info logging function
extern LogFunction log_debug; ///< Pointer to the debug logging function

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
  yasmin::log_error(extract_filename(__FILE__), __FUNCTION__, __LINE__, text,  \
                    ##__VA_ARGS__)
#define YASMIN_LOG_WARN(text, ...)                                             \
  yasmin::log_warn(extract_filename(__FILE__), __FUNCTION__, __LINE__, text,   \
                   ##__VA_ARGS__)
#define YASMIN_LOG_INFO(text, ...)                                             \
  yasmin::log_info(extract_filename(__FILE__), __FUNCTION__, __LINE__, text,   \
                   ##__VA_ARGS__)
#define YASMIN_LOG_DEBUG(text, ...)                                            \
  yasmin::log_debug(extract_filename(__FILE__), __FUNCTION__, __LINE__, text,  \
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
void set_loggers(LogFunction error, LogFunction warn, LogFunction info,
                 LogFunction debug);

/**
 * @brief Sets the default logging functions for all log levels.
 *
 * This function initializes the logging functions to the default
 * implementations.
 */
void set_default_loggers();

} // namespace yasmin

#endif // YASMIN_LOGS_HPP
