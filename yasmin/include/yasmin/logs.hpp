// Copyright (C) 2024  Miguel Ángel González Santamarta

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef YASMIN_LOGS_HPP
#define YASMIN_LOGS_HPP

#include <cstdarg>
#include <cstdio>

namespace yasmin {

// Define raw function pointer type for logging with file and function
// parameters
typedef void (*LogFunction)(const char *, const char *, int, const char *, ...);

// Declare function pointers as extern
extern LogFunction log_error;
extern LogFunction log_warn;
extern LogFunction log_info;
extern LogFunction log_debug;

// Macros to use the function pointers for logging, passing file and function
#define YASMIN_LOG_ERROR(text, ...)                                            \
  yasmin::log_error(__FILE__, __FUNCTION__, __LINE__, text, ##__VA_ARGS__)
#define YASMIN_LOG_WARN(text, ...)                                             \
  yasmin::log_warn(__FILE__, __FUNCTION__, __LINE__, text, ##__VA_ARGS__)
#define YASMIN_LOG_INFO(text, ...)                                             \
  yasmin::log_info(__FILE__, __FUNCTION__, __LINE__, text, ##__VA_ARGS__)
#define YASMIN_LOG_DEBUG(text, ...)                                            \
  yasmin::log_debug(__FILE__, __FUNCTION__, __LINE__, text, ##__VA_ARGS__)

// Function to set custom log functions
void set_loggers(LogFunction error, LogFunction warn, LogFunction info,
                 LogFunction debug);

void set_default_loggers();
} // namespace yasmin

#endif // YASMIN_LOGS_HPP
