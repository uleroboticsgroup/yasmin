// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#ifndef YASMIN__DEMANGLE_HPP_
#define YASMIN__DEMANGLE_HPP_

#include <cxxabi.h>

#include <cstdlib>
#include <list>
#include <mutex>
#include <string>
#include <unordered_map>

namespace yasmin {

/**
 * @brief Demangles a C++ type name.
 *
 * This function takes a mangled C++ type name and returns its demangled
 * representation. It uses an LRU cache to store previously demangled names
 * for improved performance.
 *
 * @param mangled_name The mangled C++ type name to be demangled.
 * @return The demangled C++ type name, or the original mangled name if
 *         demangling fails.
 */
inline std::string demangle_type(const std::string &mangled_name) {

  // LRU cache size limit; 1024 entries balances memory usage against type
  // diversity in typical apps
  static constexpr size_t kMaxCacheSize = 1024;

  static std::unordered_map<
      std::string, std::pair<std::string, std::list<std::string>::iterator>>
      cache;
  static std::list<std::string> lru;
  static std::mutex cache_mutex;

  {
    std::lock_guard<std::mutex> lock(cache_mutex);
    auto it = cache.find(mangled_name);
    if (it != cache.end()) {
      lru.splice(lru.end(), lru, it->second.second);
      return it->second.first;
    }
  }

#ifdef __GNUG__
  int status;
  char *demangled =
      abi::__cxa_demangle(mangled_name.c_str(), nullptr, nullptr, &status);
  std::string result;
  if (status == 0) {
    result = demangled;
  } else {
    result = mangled_name;
  }
  std::free(demangled);
#else
  std::string result = mangled_name;
#endif

  {
    std::lock_guard<std::mutex> lock(cache_mutex);

    auto it = cache.find(mangled_name);
    if (it != cache.end()) {
      lru.splice(lru.end(), lru, it->second.second);
      return it->second.first;
    }

    if (cache.size() >= kMaxCacheSize) {
      cache.erase(lru.front());
      lru.pop_front();
    }

    lru.push_back(mangled_name);
    auto li = std::prev(lru.end());
    cache[mangled_name] = {result, li};
  }

  return result;
}

} // namespace yasmin

#endif // YASMIN__DEMANGLE_HPP_
