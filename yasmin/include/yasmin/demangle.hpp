// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#ifndef YASMIN__DEMANGLE_HPP_
#define YASMIN__DEMANGLE_HPP_

#include <cxxabi.h>

#include <cstdlib>
#include <list>
#include <mutex>
#include <string>
#include <unordered_map>

namespace yasmin {

inline std::string demangle_type(const std::string &mangled_name) {

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
