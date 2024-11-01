// Copyright (C) 2023  Miguel Ángel González Santamarta

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

#ifndef YASMIN_STATE_HPP
#define YASMIN_STATE_HPP

#include <atomic>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <cxxabi.h>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/logs.hpp"

namespace yasmin {

class State {

protected:
  std::set<std::string> outcomes;

private:
  std::atomic_bool canceled{false};
  std::atomic_bool running{false};

public:
  State(std::set<std::string> outcomes);

  std::string operator()(std::shared_ptr<blackboard::Blackboard> blackboard);

  virtual std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) {
    (void)blackboard;
    return "";
  }

  virtual void cancel_state() {
    YASMIN_LOG_INFO("Canceling state '%s'", this->to_string().c_str());
    this->canceled.store(true);
  };
  bool is_canceled() const;
  bool is_running() const;

  std::set<std::string> const &get_outcomes();

  virtual std::string to_string() {
    std::string name = typeid(*this).name();

#ifdef __GNUG__ // if using GCC/G++
    int status;
    // demangle the name using GCC's demangling function
    char *demangled =
        abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status);
    if (status == 0) {
      name = demangled;
    }
    free(demangled);
#endif

    return name;
  }
};

} // namespace yasmin

#endif
