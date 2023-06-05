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

#ifndef YASMIN_STATE_MACHINE_HPP
#define YASMIN_STATE_MACHINE_HPP

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin {

class StateMachine : public State {

public:
  StateMachine(std::vector<std::string> outcomes);

  void add_state(std::string name, std::shared_ptr<State> state,
                 std::map<std::string, std::string> transitions);
  void add_state(std::string name, std::shared_ptr<State> state);

  void set_start_state(std::string state_name);
  std::string get_start_state();

  void cancel_state();

  std::map<std::string, std::shared_ptr<State>> const &get_states();
  std::map<std::string, std::map<std::string, std::string>> const &
  get_transitions();
  std::string get_current_state();

  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override;
  std::string execute();
  std::string operator()();
  using State::operator();

  std::string to_string();

private:
  std::map<std::string, std::shared_ptr<State>> states;
  std::map<std::string, std::map<std::string, std::string>> transitions;
  std::string start_state;
  std::string current_state;
  std::unique_ptr<std::mutex> current_state_mutex;
};

} // namespace yasmin

#endif
