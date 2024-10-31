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

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin {

class StateMachine : public State {

  using StartCallbackType = std::function<void(
      std::shared_ptr<yasmin::blackboard::Blackboard>, const std::string &,
      const std::vector<std::string> &)>;
  using TransitionCallbackType = std::function<void(
      std::shared_ptr<yasmin::blackboard::Blackboard>, const std::string &,
      const std::string &, const std::string &,
      const std::vector<std::string> &)>;
  using EndCallbackType = std::function<void(
      std::shared_ptr<yasmin::blackboard::Blackboard>, const std::string &,
      const std::vector<std::string> &)>;

public:
  StateMachine(std::set<std::string> outcomes);

  void add_state(std::string name, std::shared_ptr<State> state,
                 std::map<std::string, std::string> transitions);
  void add_state(std::string name, std::shared_ptr<State> state);

  void set_start_state(std::string state_name);
  std::string get_start_state();

  std::map<std::string, std::shared_ptr<State>> const &get_states();
  std::map<std::string, std::map<std::string, std::string>> const &
  get_transitions();
  std::string get_current_state();

  void add_start_cb(StartCallbackType cb, std::vector<std::string> args = {});
  void add_transition_cb(TransitionCallbackType cb,
                         std::vector<std::string> args = {});
  void add_end_cb(EndCallbackType cb, std::vector<std::string> args = {});
  void
  call_start_cbs(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                 const std::string &start_state);
  void call_transition_cbs(
      std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
      const std::string &from_state, const std::string &to_state,
      const std::string &outcome);
  void call_end_cbs(std::shared_ptr<yasmin::blackboard::Blackboard> blackboard,
                    const std::string &outcome);

  void validate();
  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override;
  std::string execute();
  std::string operator()();
  using State::operator();
  void cancel_state() override;

  std::string to_string();

private:
  std::map<std::string, std::shared_ptr<State>> states;
  std::map<std::string, std::map<std::string, std::string>> transitions;
  std::string start_state;
  std::string current_state;
  std::unique_ptr<std::mutex> current_state_mutex;

  std::vector<std::pair<StartCallbackType, std::vector<std::string>>> start_cbs;
  std::vector<std::pair<TransitionCallbackType, std::vector<std::string>>>
      transition_cbs;
  std::vector<std::pair<EndCallbackType, std::vector<std::string>>> end_cbs;
};

} // namespace yasmin

#endif
