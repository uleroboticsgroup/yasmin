
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
