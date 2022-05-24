
#ifndef YASMIN_STATE_HPP
#define YASMIN_STATE_HPP

#include <memory>
#include <string>
#include <vector>

#include "yasmin/blackboard/blackboard.hpp"

namespace yasmin {

class State {

protected:
  std::vector<std::string> outcomes;

private:
  bool canceled;

public:
  State(std::vector<std::string> outcomes);

  std::string operator()(std::shared_ptr<blackboard::Blackboard> blackboard);

  virtual std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) {
    (void)blackboard;
    return "";
  }

  void cancel_state();
  bool is_canceled();

  std::vector<std::string> const &get_outcomes();

  virtual std::string to_string() { return "State"; }
};

} // namespace yasmin

#endif
