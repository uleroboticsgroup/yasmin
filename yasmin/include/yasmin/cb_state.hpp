
#ifndef YASMIN_CB_STATE_HPP
#define YASMIN_CB_STATE_HPP

#include <string>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"

namespace yasmin {

class CbState : public State {

private:
  std::string (*callback)(std::shared_ptr<blackboard::Blackboard> blackboard);

public:
  CbState(std::vector<std::string> outcomes,
          std::string (*callback)(
              std::shared_ptr<blackboard::Blackboard> blackboard));

  std::string
  execute(std::shared_ptr<blackboard::Blackboard> blackboard) override;

  std::string to_string();
};

} // namespace yasmin

#endif
