
#include <string>

#include "yasmin/blackboard/blackboard.hpp"
#include "yasmin/state.hpp"

#include "yasmin/cb_state.hpp"

using namespace yasmin;

CbState::CbState(
    std::vector<std::string> outcomes,
    std::string (*callback)(std::shared_ptr<blackboard::Blackboard> blackboard))
    : State(outcomes) {
  this->callback = callback;
}

std::string
CbState::execute(std::shared_ptr<blackboard::Blackboard> blackboard) {
  return this->callback(blackboard);
}

std::string CbState::to_string() { return "CbState"; }