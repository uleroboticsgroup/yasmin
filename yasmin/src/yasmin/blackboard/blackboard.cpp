
#include <map>
#include <string>

#include "yasmin/blackboard/blackboard.hpp"

using namespace yasmin::blackboard;

Blackboard::Blackboard() {}

Blackboard::Blackboard(const Blackboard &other) {
  for (const auto &ele : other.values) {
    this->values.insert({ele.first, ele.second});
  }
}

Blackboard::~Blackboard() {
  for (const auto &ele : this->values) {
    delete this->values.at(ele.first);
  }
}

bool Blackboard::contains(std::string name) {
  return (this->values.find(name) != this->values.end());
}

std::string Blackboard::to_string() {

  std::string result = "Blackboard\n";

  for (const auto &ele : this->values) {
    result += "\t" + ele.first + " (" + ele.second->to_string() + ")\n";
  }

  return result;
}
