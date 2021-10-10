
#ifndef YASMIN_BLACKBOARD_HPP
#define YASMIN_BLACKBOARD_HPP

#include <map>
#include <string>

#include "yasmin/blackboard/blackboard_value.hpp"
#include "yasmin/blackboard/blackboard_value_interface.hpp"

namespace yasmin {
namespace blackboard {

class Blackboard {

private:
  std::map<std::string, BlackboardValueInterface *> values;

public:
  Blackboard();
  Blackboard(const Blackboard &other);
  ~Blackboard();

  template <class T> void set(std::string name, T value) {
    if (!this->contains(name)) {
      BlackboardValue<T> *b_value = new BlackboardValue<T>(value);
      this->values.insert({name, b_value});

    } else {
      ((BlackboardValue<T> *)this->values.at(name))->set(value);
    }
  }

  template <class T> T get(std::string name) {
    if (!this->contains(name)) {
      throw "Element " + name + " does not exist in the blackboard";
    }

    BlackboardValue<T> *b_value = (BlackboardValue<T> *)this->values.at(name);
    return b_value->get();
  }

  bool contains(std::string name);

  std::string to_string();
};

} // namespace blackboard
} // namespace yasmin

#endif
