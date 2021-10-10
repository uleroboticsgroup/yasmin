
#ifndef YASMIN_BLACKBOARD_VAL_HPP
#define YASMIN_BLACKBOARD_VAL_HPP

#include <string>

#include <boost/type_index.hpp>

#include "yasmin/blackboard/blackboard_value_interface.hpp"

namespace yasmin {
namespace blackboard {

template <class T> class BlackboardValue : public BlackboardValueInterface {
private:
  T value;

public:
  BlackboardValue(T value) { this->value = value; }

  T get() { return this->value; }
  void set(T value) { this->value = value; }

  std::string get_type() {
    return boost::typeindex::type_id<T>().pretty_name();
  }
  std::string to_string() { return this->get_type(); }
};

} // namespace blackboard
} // namespace yasmin

#endif
