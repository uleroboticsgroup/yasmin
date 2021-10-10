
#ifndef YASMIN_BLACKBOARD_VAL_IFACE_HPP
#define YASMIN_BLACKBOARD_VAL_IFACE_HPP

#include <string>

namespace yasmin {
namespace blackboard {

class BlackboardValueInterface {
public:
  virtual ~BlackboardValueInterface(){};
  virtual std::string to_string() { return ""; };
};

} // namespace blackboard
} // namespace yasmin

#endif
