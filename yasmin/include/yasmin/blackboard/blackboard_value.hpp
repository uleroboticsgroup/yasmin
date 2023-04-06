
#ifndef YASMIN_BLACKBOARD_VAL_HPP
#define YASMIN_BLACKBOARD_VAL_HPP

#include <string>
#include <typeinfo>

#ifdef __GNUG__     // if using GCC/G++
#include <cxxabi.h> // for abi::__cxa_demangle
#endif

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
    std::string name = typeid(T).name(); // get the mangled name of the type

#ifdef __GNUG__ // if using GCC/G++
    int status;
    // demangle the name using GCC's demangling function
    char *demangled =
        abi::__cxa_demangle(name.c_str(), nullptr, nullptr, &status);
    if (status == 0) {
      name = demangled;
    }
    free(demangled);
#endif

    return name;
  }
  std::string to_string() { return this->get_type(); }
};

} // namespace blackboard
} // namespace yasmin

#endif
