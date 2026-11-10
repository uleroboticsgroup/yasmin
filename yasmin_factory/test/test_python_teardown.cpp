// Regression test: exiting after constructing a YasminFactory used to segfault
// in pybind11::finalize_interpreter() (use-after-free of pybind11 internals).
#include "yasmin_factory/yasmin_factory.hpp"

int main() { yasmin_factory::YasminFactory factory; }
