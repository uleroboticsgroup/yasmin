// Exiting used to segfault in pybind11::finalize_interpreter().
#include "yasmin_factory/yasmin_factory.hpp"

int main() { yasmin_factory::YasminFactory factory; }
