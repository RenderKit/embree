#include <pybind11/pybind11.h>
#include <iostream>


int add(int i, int j) {
    std::cout << "AOUEAOEUAOEUAOUE" << std::endl;
    return i + j;
}

PYBIND11_MODULE(pyembree, m) {
    m.def("add", &add);
}