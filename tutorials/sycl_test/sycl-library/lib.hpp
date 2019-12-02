#include <CL/sycl.hpp>

#ifndef SYCL_EXTERNAL
#error "This application requires SYCL_EXTERNAL support
#endif

SYCL_EXTERNAL int lib_calculate(cl::sycl::global_ptr<int>,
                                cl::sycl::global_ptr<int>, int);

cl::sycl::program lib_get_program(cl::sycl::context&);
