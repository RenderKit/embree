#include "lib.hpp"

SYCL_EXTERNAL int lib_calculate(cl::sycl::global_ptr<int> A,
                                cl::sycl::global_ptr<int> B, int Size) {
  int Sum = 0;

  for (int I = 0; I < Size; ++I) {
    Sum += A[I] + B[I];
  }

  return Sum;
}

cl::sycl::program lib_get_program(cl::sycl::context &C) {
  auto Devices = C.get_devices();
  assert(Devices.size() == 1);

  auto EH = [](cl::sycl::exception_list EL) {
    for (std::exception_ptr const &E : EL) {
      try {
        std::rethrow_exception(E);
      } catch (cl::sycl::exception const &E) {
        std::cout << "Caught asynchronous SYCL exception: \n"
                  << E.what() << std::endl;
      }
    }
  };

  cl::sycl::queue Q(Devices.front(), EH);
  Q.submit([&](cl::sycl::handler &CGH) {
    CGH.single_task<class dummy>([=]() { });
  });

  Q.wait_and_throw(); // to get build logs

  cl::sycl::program P(C);
  // FIXME: there is not guarantee (according to the SYCL spec), that
  // 'lib_calculate' will be included into 'P'
  P.compile_with_kernel_type<class dummy>();

  return P;
}
