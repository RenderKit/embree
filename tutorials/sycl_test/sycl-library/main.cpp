#include <CL/sycl.hpp>

#include "lib.hpp"

#include <vector>
#include <iostream>

// Uncomment line below to enable some output from kernel
// #define DEBUG_OUTPUT

int app_calculate(cl::sycl::global_ptr<int> A, cl::sycl::global_ptr<int> B,
                  int Size) {
  int Sum = 0;

  for (int I = 0; I < Size; ++I) {
    Sum += A[I] + B[I];
  }

  return Sum;
}

int main() {
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

  cl::sycl::queue Q(EH);
  cl::sycl::context C = Q.get_context();
  cl::sycl::program AppProg(C);

  AppProg.compile_with_kernel_type<class app_kernel>();
  auto LibProg = lib_get_program(C);

  cl::sycl::program P( { AppProg, LibProg} );
  // According to the spec, P is already linked, no need to explicitly call link
  // P.link();

  const int N = 4;
  const cl::sycl::range<1> GlobalSize { 8 };
  const cl::sycl::range<1> LocalSize { 8 };

  std::vector<int> A(GlobalSize.get(0) * N, 1);
  std::vector<int> B(GlobalSize.get(0) * N, 2);

  cl::sycl::buffer<int> BufA(A.data(), { A.size() });
  cl::sycl::buffer<int> BufB(B.data(), { B.size() });

  int R = 0;
  cl::sycl::buffer<int> BufR(&R, 1);

  // We need to explicitly create a kernel and pass it to 'parallel_for',
  // otherwise, SYCL RT will implicitly create a different program and use it
  // instead of user-created one.
  cl::sycl::kernel K = P.get_kernel<class app_kernel>();
  Q.submit([&](cl::sycl::handler &CGH) {
    auto AccA = BufA.template get_access<cl::sycl::access::mode::read>(CGH);
    auto AccB = BufB.template get_access<cl::sycl::access::mode::read>(CGH);
    auto AccR = BufR.template get_access<cl::sycl::access::mode::atomic>(CGH);
    auto AccL = cl::sycl::accessor<int, 0, cl::sycl::access::mode::atomic,
                                   cl::sycl::access::target::local>(CGH);
    cl::sycl::stream Out(4096, 256, CGH);

    // Without argument K passed to parallel_for, device build (JIT) will fail
    // with something like:
    // The program was built for 1 devices
    //  Build program log for 'Intel(R) Gen9 HD Graphics NEO':
    // error: undefined reference to `_Z13lib_calculateN2cl4sycl9multi_ptrIiLNS0_6access13address_spaceE1EEES4_i()'
    // error: backend compiler failed build.
    // 0 (CL_SUCCESS)
    CGH.parallel_for<class app_kernel>(/* THIS IS AN IMPORTANT ARGUMENT: */K,
        cl::sycl::nd_range<1>{ GlobalSize, LocalSize },
        [=](cl::sycl::nd_item<1> ND) {
      auto LID = ND.get_local_id(0);
      cl::sycl::atomic<int, cl::sycl::access::address_space::local_space> A =
          AccL;

#ifdef DEBUG_OUTPUT
// Macro below provides a work-around to ensure that output to stream from
// different work-items will not interfiere. NOTE: this doesn't help to fix
// races between different threads (i.e. work-groups on CPU)
#define PRINT(X)                                                               \
  {                                                                            \
    for (int I = 0; I < LocalSize.get(0); ++I) {                               \
      if (I == LID) {                                                          \
        X                                                                      \
      }                                                                        \
    }                                                                          \
  }
#else
#define PRINT(X)
#endif // DEBUG_OUTPUT

      if (LID == 0) {
        A.store(0);
        PRINT(Out << "LID = " << LID << " A = " << A.load() << cl::sycl::endl;)
      }
      ND.barrier(cl::sycl::access::fence_space::local_space);

      auto GID = ND.get_group(0);

      auto Offset = GID * LocalSize.get(0) * N + LID * N;

      A.fetch_add(app_calculate(AccA.get_pointer() + Offset,
                                AccB.get_pointer() + Offset, N));
      PRINT(Out << "LID = " << LID << " A = " << A.load() << cl::sycl::endl;)

      A.fetch_sub(lib_calculate(AccA.get_pointer() + Offset,
                                AccB.get_pointer() + Offset, N));
      PRINT(Out << "LID = " << LID << " A = " << A.load() << cl::sycl::endl;)

      ND.barrier(cl::sycl::access::fence_space::local_space);
      AccR[0].fetch_add(A.load());
    });
  });

  Q.wait_and_throw();

  auto AccR = BufR.template get_access<cl::sycl::access::mode::read>();
  std::cout << "Result: " << AccR[0] << std::endl;

  return 0;
}
