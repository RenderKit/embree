# sycl-library

This simple project provides an example how to create a shared library that
contains SYCL device code which can be used by another SYCL device code from an
application which is linked with this shared library.

Basically, I was able to create such app and library and everything is even
works correctly on CPU (haven't tested on GPU yet), but strictly speaking, SYCL
spec doesn't guarantee that this code will work correctly. See section with
issues below.

Project structure:

* `main.cpp` contains application host code and one SYCL kernel that calls two
  payload functions: one is defined in the same translation unit and another one
  is expected to be linked from shared library containing some device code
* `lib.cpp` contains library host code and function marked with `SYCL_EXTERNAL`

Host application compiles a program, then extracts another compiled program from
a shared library, links them together and launches kernel which calls function
defined in the shared library.

### How to build

```
mkdir build && cd build/
cmake -DCMAKE_CXX_COMPILER=/path/to/sycl/clang++ ..
make
```

### How to launch

```
./myapp
```

Expected output is:

```
Result: 0
```

## Issues

### It is unclear how to compile a device code without kernels

For example, there is `lib_calculate` function in `lib.cpp` which is marked as
`SYCL_EXTERNAL` and intended to be the only thing which is available in device
code of shared library.

The question is how to get `cl::sycl::program` object in
`program_state::compiled` state which contains `lib_calculate` function?

Note: compiled state is required for subsequent linkage with device code from
an application.

How can we do that?

1. Create SYCL program from compiled OpenCL program.

   **Not applicable**. We are using SYCL, not OpenCL, right?

2. Compile SYCL program from OpenCL C string using `compile_with_source`

   **Not applicable**. We are using SYCL, not OpenCL, right?

3. Use `compile_with_kernel_type` method on SYCL program.

   According to the spec, this method will compiler SYCL kernel function which
   name is provided as template argument. I guess if this kernel function calls
   something else then this something else will also be compiled.

   Having that said, to get all helper functions compiled we need to call them
   all from some dummy kernel.

   **Ugly**

4. Anything else?

Right now in this example, helper function is not even called from the kernel
which is being compiled, but everything works due to implementation details of
our compiler.

### Is there an easy way to get program object containing several kernels?

The problem here is that by default, `parallel_for` uses program instance which
was implicitly created and compiled by SYCL RT with some default options.

In cases, where user wants to change this options, or if the program object is
required to do some additional stuff (for example, to request function pointer),
user need to pass `cl::sycl::kernel` object explicitly to each `parallel_for` to
enforce using specific program object instead of implicitly created one.

SYCL spec doesn't allow to do this easily: `compile_with_kernel_type` method
mentioned above only guarantees that specified kernel will be included into a
program object and nothing is said about the rest.

This means that to get SYCL program with several kernels, we basically need to
create several program objects, compile all of them for each kernel and then
link them together to get executable device binary: not very user-friendly.
