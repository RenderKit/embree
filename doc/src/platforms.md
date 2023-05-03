Supported Platforms
-------------------

Embree supports Windows (32-bit and 64-bit), Linux (64-bit), and macOS
(64-bit). Under Windows, Linux and macOS x86 based CPUs are supported,
while ARM CPUs are currently only supported under Linux and macOS (e.g. 
Apple M1). ARM support for Windows experimental.

Embree supports Intel GPUs based on the Xe HPG microarchitecture
(Intel® Arc™ GPU) under Linux and Windows and Xe HPC microarchitecture
(Intel® Data Center GPU Flex Series and Intel® Data Center GPU Max
Series) under Linux.

The code compiles with the Intel® Compiler, Intel® oneAPI DPC++
Compiler, GCC, Clang, and the Microsoft Compiler. To use Embree on the
GPU the Intel® oneAPI DPC++ Compiler must be used. Please see section
[Compiling Embree] for details on tested compiler versions.

Embree requires at least an x86 CPU with support for
SSE2 or an Apple M1 CPU.

