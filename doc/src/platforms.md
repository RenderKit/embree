Supported Platforms
-------------------

Embree supports Windows (32\ bit and 64\ bit), Linux (64\ bit) and Mac
OS\ X (64\ bit). The code compiles with the Intel Compiler, GCC, CLANG
and the Microsoft Compiler. Embree is tested with Intel
Compiler 15.0.0, CLANG 3.4.2, GCC 4.8.2, and Visual Studio
12 2013. Using the Intel Compiler improves performance by
approximately 10%.

Performance also varies across different operating systems. Embree is
optimized for Intel CPUs supporting SSE, AVX, and AVX2 instructions,
and requires at least a CPU with support for SSE2.

The Xeon Phi™ version of Embree only works under Linux in 64\ bit mode.
For compilation of the the Xeon Phi™ code the Intel Compiler is
required. The host side code compiles with GCC, CLANG, and the Intel
Compiler.

