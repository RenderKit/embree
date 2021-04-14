Supported Platforms
-------------------

Embree supports Windows (32-bit and 64-bit), Linux (64-bit), and macOS
(64-bit) both x86 and Apple M1 based. The code compiles with the Intel®
Compiler, GCC, Clang, and the Microsoft Compiler.

Using the Intel® Compiler improves performance by approximately
10%. Performance also varies across different operating
systems, with Linux typically performing best as it supports
transparently transitioning to 2MB pages.

Embree is optimized for Intel CPUs supporting SSE, AVX, AVX2, and
AVX-512 instructions. Embree requires at least an x86 CPU with support for
SSE2 or an Apple M1 CPU.
