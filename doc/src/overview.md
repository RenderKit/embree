Embree Overview
===============

Intel® Embree is a collection of high-performance ray tracing kernels,
developed at Intel. The target users of Intel® Embree are graphics application
engineers who want to improve the performance of their
photo-realistic rendering application by leveraging Embree's
performance-optimized ray tracing kernels. The kernels are optimized
for the latest Intel® processors with support for SSE, AVX, AVX2, and
AVX-512 instructions. Intel® Embree supports runtime code selection to choose
the traversal and build algorithms that best matches the instruction
set of your CPU. We recommend using Intel® Embree through its API to get the
highest benefit from future improvements. Intel® Embree is released as Open
Source under the
[Apache 2.0 license](http://www.apache.org/licenses/LICENSE-2.0).

Intel® Embree supports applications written with the Intel® SPMD Program
Compiler (ISPC, <https://ispc.github.io/>) by also providing an ISPC
interface to the core ray tracing algorithms. This makes it possible
to write a renderer in ISPC that automatically vectorizes and
leverages SSE, AVX, AVX2, and AVX-512 instructions. ISPC also supports
runtime code selection, thus ISPC will select the best code path for
your application.

Intel® Embree contains algorithms optimized for incoherent workloads (e.g.
Monte Carlo ray tracing algorithms) and coherent workloads
(e.g. primary visibility and hard shadow rays).

The single-ray traversal kernels of Intel® Embree provide high performance
for incoherent workloads and are very easy to integrate into existing
rendering applications. Using the stream kernels, even higher
performance for incoherent rays is possible, but integration might
require significant code changes to the application to use the stream
paradigm. In general for coherent workloads, the stream mode with
coherent flag set gives the best performance.

Intel® Embree also supports dynamic scenes by implementing high-performance
two-level spatial index structure construction algorithms.

In addition to the ray tracing kernels, Intel® Embree provides some
[Embree Tutorials] to demonstrate how to use the
[Embree API].

