Embree Overview
===============

Embree is a collection of high-performance ray tracing kernels,
developed at Intel. The target user of Embree are graphics application
engineers that want to improve the performance of their application by
leveraging the optimized ray tracing kernels of Embree. The kernels are
optimized for photo-realistic rendering on the latest Intel® processors
with support for SSE, AVX, AVX2, and the 16-wide Xeon Phi™ vector
instructions. Embree supports runtime code selection to choose the
traversal and build algorithms that best matches the instruction set of
your CPU. We recommend using Embree through its API to get the highest
benefit from future improvements. Embree is released as Open Source
under the [Apache 2.0
license](http://www.apache.org/licenses/LICENSE-2.0).

Embree supports applications written with the Intel SPMD Programm
Compiler (ISPC, <https://ispc.github.io/>) by also providing an ISPC
interface to the core ray tracing algorithms. This makes it possible to
write a renderer in ISPC that leverages SSE, AVX, AVX2, and Xeon Phi™
instructions without any code change. ISPC also supports runtime code
selection, thus ISPC will select the best code path for your
application, while Embree selects the optimal code path for the ray
tracing algorithms.

Embree contains algorithms optimized for incoherent workloads (e.g.
Monte Carlo ray tracing algorithms) and coherent workloads (e.g. primary
visibility and hard shadow rays). For standard CPUs, the single-ray
traversal kernels in Embree provide the best performance for incoherent
workloads and are very easy to integrate into existing rendering
applications. For Xeon Phi™, a renderer written in ISPC using the
default hybrid ray/packet traversal algorithms have shown to perform
best, but requires writing the renderer in ISPC. In general for coherent
workloads, ISPC outperforms the single ray mode on each platform. Embree
also supports dynamic scenes by implementing high performance two-level
spatial index structure construction algorithms.

In addition to the ray tracing kernels, Embree provides some tutorials
to demonstrate how to use the [Embree API]. The example photorealistic
renderer that was originally included in the Embree kernel package is
now available in a separate GIT repository (see [Embree Example
Renderer]).

