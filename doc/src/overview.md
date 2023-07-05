Intel® Embree Overview
======================

Intel® Embree is a high-performance ray tracing library developed at
Intel, which is released as open source under the [Apache 2.0
license](http://www.apache.org/licenses/LICENSE-2.0). Intel® Embree
supports x86 CPUs under Linux, macOS, and Windows; ARM CPUs on Linux
and macOS; as well as Intel® GPUs under Linux and Windows.

Intel® Embree targets graphics application developers to improve the
performance of photo-realistic rendering applications. Embree is
optimized towards production rendering, by putting focus on incoherent
ray performance, high quality acceleration structure construction, a
rich feature set, accurate primitive intersection, and low memory
consumption.

Embree's feature set includes various primitive types such as
triangles (as well quad and grids for lower memory consumption);
Catmull-Clark subdivision surfaces; various types of curve primitives,
such as flat curves (for distant views), round curves (for closeup
views), and normal oriented curves, all supported with different basis
functions (linear, Bézier, B-spline, Hermite, and Catmull Rom);
point-like primitives, such as ray oriented discs, normal oriented
discs, and spheres; user defined geometries with a procedural
intersection function; multi-level instancing; filter callbacks
invoked for any hit encountered; motion blur including multi-segment
motion blur, deformation blur, and quaternion motion blur; and ray
masking.

Intel® Embree contains ray tracing kernels optimized for the latest
x86 processors with support for SSE, AVX, AVX2, and AVX-512
instructions, and uses runtime code selection to choose between these
kernels. Intel® Embree contains algorithms optimized for incoherent
workloads (e.g.  Monte Carlo ray tracing algorithms) and coherent
workloads (e.g. primary visibility and hard shadow rays) as well as
supports for dynamic scenes by implementing high-performance two-level
spatial index structure construction algorithms.

Intel® Embree supports applications written with the Intel® Implicit
SPMD Program Compiler (Intel® ISPC, <https://ispc.github.io/>) by
providing an ISPC interface to the core ray tracing
algorithms. This makes it possible to write a renderer that
automatically vectorizes and leverages SSE, AVX, AVX2, and AVX-512
instructions.

Intel® Embree supports Intel GPUs through the
[SYCL](https://www.khronos.org/sycl/) open standard programming
language. SYCL allows to write C++ code that can be run on various
devices, such as CPUs and GPUs. Using Intel® Embree application
developers can write a single source renderer that executes
efficiently on CPUs and GPUs. Maintaining just one code base
this way can significantly improve productivity and eliminate
inconsistencies between a CPU and GPU version of the renderer. Embree
supports GPUs based on the Xe HPG and Xe HPC microarchitecture,
which support hardware accelerated ray tracing do deliver excellent
levels of ray tracing performance.

