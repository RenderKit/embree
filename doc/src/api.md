Embree API
==========

``` {include=src/intro.md}
```

Embree DPC++ API
================

Embree supports Xe HPG and Xe HPC GPUs by using DPC++, which is
Intel's extension of the SYCL programming language. SYCL is a Khronos
standardized C++ based programming language for single source
heterogenous programming for acceleration offload, see
the [SYCL webpage](https://www.khronos.org/sycl/) for details.

The Embree DPC++ API is designed for photorealistic rendering use
cases, where scene setup is performed on the host, and rendering on
the device. The Embree DPC++ API is very similar to the standard
Embree C99 API. To enable SYCL support you have to include the
`sycl.hpp` file before the Embree API headers:

    #include <CL/sycl.hpp>
    #include <embree4/rtcore.h>

Next you need to initializes a DPC++ Embree device using the
`rtcNewSYCLDevice` API function by providing a SYCL context and
queue.

Embree provides the `rtcIsSYCLDeviceSupported` API function to check
if some SYCL device is supported by Embree. You can also use the
`RTCDeviceSelector` to conveniently select the first SYCL device that
is supported by Embree, e.g.:

    sycl::device device(RTCDeviceSelector());
    sycl::queue queue(device, exception_handler);
    sycl::context context(queue.get_context());
    RTCDevice device = rtcNewSYCLDevice(&context,&queue,"");

Files containing SYCL code, have to get compiled with the
[Intel(R) oneAPI DPC++
compiler](https://www.intel.com/content/www/us/en/developer/articles/tool/oneapi-standalone-components.html#dpcpp-cpp)
or open source [oneAPI DPC++
compiler](https://github.com/intel/llvm/). The DPC++ compiler performs
a two-phase compilation, where host code is compiled in a first phase,
and device code compiled in a second compilation phase.

Standard Embree API functions for scene construction can get used on
the host (but not the device). Data buffers that are shared with
Embree (e.g. for vertex of index buffers) have to get allocated as
SYCL unified shared memory (USM memory), using the `sycl::malloc` or
`sycl::aligned_alloc` calls, with `sycl::usm::alloc::shared` property,
e.g:

    void* ptr = sycl::aligned_alloc(16, bytes, queue, sycl::usm::alloc::shared);

These shared allocations have to be valid during rendering, as Embree
may access contained data when tracing rays. Embree does not support
device-only memory allocations, as the BVH builder implemented on the
CPU relies on reading the same data buffers.

Device side rendering can then get invoked by submitting a SYCL
`parallel_for` to the SYCL queue:


    queue.submit([=](sycl::handler& cgh)
    {
      cgh.parallel_for(sycl::range<1>(1),[=](sycl::id<1> item) RTC_SYCL_KERNEL
      {
        struct RTCIntersectContext context;
        rtcInitIntersectContext(&context);
  
        struct RTCRayHit rayhit;
        rayhit.ray.org_x = ox;
        rayhit.ray.org_y = oy;
        rayhit.ray.org_z = oz;
        rayhit.ray.dir_x = dx;
        rayhit.ray.dir_y = dy;
        rayhit.ray.dir_z = dz;
        rayhit.ray.tnear = 0;
        rayhit.ray.tfar = std::numeric_limits<float>::infinity();
        rayhit.ray.mask = -1;
        rayhit.ray.flags = 0;
        rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rayhit.hit.instID[0] = RTC_INVALID_GEOMETRY_ID;
  
        rtcIntersect1(scene, &context, &rayhit);
  
        result->geomID = rayhit.hit.geomID;
        result->primID = rayhit.hit.primID;
        result->tfar = rayhit.ray.tfar;
      });
    });
    queue.wait_and_throw();

Inside the SYCL `parallel_for` you can use rendering related functions,
such as `rtcIntersect1` and `rtcOccluded1` functions to trace rays,
`rtcForwardIntersect1` and `rtcForwardOccluded` to continue object
traversal from inside a user geometry callback,
`rtcGetGeometryUserData` to get the user data pointer of some
geometry. All functions that are allowed to be used during device side
rendering are marked in the API reference.

The `RTC_SYCL_KERNEL` kernel attribute is required for each kernel
that invokes ray traversal.

Have a look at the [Minimal] tutorial for a minimal SYCL example.


DPC++ JIT caching
-----------------

While DPC++ allows precompiling device side code ahead of time (AOT
compilation), the use of specialization constants and new hardware may
trigger just in time compilation (JIT compilation). As compile times
can be large we recommend enabling persistent JIT compilation caching
inside your application, by setting the `SYCL_CACHE_PERSISTENT`
environment variable to `1`, and the `SYCL_CACHE_DIR` environment
variable to some proper directory where the JIT cache should get
stored. These environment variables have to get set before the Embree
DPC++/SYCL device is created.

    setenv("SYCL_CACHE_PERSISTENT","1",1);
    setenv("SYCL_CACHE_DIR","cache_dir",1);


DPC++ Memory Pooling
--------------------

Memory Pooling is a mechanism where small USM memory allocations are
packed into larger allocation blocks. This mode is required when your
application performs many small USM allocations, as otherwise only a
small fraction of GPU memory is usable.

Embree currently also relies on memory pooling to be enabled, but this
restriction will get fixed.

Memory pooling can get enabled by setting the
`SYCL_PI_LEVEL_ZERO_USM_ALLOCATOR` environment variable **before** the
SYCL device creation, e.g.:

    setenv("SYCL_PI_LEVEL_ZERO_USM_ALLOCATOR","1;0;shared:64K,0,2M",1);

Please see the documentation of that environment variable for details
on that feature
(https://github.com/intel/llvm/blob/sycl/sycl/doc/EnvironmentVariables.md).


Embree DPC++ Limitations
------------------------

Embree only supports Intel Xe HPC/HPG GPUs as DPC++ devices, thus in
particular the CPU and other GPUs cannot get used as a DPC++
device. To render on the CPU just use the standard C99 API without
relying on DPC++.

The SYCL language spec puts some language restrictions to device
functions, such as disallowing: malloc, invokation of virtual
functions, function pointers, runtime type information, exceptions,
recursion, etc. See Section `5.4. Language Restrictions for device
functions` of the [SYCL
specification](https://www.khronos.org/registry/SYCL/specs/sycl-2020/html/sycl-2020.html#sec:language.restrictions.kernels)
for more details. In DPC++ invoking a function through a function
pointer is allowed, but we do not recommend this for performance
reasons.

Some features have not been ported to the Embree DPC++ API thus cannot
get used on the device:

- The packet tracing functions `rtcIntersect4/8/16` and
  `rtcOccluded4/8/16`, as well as stream tracing functions
  `rtcIntersect1M`, `rtcIntersect1Mp`, `rtcIntersectNM`,
  `rtcIntersectNp`, `rtcOccluded1M`, `rtcOccluded1Mp`,
  `rtcOccludedNM`, and `rtcOccludedNp` are not supported in DPC++
  device side code. Using these functions make no sense for SYCL, as
  the programming model is implicitely executed in SIMT mode on the
  GPU.

- The `rtcInterpolate` function cannot get used on the the device. For
  most primitive types the vertex data interpolation is anyway a
  trivial operation, and an API call just introduces overheads. On the
  CPU that overhead is acceptable, but on the GPU it is not. The
  `rtcInterpolate` function does not know the geometry type it is
  interpolating over, thus its implementation on the GPU would contain
  a large switch statement for all potential geometry types.

- Subdivision surfaces are not supported for Embree DPC++ devices.

- Collision detection (`rtcCollide` API call) is not supported in DPC++ device side code.

- Point queries (`rtcPointQuery` API call) are not supported in DPC++ device side code.


Embree DPC++/SYCL Known Issues
------------------------------

There are some known DPC++ and driver issues:

- Ahead of time compilation (AOT compilation) does currently only work
  for a single device, thus multiple devices cannot get specified with
  the `EMBREE_DPCPP_AOT_DEVICES` CMake option.

- The function pointer types `RTCFilterFunctionN` for the filter
  function callback as well as `RTCIntersectFunctionN` and
  `RTCOccludedFunctionN` for the user geometry callbacks are defined
  using a `const void*` input instead a pointer to the proper
  arguments struct. This is a temporary workaround for some DPC++
  compiler issue that prevents inlining of function pointers passed
  via the `RTCIntersectArguments` struct to ray intersection.

- Under Windows the GPU side timers are currently not working with the
  oneAPI DPC++ compiler 165 from 2022.03.10. Thus under Windows the
  `--benchmark` mode of the tutorials uses host timers.

- Embree does not yet properly uses global SYCL pointers, which
  requires using the `-cl-intel-force-global-mem-allocation` and
  `-cl-intel-no-local-to-generic` option when compiling Embree DPC++
  application, see section [Building Embree DPC++ Applications]. If
  this compile option is not used, you get some compile warning that
  generic address space is used, which will significantly reduce
  performance.


Upgrading from Embree 3 to Embree 4
===================================

This section summarizes API changes between Embree 3 and Embree4. Most
of these changes are motivated by having a consistent API that works
properly for the CPU and GPU.

- User geometries callbacks get an valid vector as input to identify
  valid and invalid rays. In Embree 3 the user geometry callback just
  had to update the ray hit members when an intersection was found and
  perform no operation otherwise. In Embree 4 the callback
  additionally has to return valid=-1 when a hit was found, and
  valid=0 when no hit was found. This allows Embree to properly pass
  the new hit distance to the ray tracing hardware only in the case a
  hit was found.

- The default ray mask for geometries got changed from 0xFFFFFFFF to
  0x1.

- The API include folder got renamed from embree3 to embree4, to be
  able to install Embree 3 and Embree 4 side by side, without having
  conflicts in API folder.

- The geometry object of Embree 4 is a host side only object, thus
  accessing it during rendering from the GPU is not allowed. Thus all
  API functions that take an RTCGeometry object as argument cannot get
  used during rendering. Thus in particular the
  `rtcGetGeometryUserData(RTCGeometry)` call cannot get used, but
  there is an alternative function
  `rtcGetGeometryUserDataFromScene(RTCScene scene,uint geomID)` that
  should get used instead.

- For performance reasons, the `rtcInterpolate` function cannot get
  used on the device, and vertex data interpolation should get
  implemented by the application.

- Embree 3 allows to use `rtcIntersect` recursively from a user
  geometry or intersection filter callback to continue a ray inside an
  instantiated object. In Embree 4 using `rtcIntersect` recursively is
  disallowed on the GPU but still supported on the CPU. To properly
  continue a ray inside an instantiated object use the new
  `rtcForwardIntersect1` and `rtcForwardOccluded1` functions.

- When intersection filter callbacks and user geometry callbacks
  assigned to geometries are used, the traversal kernel must have the
  `RTC_SYCL_KERNEL` attribute attached, the indirectly called
  functions must be declared with `RTC_SYCL_INDIRECTLY_CALLABLE`, and
  the `rtcGetSYCLFunctionPointer` API function helper should get used
  to obtain the device side function pointer. However, we recommend to
  pass the callback functions directly to the `rtcIntersect1Ex` and
  `rtcOcclude1Ex` functions to allow inlining (see Section [Xe GPU
  Performance Recommendations]).


\pagebreak

Embree API Reference
====================

``` {include=src/api-ref.md}
```

CPU Performance Recommendations
===============================

MXCSR control and status register
---------------------------------

It is strongly recommended to have the `Flush to Zero` and `Denormals
are Zero` mode of the MXCSR control and status register enabled for
each thread before calling the `rtcIntersect`-type and
`rtcOccluded`-type functions. Otherwise, under some circumstances
special handling of denormalized floating point numbers can
significantly reduce application and Embree performance. When using
Embree together with the Intel® Threading Building Blocks, it is
sufficient to execute the following code at the beginning of the
application main thread (before the creation of the
`tbb::task_scheduler_init` object):

    #include <xmmintrin.h>
    #include <pmmintrin.h>
    ...
    _MM_SET_FLUSH_ZERO_MODE(_MM_FLUSH_ZERO_ON);
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_ON);

If using a different tasking system, make sure each rendering thread
has the proper mode set.

Thread Creation and Affinity Settings
--------------------------------------

Tasking systems like TBB create worker threads on demand, which will
add a runtime overhead for the very first `rtcCommitScene` call. In
case you want to benchmark the scene build time, you should start the
threads at application startup. You can let Embree start TBB threads
by passing `start_threads=1` to the `cfg` parameter of `rtcNewDevice`.

On machines with a high thread count (e.g. dual-socket Xeon or Xeon
Phi machines), affinitizing TBB worker threads increases build and
rendering performance. You can let Embree affinitize TBB worker
threads by passing `set_affinity=1` to the `cfg` parameter of
`rtcNewDevice`. By default, threads are not affinitized by Embree with
the exception of Xeon Phi Processors where they are affinitized by
default.

All Embree tutorials automatically start and affinitize TBB worker
threads by passing `start_threads=1,set_affinity=1` to `rtcNewDevice`.

Fast Coherent Rays
------------------

For getting the highest performance for highly coherent rays, e.g.
primary or hard shadow rays, it is recommended to use packets or
streams of single rays/packets with setting the
`RTC_INTERSECT_CONTEXT_FLAG_COHERENT` flag in the
`RTCIntersectContext` passed to the `rtcIntersect`/`rtcOccluded`
calls. The total number of rays in a coherent stream of ray packets
should be around 64, e.g. 8 times 8-wide packets, or 4 times 16-wide
packets. The rays inside each packet should be grouped as coherent as
possible.

Huge Page Support
-----------------

It is recommended to use huge pages under Linux to increase rendering
performance. Embree supports 2MB huge pages under Windows, Linux, and
macOS. Under Linux huge page support is enabled by default, and under
Windows and macOS disabled by default. Huge page support can be
enabled in Embree by passing `hugepages=1` to `rtcNewDevice` or
disabled by passing `hugepages=0` to `rtcNewDevice`.

We recommend using 2MB huge pages with Embree under Linux as this
improves ray tracing performance by about 5-10%. Under Windows using
huge pages requires the application to run in elevated mode which is a
security issue, thus likely not an option for most use cases. Under
macOS huge pages are rarely available as memory tends to get quickly
fragmented, thus we do not recommend using huge pages on macOS.

### Huge Pages under Linux

Linux supports transparent huge pages and explicit huge pages. To
enable transparent huge page support under Linux, execute the
following as root:

    echo always > /sys/kernel/mm/transparent_hugepage/enabled

When transparent huge pages are enabled, the kernel tries to merge 4KB
pages to 2MB pages when possible as a background job. Many Linux
distributions have transparent huge pages enabled by default. See the
following webpage for more information on
[transparent huge pages under Linux](https://www.kernel.org/doc/Documentation/vm/transhuge.txt).
In this mode each application, including your rendering application
based on Embree, will automatically tend to use huge pages.

Using transparent huge pages, the transitioning from 4KB to 2MB pages
might take some time. For that reason Embree also supports allocating
2MB pages directly when a huge page pool is configured. Such a pool
can be configured by writing some number of huge pages to allocate to
`/proc/sys/vm/nr_overcommit_hugepages` as root user. E.g. to configure
2GB of address space for huge page allocation, execute the following as
root:

    echo 1000 > /proc/sys/vm/nr_overcommit_hugepages

See the following webpage for more information on [huge pages under
Linux](https://www.kernel.org/doc/Documentation/vm/hugetlbpage.txt).

### Huge Pages under Windows

To use huge pages under Windows, the current user must have the "Lock
pages in memory" (SeLockMemoryPrivilege) assigned. This can be
configured through the "Local Security Policy" application, by adding
a user to "Local Policies" -> "User Rights Assignment" -> "Lock pages
in memory". You have to log out and in again for this change to take
effect.

Further, your application must be executed as an elevated process
("Run as administrator") and the "SeLockMemoryPrivilege" must be
explicitly enabled by your application. Example code on how to
enable this privilege can be found in the "common/sys/alloc.cpp" file
of Embree. Alternatively, Embree will try to enable this privilege
when passing `enable_selockmemoryprivilege=1` to `rtcNewDevice`.
Further, huge pages should be enabled in Embree by passing
`hugepages=1` to `rtcNewDevice`.

When the system has been running for a while, physical memory gets
fragmented, which can slow down the allocation of huge pages
significantly under Windows.

### Huge Pages under macOS

To use huge pages under macOS you have to pass `hugepages=1` to
`rtcNewDevice` to enable that feature in Embree.

When the system has been running for a while, physical memory gets
quickly fragmented, and causes huge page allocations to fail. For this
reason, huge pages are not very useful under macOS in practice.

## Avoid store-to-load forwarding issues with single rays

We recommend to use a single SSE store to set up the `org` and `tnear`
components, and a single SSE store to set up the `dir` and `time`
components of a single ray (`RTCRay` type). Storing these values using
scalar stores causes a store-to-load forwarding penalty because Embree
is reading these components using SSE loads later on.

\pagebreak


Xe GPU Performance Recommendations
==================================

Low Code Complexity
-------------------

As a general rule try to keep code complexity
low, to avoid spill code generation. To achieve this we recommend:

- Splitting your rendering into separate kernels instead of using a
  single Uber kernel invokation.

- Configure Embree to the minimal CMake configuration required for
  your application, see Section [CMake Configuration].

- Use SYCL specialization constants and the feature enable mask of the
  `rtcIntersect1Ex` and `rtcOccluded1Ex` calls to JIT compile minimal
  code. The passed feature mask should just contain features required
  to render the current scene. If JIT compile times are an issue,
  reduce the number of feature masks and use JIT caching.

Inline Indirect Calls
---------------------

Do not attach user geometry and intersection filter callbacks to the
geometries of the scene, but directly pass some user geometry and
intersection filter callback function pointers through the
`RTCIntersectArguments` struct to `rtcIntersect1Ex` and
`rtcOccluded1Ex`. If the function is directly passed that way, the
DPC++ compiler can inline the indirect call, which gives a huge
performance benefit.

7 Bit Ray Mask
--------------

Use just the lower 7 bits of the ray and geometry mask. Embree
supports 32 bit ray masks for geometry masking. On the CPU using any
of the 32 bits yields the same performance. The Xe ray tracing
hardware only supports an 8 bit mask, thus Embree has to emulate the
32 bit masking. For that reason the lower 7 mask bits are hardware
accelerated and fast, while the mask bits 7-31 require some software
intervention and using them reduces performance.

Limit Motion Blur Motions
-------------------------

The motion blur implementation on DPC++ has some limitations regarding
supported motion. Primitive motion should be maximally as large as a
small multiple of the primitive size, otherwise performance can
degrade a lot. If detailed geometry moves fast, best put the geometry
into an instance, and apply motion blur the instance itself, to allow
larger motions. As a fallback, problematic scenes can always still get
rendered robustly on the CPU.




