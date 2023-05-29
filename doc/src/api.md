Embree API
==========

``` {include=src/intro.md}
```

Embree SYCL API
===============

Embree supports ray tracing on Intel GPUs by using the SYCL
programming language. SYCL is a Khronos standardized C++ based
language for single source heterogenous programming for
acceleration offload, see the [SYCL
webpage](https://www.khronos.org/sycl/) for details.

The Embree SYCL API is designed for photorealistic rendering use
cases, where scene setup is performed on the host, and rendering on
the device. The Embree SYCL API is very similar to the standard Embree
C99 API, and supports most of its features, such as all triangle-type
geometries, all curve types and basis functions, point geometry types,
user geometries, filter callbacks, multi-level instancing, and motion
blur.

To enable SYCL support you have to include the
`sycl.hpp` file before the Embree API headers:

    #include <sycl/sycl.hpp>
    #include <embree4/rtcore.h>

Next you need to initializes an Embree SYCL device using the
`rtcNewSYCLDevice` API function by providing a SYCL context.

Embree provides the `rtcIsSYCLDeviceSupported` API function to check
if some SYCL device is supported by Embree. You can also use the
`rtcSYCLDeviceSelector` to conveniently select the first SYCL device that
is supported by Embree, e.g.:

    sycl::device device(rtcSYCLDeviceSelector);
    sycl::queue queue(device, exception_handler);
    sycl::context context(device);
    RTCDevice device = rtcNewSYCLDevice(context,"");

Scenes created with an Embree SYCL device can only get used to trace
rays using SYCL on the GPU, it is not possible to trace rays on the
CPU with such a device. To render on the CPU and GPU in parallel, the
user has to create a second Embree device and create a second
scene to be used on the CPU.

Files containing SYCL code, have to get compiled with the Intel®
oneAPI DPC++ compiler. Please see section [Linux SYCL Compilation] and
[Windows SYCL Compilation] for supported compilers. The DPC++ compiler
performs a two-phase compilation, where host code is compiled in a
first phase, and device code compiled in a second compilation phase.

Standard Embree API functions for scene construction can get used on
the host but not the device. Data buffers that are shared with Embree
(e.g. for vertex of index buffers) have to get allocated as SYCL
unified shared memory (USM memory), using the `sycl::malloc` or
`sycl::aligned_alloc` calls with `sycl::usm::alloc::shared` property,
or the sycl::aligned_alloc_shared call, e.g:

    void* ptr = sycl::aligned_alloc(16, bytes, queue, sycl::usm::alloc::shared);

These shared allocations have to be valid during rendering, as Embree
may access contained data when tracing rays. Embree does not support
device-only memory allocations, as the BVH builder implemented on the
CPU relies on reading the data buffers.

Device side rendering can get invoked by submitting a SYCL
`parallel_for` to the SYCL queue:

    const sycl::specialization_id<RTCFeatureFlags> feature_mask;

    RTCFeatureFlags required_features = RTC_FEATURE_FLAG_TRIANGLE;

    queue.submit([=](sycl::handler& cgh)
    {
      cgh.set_specialization_constant<feature_mask>(required_features);
      
      cgh.parallel_for(sycl::range<1>(1),[=](sycl::id<1> item, sycl::kernel_handler kh)
      {
        RTCIntersectArguments args;
        rtcInitIntersectArguments(&args);

        const RTCFeatureFlags features = kh.get_specialization_constant<feature_mask>();
        args.feature_mask = features;
  
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
  
        rtcIntersect1(scene, &rayhit, &args);
  
        result->geomID = rayhit.hit.geomID;
        result->primID = rayhit.hit.primID;
        result->tfar = rayhit.ray.tfar;
      });
    });
    queue.wait_and_throw();


This example passes a feature mask using a specialization contant to
the `rtcIntersect1` function, which is recommended for GPU
rendering. For best performance, this feature mask should get used to
enable only features required by the application to render the scene,
e.g. just triangles in this example.

Inside the SYCL `parallel_for` loop you can use rendering related functions,
such as the `rtcIntersect1` and `rtcOccluded1` functions to trace rays,
`rtcForwardIntersect1/Ex` and `rtcForwardOccluded1/Ex` to continue object
traversal from inside a user geometry callback, 
and `rtcGetGeometryUserDataFromScene` to get the user data pointer of some
geometry.

Have a look at the [Minimal] tutorial for a minimal SYCL example.


SYCL JIT caching
-----------------

Compile times for just in time compilation (JIT compilation) can be
large. To resolve this issue we recommend enabling persistent JIT
compilation caching inside your application, by setting the
`SYCL_CACHE_PERSISTENT` environment variable to `1`, and the
`SYCL_CACHE_DIR` environment variable to some proper directory where
the JIT cache should get stored. These environment variables have to
get set before the SYCL device is created, e.g:

    setenv("SYCL_CACHE_PERSISTENT","1",1);
    setenv("SYCL_CACHE_DIR","cache_dir",1);

    sycl::device device(rtcSYCLDeviceSelector);
    ...


SYCL Memory Pooling
--------------------

Memory Pooling is a mechanism where small USM memory allocations are
packed into larger allocation blocks. This mode is required when your
application performs many small USM allocations, as otherwise only a
small fraction of GPU memory is usable and data transfer performance
will be low.

Memory pooling is supported for USM allocations that are read-only by
the device. The following example allocated device read-only memory
with memory pooling support:

    sycl::aligned_alloc_shared(align, bytes, queue,
      sycl::ext::oneapi::property::usm::device_read_only());


Embree SYCL Limitations
-----------------------

Embree only supports Xe HPC and HPG GPUs as SYCL devices, thus in
particular the CPU and other GPUs cannot get used as a SYCL
device. To render on the CPU just use the standard C99 API without
relying on SYCL.

The SYCL language spec puts some restrictions to device functions,
such as disallowing: global variable access, malloc, invokation of
virtual functions, function pointers, runtime type information,
exceptions, recursion, etc. See Section `5.4. Language Restrictions
for device functions` of the [SYCL
specification](https://www.khronos.org/registry/SYCL/specs/sycl-2020/html/sycl-2020.html#sec:language.restrictions.kernels)
for more details.

Using Intel's oneAPI DPC++ compiler invoking an indirectly called
function is allowed, but we do not recommend this for performance
reasons.

Some features are not supported by the Embree SYCL API thus cannot
get used on the GPU:

- The packet tracing functions `rtcIntersect4/8/16` and
  `rtcOccluded4/8/16` are not supported in SYCL
  device side code. Using these functions makes no sense for SYCL, as
  the programming model is implicitely executed in SIMT mode on the
  GPU anyway.

- Filter and user geometry callbacks stored inside the geometry
  objects are not supported on SYCL. Please use the alternative
  approach of passing the function pointer through the
  `RTCIntersectArguments` (or `RTCOccludedArguments`) structures to
  the tracing function, which enables inlining on the GPU.

- The `rtcInterpolate` function cannot get used on the the device. For
  most primitive types the vertex data interpolation is anyway a
  trivial operation, and an API call just introduces overheads. On the
  CPU that overhead is acceptable, but on the GPU it is not. The
  `rtcInterpolate` function does not know the geometry type it is
  interpolating over, thus its implementation on the GPU would contain
  a large switch statement for all potential geometry types.

- Tracing rays using `rtcIntersect1` and `rtcOccluded1` functions from
  user geometry callbacks is not supported in SYCL. Please use the
  tail recursive `rtcForwardIntersect1` and `rtcForwardOccluded1`
  calls instead.

- Subdivision surfaces are not supported for Embree SYCL devices.

- Collision detection (`rtcCollide` API call) is not supported in SYCL
  device side code.

- Point queries (`rtcPointQuery` API call) are not supported in SYCL
  device side code.


Embree SYCL Known Issues
------------------------

- The SYCL support of Embree is in beta phase. Current functionality,
  quality, and GPU performance may not reflect that of the final
  product.
  
- Compilation with build configuration "debug" is currently not working on
  Windows.


Upgrading from Embree 3 to Embree 4
===================================

This section summarizes API changes between Embree 3 and Embree4. Most
of these changes are motivated by GPU performance and having a
consistent API that works properly for the CPU and GPU.

- The API include folder got renamed from embree3 to embree4, to be
  able to install Embree 3 and Embree 4 side by side, without having
  conflicts in API folder.

- The `RTCIntersectContext` is renamed to `RTCRayQueryContext` and the
  `RTCIntersectContextFlags` got renamed to `RTCRayQueryFlags`.

- There are some changes to the `rtcIntersect` and `rtcOccluded`
  functions.  Most members of the old intersect context have been
  moved to some optional `RTCIntersectArguments` (and
  `RTCOccludedArguments`) structures, which also contains a pointer to
  the new ray query context. The argument structs fulfill the task of
  providing additional advanced arguments to the traversal
  functions. The ray query context can get used to pass additional
  data to callbacks, and to maintain an instID stack in case
  instancing is done manually inside user geometry callbacks. The
  arguments struct is not available inside callbacks. This change was
  in particular necessary for SYCL to allow inlining of function
  pointers provided to the traversal functions, and to reduce the
  amount of state passed to callbacks, which both improves GPU
  performance. Most applications can just drop passing the ray query
  context to port to Embree 4.
  
- The `rtcFilterIntersection` and `rtcFilterOcclusion` API calls that
  invoke both, the geometry and argument version of the filter
  callback, from a user geometry callback are no longer
  supported. Instead applications should use the
  `rtcInvokeIntersectFilterFromGeometry` and
  `rtcInvokeOccludedFilterFromGeometry` API calls that invoke just the
  geometry version of the filter function, and invoke the argument
  filter function manually if required.

- The filter function passed as arguments to `rtcIntersect` and
  `rtcOccluded` functions is only invoked for some geometry if enabled through
  `rtcSetGeometryEnableFilterFunctionFromArguments` for that
  geometry. Alternatively, argument filter functions can get enabled
  for all geometries using the
  `RTC_RAY_QUERY_FLAG_INVOKE_ARGUMENT_FILTER` ray query flag.

- User geometry callbacks get a valid vector as input to identify
  valid and invalid rays. In Embree 3 the user geometry callback just
  had to update the ray hit members when an intersection was found and
  perform no operation otherwise. In Embree 4 the callback
  additionally has to return valid=-1 when a hit was found, and
  valid=0 when no hit was found. This allows Embree to properly pass
  the new hit distance to the ray tracing hardware only in the case a
  hit was found.

- Further ray masking is enabled by default now as required by most
  applications and the default ray mask for geometries got changed
  from 0xFFFFFFFF to 0x1.

- The stream tracing functions `rtcIntersect1M`, `rtcIntersect1Mp`,
  `rtcIntersectNM`, `rtcIntersectNp`, `rtcOccluded1M`,
  `rtcOccluded1Mp`, `rtcOccludedNM`, and `rtcOccludedNp` got removed
  as they were rarely used and did not provide relevant performance
  benefits. As alternative the application can just iterate over
  `rtcIntersect1` and potentially `rtcIntersect4/8/16` to get similar
  performance.
  
To use Embree through SYCL on the CPU and GPU additional changes are
required:

- Embree 3 allows to use `rtcIntersect` recursively from a user
  geometry or intersection filter callback to continue a ray inside an
  instantiated object. In Embree 4 using `rtcIntersect` recursively is
  disallowed on the GPU but still supported on the CPU. To properly
  continue a ray inside an instantiated object use the new
  `rtcForwardIntersect1` and `rtcForwardOccluded1` functions.

- The geometry object of Embree 4 is a host side only object, thus
  accessing it during rendering from the GPU is not allowed. Thus all
  API functions that take an RTCGeometry object as argument cannot get
  used during rendering. Thus in particular the
  `rtcGetGeometryUserData(RTCGeometry)` call cannot get used, but
  there is an alternative function
  `rtcGetGeometryUserDataFromScene(RTCScene scene,uint geomID)` that
  should get used instead.

- The user geometry callback and filter callback functions should get
  passed through the intersection and occlusion argument structures to
  the `rtcIntersect1` and `rtcOccluded1` functions directly to allow
  inlining. The experimental geometry version of the callbacks is
  disabled in SYCL and should not get used.

- The feature flags should get used in SYCL to minimal GPU code for
  optimal performance.

- The `rtcInterpolate` function cannot get used on the device, and
  vertex data interpolation should get implemented by the application.

- Indirectly called functions must be declared with
  `RTC_SYCL_INDIRECTLY_CALLABLE` when used as filter or user geometry
  callbacks.

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
primary or hard shadow rays, it is recommended to use packets with
setting the `RTC_RAY_QUERY_FLAG_COHERENT` flag in the
`RTCIntersectArguments` struct passed to the
`rtcIntersect`/`rtcOccluded` calls. The rays inside each packet should
be grouped as coherent as possible.

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


GPU Performance Recommendations
================================

Low Code Complexity
-------------------

As a general rule try to keep code complexity low, to avoid spill code
generation. To achieve this we recommend splitting your renderer into
separate kernels instead of using a single Uber kernel invokation.

Code can further get reduced by using SYCL specialization constants to
just enable rendering features required to render a given scene.

Feature Flags
-------------

Use SYCL specialization constants and the feature flags (see section
[RTCFeatureFlags]) of the `rtcIntersect1` and `rtcOccluded1` calls to
JIT compile minimal code. The passed feature flags should just contain
features required to render the current scene. If JIT compile times
are an issue, reduce the number of feature masks used and use JIT
caching (see section [SYCL JIT caching]).

Inline Indirect Calls
---------------------

Attaching user geometry and intersection filter callbacks to the
geometries of the scene is not supported in SYCL for performance reasons.

Instead directly pass the user geometry and intersection filter
callback functions through the `RTCIntersectArguments` (and
`RTCOccludedArguments`) struct to `rtcIntersect1` (and `rtcOccluded1`)
API functions as in the following example:

    RTC_SYCL_INDIRECTLY_CALLABLE void intersectionFilter(
      const RTCFilterFunctionNArguments* args
    ) { ... }

    RTCIntersectArguments args;
    rtcInitIntersectArguments(&args);
    args.filter = intersectionFilter;
    
    rtcIntersect1(scene,&ray,&args);
    
If the callback function is directly passed that way, the SYCL
compiler can inline the indirect call, which gives a huge performance
benefit. Do *not* read a function pointer form some memory location
and pass it to `rtcIntersect1` (and `rtcOccluded1`) as this will also
prevent inlining.

7 Bit Ray Mask
--------------

Use just the lower 7 bits of the ray and geometry mask if possible,
even though Embree supports 32 bit ray masks for geometry masking. On
the CPU using any of the 32 bits yields the same performance, but the
ray tracing hardware only supports an 8 bit mask, thus Embree has to
emulate 32 bit masking if used. For that reason the lower 7 mask bits are
hardware accelerated and fast, while the mask bits 7-31 require some
software intervention and using them reduces performance. To turn on 32 bit
ray masks use the RTC_FEATURE_FLAG_32_BIT_RAY_MASK (see section [RTCFeatureFlags]).

Limit Motion Blur Motions
-------------------------

The motion blur implementation on SYCL has some limitations regarding
supported motion. Primitive motion should be maximally as large as a
small multiple of the primitive size, otherwise performance can
degrade a lot. If detailed geometry moves fast, best put the geometry
into an instance, and apply motion blur to the instance itself, which
efficiently allows larger motions. As a fallback, problematic scenes
can always still get rendered robustly on the CPU.

Generic Pointers
----------------

Embree uses standard C++ pointers in its implementation. SYCL might
not be able to detect the memory space these pointers refer to and has
to treat them as generic pointers which are not performing optimal. The DPC++
compiler has advanced optimizations to infer the proper address space
to avoid usage of generic pointers.

However, if you still encounter the following warning during ahead of
time compilation of SYCL kernels, then loads from generic pointer are
present:

    warning: Adding XX occurrences of additional control flow due to presence
             of generic address space operations in function YYY.

To work around this issue we recommend:

- Do not use local memory inside kernels that trace rays. In this case
  the DPC++ compiler knows that no local memory pointer can exist and
  will optimize generic loads. As this is typically the case for
  renderers, generic pointer will typically not cause issues.

- Indirectly callable functions may still cause problems, even if your
  kernel does not use local memory. Thus best use SYCL pointers like
  sycl::global_ptr<T> and sycl::private_ptr<T> in indirectly callable
  functions to avoid generic address space usage.

- You can also enforce usage of global pointers using the following
  DPC++ compile flags: `-cl-intel-force-global-mem-allocation
  -cl-intel-no-local-to-generic`.

