Embree API
==========

``` {include=src/intro.md}
```
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

- Embree supports 32 bit ray masks for geometry masking. On the CPU
  using any of the 32 bits yields the same performance. The Xe ray
  tracing hardware only supports 8 bit masks, thus Embree has to
  emulate the 32 bit masking. For that reason the lower 7 mask bits
  are hardware accelerated and fast, while the mask bits 7-31 require
  some software intervention and using them reduces performance.

- The default ray mask for some geometry got changed from 0xFFFFFFFF
  to 0x1 to get best performance on ray tracing hardware.

- The API include folder got renamed from embree3 to embree4, to be
  able to install Embree 3 and Embree 4 side by side, without having
  conflicts in API folder.

- The geometry object of Embree 4 is a host side only object, thus
  accessing it during rendering from the GPU is not allowed. Thus all
  API functions that take an RTCGeometry object as argument cannot get
  used during rendering. Thus in particular the
  `rtcGetGeometryUserData(RTCGeometry)` call cannot get used, but
  there is an alternative function `rtcGetGeometryUserData(RTCScene
  scene,uint geomID)` that should get used instead.

- The `rtcInterpolate` function is forbidden to get used on the
  GPU. For most primitive types the vertex data interpolation is
  anyway a trivial operation, and an API call just introduces
  overheads. On the CPU that overhead is acceptable, but on the GPU it
  is not. The rtcInterpolate function does not know the geometry type
  it is interpolating over, thus its implementation on the GPU would
  contain a large switch statement for all potential geometry types.

- Embree 3 allows to use `rtcIntersect` recursivaly from a user
  geometry or intersection filter callback to continue a ray inside an
  instantiated object. In Embree 4 using `rtcIntersect` recursively is
  disallowed on the GPU but still supported on the CPU. To properly
  continue a ray inside an instantiated object use the new
  `rtcForwardIntersect1` function.

- Embree 4 does not support subdivision surfaces on the GPU, but on
  the CPU.

- Embree 4 does not implement the collision detection `rtcCollide`
  feature for the GPU.

- Embree 4 does not implement point queries on the GPU.

\pagebreak

Embree API Reference
====================

``` {include=src/api-ref.md}
```

Performance Recommendations
===========================

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


