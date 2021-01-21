Embree API
==========

``` {include=src/intro.md}
```

Upgrading from Embree 2 to Embree 3
===================================

We decided to introduce an improved API in Embree 3 that is not
backward compatible with the Embree 2 API. This step was required to
remove various deprecated API functions that accumulated over time,
improve extensibility of the API, fix suboptimal design decisions,
fix design mistakes (such as incompatible single ray and ray packet
layouts), clean up inconsistent naming, and increase flexibility.

To make porting to the new API easy, we provide a conversion script
that can do most of the work, and will annotate the code with
remaining changes required. The script can be invoked the following
way for CPP files:

    ./scripts/cpp-patch.py --patch embree2_to_embree3.patch
      --in infile.cpp --out outfile.cpp

When invoked for ISPC files, add the `--ispc` option:

    ./scripts/cpp-patch.py --ispc --patch embree2_to_embree3.patch
      --in infile.ispc --out outfile.ispc

Apply the script to each source file of your project that contains
Embree API calls or types. The input file and output file can also be
identical to perform the patch in-place. Please always backup your
original code before running the script, and inspect the code changes
done by the script using diff (e.g. `git diff`), to make sure no
undesired code locations got changed. Grep the code for comments
containing `EMBREE_FIXME` and perform the action described in the
comment.

The following changes need to be performed when switching from Embree
2 to Embree 3. Most of these changes are automatically done by the
script if not described differently.

We strongly recommend to set an error callback function (see
`rtcSetDeviceErrorFunction`) when porting to Embree 3 to detect all
runtime errors early.

Device
------

*   `rtcInit` and `rtcExit` got removed. Please use the device concept
    using the `rtcNewDevice` and `rtcReleaseDevice` functions
    instead.

*   Functions that conceptually should operate on a device but did not
    get a device argument got removed. The upgrade script replaces
    these functions by the proper functions that operate on a device,
    however, manually propagating the device handle to these function
    calls might still be required.

Scene
-----

*   The API no longer distinguishes between a static and a dynamic
    scene. Some users had issues as they wanted to do minor
    modifications to static scenes, but maintain high traversal
    performance.

    The new approach gives more flexibility, as each scene is
    changeable, and build quality settings can be changed on a commit
    basis to balance between build performance and render performance.
   
*   The `rtcCommitThread` function got removed; use
    `rtcJoinCommitScene` instead.

*   The scene now supports different build quality settings. Please use
    those instead of the previous way of `RTC_SCENE_STATIC`,
    `RTC_SCENE_DYNAMIC`, and `RTC_SCENE_HIGH_QUALITY` flags.

Geometry
--------

*   There is now only one `rtcNewGeometry` function to create geometries
    which gets passed an enum to specify the type of geometry to create.
    The number of vertices and primitives of the geometries is inferred
    from the size of data buffers.

*   We introduced an object type `RTCGeometry` for all geometries.
    Previously a geometry was not a standalone object and could only
    exist inside a scene. The new approach comes with more flexibility
    and more readable code.

    Operations like `rtcInterpolate` can now be performed on the
    geometry object directly without the need of a scene. Further, an
    application can choose to create its geometries independent of a
    scene, e.g. each time a geometry node is added to its scene graph.

    This modification changed many API functions to get passed one
    `RTCGeometry` object instead of a `RTCScene` and `geomID`. The
    script does all required changed automatically. However, in some
    cases the script may introduce `rtcGetGeometry(scene, geomID)`
    calls to retrieve the geometry handle. Best store the geometry
    handle inside your scene representation (and release it in the
    destructor) and access the handle directly instead of calling
    `rtcGetGeometry`.

*   Geometries are not included inside a scene anymore but can be
    attached to a multiple scenes using the `rtcAttachGeomety` or
    `rtcAttachGeometryByID` functions.

*   As geometries are separate objects, commit semantics got introduced
    for them too. Thus geometries must be committed through the
    `rtcCommitGeometry` call before getting used. This allows for
    earlier error checking and pre-calculating internal data per
    geometry object.

    Such commit points were previously not required in the Embree 2
    API. The upgrade script attempts to insert the commits
    automatically, but cannot do so properly under all
    circumstances. Thus please check if every `rtcCommitGeometry` call
    inserted by the script is properly placed, and if a
    `rtcCommitGeometry` call is placed after a sequence of
    changes to a geometry.

*   Only the latest version of the previous displacement function call
    (`RTCDisplacementFunc2`) is now supported, and the callback is
    passed as a structure containing all arguments.

*   The deprecated `RTCBoundaryMode` type and `rtcSetBoundaryMode`
    function got removed and replaced by `RTCSubdivisionMode` enum and
    the `rtcSetGeometrySubdivisionMode` function. The script does this
    replacement automatically.

*   Ribbon curves and lines now avoid self-intersections automatically
    The application can be simplified by removing special code paths
    that previously did the self-intersection handling.

*   The previous Embree 2 way of instancing was suboptimal as it
    required user geometries to update the `instID` field of the ray
    differently when used inside an instanced scene or inside a
    top-level scene. The user geometry intersection code now just
    has to copy the `context.instID` field into the `ray.instID`
    field to function properly under all circumstances.

*   The internal instancing code will update the `context.instID` field
    properly when entering or leaving an instance. When instancing is
    implemented manually through user geometries, the code must be
    modified to set the `context.instID` field properly and no longer
    pass `instID` through the ray. This change must done manually
    and cannot be performed by the script.

*   We flipped the direction of the geometry normal to the widely used
    convention that a shape with counter-clockwise layout of vertices
    has the normal pointing upwards (right-hand rule). Most modeling tools
    follow that convention.

    The conversion script does not perform this change, thus if
    required adjust your code to flip `Ng` for triangle, quad, and
    subdivision surfaces.

Buffers
-------

*   With Embree 3 we are introducing explicit `RTCBuffer` objects.
    However, you can still use the short way of sharing buffers with
    Embree through the `rtcSetSharedGeometryBuffer` call.

*   The `rtcMapBuffer` and `rtcUnmapBuffer` API calls were removed, and
    we added the `rtcGetBufferData` call instead.

    Previously the `rtcMapBuffer` call had the semantics of creating
    an internal buffer when no buffer was shared for the corresponding
    buffer slot. These invocations of `rtcMapBuffer` must be
    replaced by an explicit creation of an internally managed buffer
    using the `rtcNewGeometryBuffer` function.

    The upgrade script cannot always detect if the `rtcMapBuffer` call
    would create an internal buffer or just map the buffer pointer.
    Thus check whether the `rtcNewGeometryBuffer` and
    `rtcGetBufferData` calls are correct after the conversion.

*   The `rtcUpdateGeometryBuffer` function now must be called for
    every buffer that got modified by the application. Note that the
    conversion script cannot automatically detect each location where
    a buffer update is now required.

*   The buffer type no longer encodes the time step or user vertex
    buffer index. Now `RTC_VERTEX_BUFFER_TYPE` and additional `slot`
    specifies the vertex buffer for a specific time step, and
    `RTC_USER_VERTEX_BUFFER_TYPE` and additional `slot` specifies a
    vertex attribute.

Miscellaneous
-------------

*   The header files for Embree 3 are now inside the `embree3` folder
    (instead of `embree2` folder) and `libembree.so` is now called
    `libembree3.so` to be able to install multiple Embree versions side
    by side. We made the headers C99 compliant.

*   All API objects are now reference counted with release functions to
    decrement and retain functions to increment the reference count
    (if required).

*   Most callback functions no longer get different arguments as input,
    but a pointer to a structure containing all arguments. This
    results in more readable code, faster callback invocation (as some
    arguments do not change between invocations) and is extensible, as
    new members to the structure can be later added in a backward
    compatible way (if required).

    The conversion script can convert the definition and declaration
    of the old callback functions in most cases. Before running the
    script, make sure that you never type-cast a callback function
    when assigning it (as this has the danger of assigning a callback
    function with a wrong type if the conversion did not detect some
    callbacks as such). If the script does not detect a callback
    function, make sure the argument types match exactly the types in
    the header (e.g. write `const int` instead of `int const` or
    convert the callback manually).

*   An intersection context is now required for each ray query
    invocation. The context should be initialized using the
    `rtcInitIntersectContext` function.

*   The `rtcIntersect`-type functions get as input an `RTCRayHit` type,
    which is similar to before, but has the ray and hit parts split
    into two sub-structures.

    The `rtcOccluded`-type functions get as input an `RTCRay` type,
    which does not contain hit data anymore. When an occlusion is
    found, the `tfar` element of the ray is set to `-inf`.

    Required code changes cannot be done by the upgrade script and
    need to be done manually.

*   The ray layout for single rays and packets of rays had certain
    incompatibilities (alignment of `org` and `dir` for single rays
    caused gaps in the single ray layout that were not in the ray
    packet layout). This issue never showed up because single rays
    and ray packets were separate in the system initially. This layout
    issue is now fixed, and a single ray has the same layout as a ray
    packet of size 1.

*   Previously Embree supported placing additional data at the end of
    the ray structure, and accessing that data inside user geometry
    callbacks and filter callback functions.

    With Embree 3 this is no longer supported, and the ray passed to a
    callback function may be copied to a different memory location. To
    attach additional data to your ray, simply extend the intersection
    context with a pointer to that data.

    This change cannot be done by the script. Further, code will still
    work if you extend the ray as the implementation did not change yet.

*   The ray structure now contains an additional `id` and `flags`
    field. The `id` can be used to store the index of the ray with
    respect to a ray packet or ray stream. The `flags` is reserved for
    future use, and currently must be set to 0.

*   All previous intersection filter callback variants have been
    removed, except for the `RTCFilterFuncN` which gets a varying size
    ray packet as input. The semantics of this filter function type
    have changed from copying the hit on acceptance to clearing the
    ray's valid argument in case of non-acceptance. This way, chaining
    multiple filters is more efficient.

    We kept the guarantee that for `rtcIntersect1/4/8/16` and
    `rtcOccluded1/4/8/16` calls the packet size and ray order will not
    change from the initial size and ordering when entering a filter
    callback.

*   We no longer export ISPC-specific symbols. This has the advantage
    that certain linking issues went away, e.g. it is now possible to
    link an ISPC application compiled for any combination of ISAs, and
    link this to an Embree library compiled with a different set of
    ISAs. Previously the ISAs of the application had to be a subset of
    the ISAs of Embree, and when the user enabled exactly one ISA, they
    had to do this in Embree and the application.

*   We no longer export the ISPC tasking system, which means that the
    application has the responsibility to implement the ISPC tasking
    system itself. ISPC comes with example code on how to do this. This
    change is not performed by the script and must be done manually.

*   Fixed many naming inconsistencies, and changed names of further API
    functions. All these renamings are properly done by the script and
    need no further attention.

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


