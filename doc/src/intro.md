The Embree API is a low-level C99 ray tracing API which can be used to
construct 3D scenes and perform ray queries of different types inside
these scenes. All API calls carry the prefix `rtc` (or `RTC` for
types) which stands for **r**ay **t**racing **c**ore.

The API also exists in an ISPC version, which is almost identical but
contains additional functions that operate on ray packets with a size
of the native SIMD width used by ISPC. For simplicity this document
refers to the C99 version of the API functions. For changes when
upgrading from the Embree 2 to the current Embree 3 API see Section
[Upgrading from Embree 2 to Embree 3].

The API supports scenes consisting of different geometry types such as
triangle meshes, quad meshes (triangle pairs), grid meshes, flat
curves, round curves, oriented curves, subdivision meshes, instances,
and user-defined geometries. See Section [Scene Object] for more
information.

Finding the closest hit of a ray segment with the scene
(`rtcIntersect`-type functions), and determining whether any hit
between a ray segment and the scene exists (`rtcOccluded`-type
functions) are both supported. The API supports queries for single
rays, ray packets, and ray streams. See Section [Ray Queries] for
more information.

The API is designed in an object-oriented manner, e.g. it contains
device objects (`RTCDevice` type), scene objects (`RTCScene` type),
geometry objects (`RTCGeometry` type), buffer objects (`RTCBuffer`
type), and BVH objects (`RTCBVH` type). All objects are reference
counted, and handles can be released by calling the appropriate release
function (e.g. `rtcReleaseDevice`) or retained by incrementing the
reference count (e.g. `rtcRetainDevice`). In general, API calls that
access the same object are not thread-safe, unless specified
differently. However, attaching geometries to the same scene and
performing ray queries in a scene is thread-safe.

Device Object
-------------

Embree supports a device concept, which allows different components of
the application to use the Embree API without interfering with each
other. An application typically first creates a device using the
[rtcNewDevice] function. This device can then be used to construct
further objects, such as scenes and geometries. Before the application
exits, it should release all devices by invoking [rtcReleaseDevice]. An
application typically creates only a single device. If required
differently, it should only use a small number of devices at any given
time.

Each user thread has its own error flag per device. If an error occurs
when invoking an API function, this flag is set to an error code (if
it isn't already set by a previous error). See Section
[rtcGetDeviceError] for information on how to read the error code
and Section [rtcSetDeviceErrorFunction] on how to register a
callback that is invoked for each error encountered. It is recommended
to always set a error callback function, to detect all errors.

Scene Object
------------

A scene is a container for a set of geometries, and contains a spatial
acceleration structure which can be used to perform different types of
ray queries.

A scene is created using the `rtcNewScene` function call, and released
using the `rtcReleaseScene` function call. To populate a scene with
geometries use the `rtcAttachGeometry` call, and to detach them use the
`rtcDetachGeometry` call. Once all scene geometries are attached, an
`rtcCommitScene` call (or `rtcJoinCommitScene` call) will finish the
scene description and trigger building of internal data structures.
After the scene got committed, it is safe to perform ray queries (see
Section [Ray Queries]) or to query the scene bounding box (see
[rtcGetSceneBounds] and [rtcGetSceneLinearBounds]).

If scene geometries get modified or attached or detached, the
`rtcCommitScene` call must be invoked before performing any further
ray queries for the scene; otherwise the effect of the ray query is
undefined. The modification of a geometry, committing the scene, and
tracing of rays must always happen sequentially, and never at the same
time. Any API call that sets a property of the scene or geometries
contained in the scene count as scene modification, e.g. including
setting of intersection filter functions.

Scene flags can be used to configure a scene to use less memory
(`RTC_SCENE_FLAG_COMPACT`), use more robust traversal algorithms
(`RTC_SCENE_FLAG_ROBUST`), and to optimize for dynamic content. See
Section [rtcSetSceneFlags] for more details.

A build quality can be specified for a scene to balance between
acceleration structure build performance and ray query performance.
See Section [rtcSetSceneBuildQuality] for more details on build
quality.

Geometry Object
---------------

A new geometry is created using the `rtcNewGeometry` function.
Depending on the geometry type, different buffers must be bound (e.g.
using `rtcSetSharedGeometryBuffer`) to set up the geometry data. In
most cases, binding of a vertex and index buffer is required. The
number of primitives and vertices of that geometry is typically
inferred from the size of these bound buffers.

Changes to the geometry always must be committed using the
`rtcCommitGeometry` call before using the geometry. After committing,
a geometry is not included in any scene. A geometry can be added to
a scene by using the `rtcAttachGeometry` function (to automatically
assign a geometry ID) or using the `rtcAttachGeometryById` function
(to specify the geometry ID manually). A geometry can get attached
to multiple scenes.

All geometry types support multi-segment motion blur with an arbitrary
number of equidistant time steps (in the range of 2 to 129) inside a
user specified time range. Each geometry can have a different number
of time steps and a different time range. The motion blur geometry is
defined by linearly interpolating the geometries of neighboring time
steps. To construct a motion blur geometry, first the number of time
steps of the geometry must be specified using the
`rtcSetGeometryTimeStepCount` function, and then a vertex buffer for
each time step must be bound, e.g. using the
`rtcSetSharedGeometryBuffer` function. Optionally, a time range
defining the start (and end time) of the first (and last) time step
can be set using the `rtcSetGeometryTimeRange` function. This feature
will also allow geometries to appear and disappear during the camera
shutter time if the time range is a sub range of [0,1].

The API supports per-geometry filter callback functions (see
`rtcSetGeometryIntersectFilterFunction` and
`rtcSetGeometryOccludedFilterFunction`) that are invoked for each
intersection found during the `rtcIntersect`-type or
`rtcOccluded`-type calls. The former ones are called geometry
intersection filter functions, the latter ones geometry occlusion
filter functions. These filter functions are designed to be used to
ignore intersections outside of a user-defined silhouette of a
primitive, e.g. to model tree leaves using transparency textures.

Ray Queries
-----------

The API supports finding the closest hit of a ray segment with the
scene (`rtcIntersect`-type functions), and determining whether any hit
between a ray segment and the scene exists (`rtcOccluded`-type
functions).

Supported are single ray queries (`rtcIntersect1` and `rtcOccluded1`)
as well as ray packet queries for ray packets of size 4
(`rtcIntersect4` and `rtcOccluded4`), ray packets of size 8
(`rtcIntersect8` and `rtcOccluded8`), and ray packets of size 16
(`rtcIntersect16` and `rtcOccluded16`).

Ray streams in a variety of layouts are supported as well, such as
streams of single rays (`rtcIntersect1M` and `rtcOccluded1M`), streams
of pointers to single rays (`rtcIntersect1p` and `rtcOccluded1p`),
streams of ray packets (`rtcIntersectNM` and `rtcOccludedNM`), and
large packet-like streams in structure of pointer layout
(`rtcIntersectNp` and `rtcOccludedNp`).

See Sections [rtcIntersect1] and [rtcOccluded1] for a detailed
description of how to set up and trace a ray.

See tutorial [Triangle Geometry] for a complete example of how to
trace single rays and ray packets. Also have a look at the tutorial
[Stream Viewer] for an example of how to trace ray streams.

Point Queries
-------------

The API supports traversal of the BVH using a point query object that
specifies a location and a query radius. For all primitives intersecting the
according domain, a user defined callback function is called which allows
queries such as finding the closest point on the surface geometries of the
scene (see Tutorial [Closest Point]) or nearest neighbour queries (see
Tutorial [Voronoi]).

See Section [rtcPointQuery] for a detailed description of how to set up
point queries.

Collision Detection
-------------------

The Embree API also supports collision detection queries between two
scenes consisting only of user geometries. Embree only performs
broadphase collision detection, the narrow phase detection can be
performed through a callback function.

See Section [rtcCollide] for a detailed description of how to set up collision
detection.

Seen tutorial [Collision Detection] for a complete example of collsion 
detection being used on a simple cloth solver.


Miscellaneous
-------------

A context filter function, which can be set per ray query is supported
(see `rtcInitIntersectContext`). This filter function is designed to
change the semantics of the ray query, e.g. to accumulate opacity for
transparent shadows, count the number of surfaces along a ray,
collect all hits along a ray, etc.

The internal algorithms to build a BVH are exposed through the `RTCBVH`
object and `rtcBuildBVH` call. This call makes it possible to build a
BVH in a user-specified format over user-specified primitives. See the
documentation of the `rtcBuildBVH` call for more details.

For getting the most performance out of Embree, see the Section
[Performance Recommendations].
