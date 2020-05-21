
Embree Tutorials
================

Embree comes with a set of tutorials aimed at helping users understand
how Embree can be used and extended. There is a very basic minimal
that can be compiled as both C and C++, which should get new users started quickly. 
All other tutorials exist in an ISPC and C++ version to demonstrate 
the two versions of the API. Look for files
named `tutorialname_device.ispc` for the ISPC implementation of the
tutorial, and files named `tutorialname_device.cpp` for the single ray C++
version of the tutorial. To start the C++ version use the `tutorialname`
executables, to start the ISPC version use the `tutorialname_ispc`
executables. All tutorials can print available command line options
using the `--help` command line parameter.

For all tutorials except minimal, you can select an initial camera using 
the `--vp` (camera position), `--vi` (camera look-at point), `--vu` 
(camera up vector), and `--fov` (vertical field of view) command line 
parameters:

    ./triangle_geometry --vp 10 10 10 --vi 0 0 0

You can select the initial window size using the `--size` command line
parameter, or start the tutorials in full screen using the `--fullscreen`
parameter:

    ./triangle_geometry --size 1024 1024
    ./triangle_geometry --fullscreen

The initialization string for the Embree device (`rtcNewDevice` call)
can be passed to the ray tracing core through the `--rtcore` command
line parameter, e.g.:

    ./triangle_geometry --rtcore verbose=2,threads=1

The navigation in the interactive display mode follows the camera orbit
model, where the camera revolves around the current center of interest.
With the left mouse button you can rotate around the center of interest
(the point initially set with `--vi`). Holding Control pressed while
clicking the left mouse button rotates the camera around its location.
You can also use the arrow keys for navigation.

You can use the following keys:

F1
:   Default shading

F2
:   Gray EyeLight shading

F3
:   Traces occlusion rays only.

F4
:   UV Coordinate visualization

F5
:   Geometry normal visualization

F6
:   Geometry ID visualization

F7
:   Geometry ID and Primitive ID visualization

F8
:   Simple shading with 16 rays per pixel for benchmarking.

F9
:   Switches to render cost visualization. Pressing again reduces
    brightness.

F10
:   Switches to render cost visualization. Pressing again increases
    brightness.

f
:   Enters or leaves full screen mode.

c
:   Prints camera parameters.

ESC
:   Exits the tutorial.

q
:   Exits the tutorial.

Minimal
-------

This tutorial is designed to get new users started with Embree.
It can be compiled as both C and C++. It demonstrates how to initialize
a device and scene, and how to intersect rays with the scene.
There is no image output to keep the tutorial as simple as possible.

Triangle Geometry
-----------------

![][imgTriangleGeometry]

This tutorial demonstrates the creation of a static cube and ground
plane using triangle meshes. It also demonstrates the use of the
`rtcIntersect1` and `rtcOccluded1` functions to render primary visibility
and hard shadows. The cube sides are colored based on the ID of the hit
primitive.

Dynamic Scene
-------------

![][imgDynamicScene]

This tutorial demonstrates the creation of a dynamic scene, consisting
of several deforming spheres. Half of the spheres use the
`RTC_BUILD_QUALITY_REFIT` geometry build quality, which allows Embree
to use a refitting strategy for these spheres, the other half uses the
`RTC_BUILD_QUALITY_LOW` geometry build quality, causing a high
performance rebuild of their spatial data structure each frame. The
spheres are colored based on the ID of the hit sphere geometry.

Multi Scene Geometry
-------------

![][imgDynamicScene]

This tutorial demonstrates the creation of multiple scenes sharing the
same geometry objects.  Here, three scenes are built.  One with all
the dynamic spheres of the Dynamic Scene test and two others each with
half.  The ground plane is shared by all three scenes.  The space bar
is used to cycle the scene chosen for rendering.

User Geometry
-------------

![][imgUserGeometry]

This tutorial shows the use of user-defined geometry, to re-implement
instancing, and to add analytic spheres. A two-level scene is created,
with a triangle mesh as ground plane, and several user geometries that
instance other scenes with a small number of spheres of different kinds.
The spheres are colored using the instance ID and geometry ID of the hit
sphere, to demonstrate how the same geometry instanced in different
ways can be distinguished.

Viewer
------

![][imgViewer]

This tutorial demonstrates a simple OBJ viewer that traces primary
visibility rays only. A scene consisting of multiple meshes is created,
each mesh sharing the index and vertex buffer with the application.
It also demonstrates how to support additional per-vertex data, such as
shading normals.

You need to specify an OBJ file at the command line for this tutorial to
work:

    ./viewer -i model.obj

Stream Viewer
-------------

![][imgViewerStream]

This tutorial is a simple OBJ viewer that demonstrates the use of ray
streams. You need to specify an OBJ file at the command line for this
tutorial to work:

    ./viewer_stream -i model.obj

Intersection Filter
-------------------

![][imgIntersectionFilter]

This tutorial demonstrates the use of filter callback functions to
efficiently implement transparent objects. The filter function used for
primary rays lets the ray pass through the geometry if it is entirely
transparent. Otherwise, the shading loop handles the transparency
properly, by potentially shooting secondary rays. The filter function
used for shadow rays accumulates the transparency of all surfaces along
the ray, and terminates traversal if an opaque occluder is hit.

Instanced Geometry
------------------

![][imgInstancedGeometry]

This tutorial demonstrates the in-build instancing feature of Embree, by
instancing a number of other scenes built from triangulated spheres. The
spheres are again colored using the instance ID and geometry ID of the
hit sphere, to demonstrate how the same geometry instanced in different
ways can be distinguished.

Multi Level Instancing
----------------------

![][imgMultiLevelInstancing]

This tutorial demonstrates multi-level instancing, i.e., nesting instances
into instances. To enable the tutorial, set the compile-time variable
`EMBREE_MAX_INSTANCE_LEVEL_COUNT` to a value other than the default 1.
This variable is available in the code as `RTC_MAX_INSTANCE_LEVEL_COUNT`.

The renderer uses a basic path tracing approach, and the
image will progressively refine over time.
There are two levels of instances in this scene: multiple instances of
the same tree nest instances of a twig.
Intersections on up to `RTC_MAX_INSTANCE_LEVEL_COUNT` nested levels of
instances work out of the box. Users may obtain the *instance ID stack* for
a given hitpoint from the `instID` member.
During shading, the instance ID stack is used to accumulate
normal transformation matrices for each hit. The tutorial visualizes
transformed normals as colors.

Path Tracer
-----------

![][imgPathtracer]

This tutorial is a simple path tracer, based on the viewer tutorial.

You need to specify an OBJ file and light source at the command line for
this tutorial to work:

    ./pathtracer -i model.obj --ambientlight 1 1 1

As example models we provide the "Austrian Imperial Crown" model by
[Martin Lubich](http://www.loramel.net) and the "Asian Dragon" model from the
[Stanford 3D Scanning Repository](http://graphics.stanford.edu/data/3Dscanrep/).

[crown.zip](https://github.com/embree/models/releases/download/release/crown.zip)

[asian_dragon.zip](https://github.com/embree/models/releases/download/release/asian_dragon.zip)

To render these models execute the following:

    ./pathtracer -c crown/crown.ecs
    ./pathtracer -c asian_dragon/asian_dragon.ecs

Hair
----

![][imgHairGeometry]

This tutorial demonstrates the use of the hair geometry to render a
hairball.

Curve Geometry
--------------

![][imgCurveGeometry]

This tutorial demonstrates the use of the B-Spline and Catmull-Rom curve geometries.

Subdivision Geometry
--------------------

![][imgSubdivisionGeometry]

This tutorial demonstrates the use of Catmull-Clark subdivision
surfaces.

Displacement Geometry
---------------------

![][imgDisplacementGeometry]

This tutorial demonstrates the use of Catmull-Clark subdivision
surfaces with procedural displacement mapping using a constant edge
tessellation level.

Grid Geometry
---------------------

![][imgGridGeometry]

This tutorial demonstrates the use of the memory efficient grid
primitive to handle highly tessellated and displaced geometry.

Point Geometry
---------------------

![][imgPointGeometry]

This tutorial demonstrates the use of the three representations
of point geometry.

Motion Blur Geometry
--------------------

![][imgMotionBlurGeometry]

This tutorial demonstrates rendering of motion blur using the
multi-segment motion blur feature. Shown is motion blur of a triangle mesh,
quad mesh, subdivision surface, line segments, hair geometry, Bézier
curves, instantiated triangle mesh where the instance moves,
instantiated quad mesh where the instance and the quads move, and user
geometry.

The number of time steps used can be configured using the `--time-steps
<int>` and `--time-steps2 <int>` command line parameters, and the
geometry can be rendered at a specific time using the the `--time
<float>` command line parameter.

Quaternion Motion Blur
----------------------

![][imgQuaternionMotionBlur]

This tutorial demonstrates rendering of motion blur using quaternion
interpolation. Shown is motion blur using spherical linear interpolation of
the rotational component of the instance transformation on the left and
simple linear interpolation of the instance transformation on the right. The
number of time steps can be modified as well.


Interpolation
-------------

![][imgInterpolation]

This tutorial demonstrates interpolation of user-defined per-vertex data.

Closest Point
----------------------

![][imgClosestPoint]

This tutorial demonstrates a use-case of the point query API. The scene
consists of a simple collection of objects that are instanced and for several
point in the scene (red points) the closest point on the surfaces of the
scene are computed (white points). The closest point functionality is
implemented for Embree internal and for user-defined instancing. The tutorial
also illustrates how to handle instance transformations that are not
similarity transforms.

Voronoi
----------------------

![][imgVoronoi]

This tutorial demonstrates how to implement nearest neighbour lookups using
the point query API. Several colored points are located on a plane and the
corresponding voroni regions are illustrated.

Collision Detection
----------------------

![][imgCollision]

This tutorial demonstrates how to implement collision detection using
the collide API. A simple cloth solver is setup to collide with a sphere.

The cloth can be reset with the `space` bar.  The sim stepped once with `n` 
and continuous simulation started and paused with `p`.

BVH Builder
-----------

This tutorial demonstrates how to use the templated hierarchy builders
of Embree to build a bounding volume hierarchy with a user-defined
memory layout using a high-quality SAH builder using spatial splits, a
standard SAH builder, and a very fast Morton builder.

BVH Access
-----------

This tutorial demonstrates how to access the internal triangle
acceleration structure build by Embree. Please be aware that the
internal Embree data structures might change between Embree updates.

Find Embree
-----------

This tutorial demonstrates how to use the `FIND_PACKAGE` CMake feature
to use an installed Embree. Under Linux and macOS the tutorial finds
the Embree installation automatically, under Windows the `embree_DIR`
CMake variable must be set to the following folder of the Embree
installation: `C:\Program Files\Intel\Embree3`.

Next Hit
-----------

This tutorial demonstrates how to robustly enumerate all hits along
the ray using multiple ray queries and an intersection filter
function. To improve performance, the tutorial also supports
collecting the next N hits in a single ray query.



