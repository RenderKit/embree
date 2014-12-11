Embree Tutorials
================

Embree comes with a set of tutorials aimed at helping users understand
how Embree can be used and extended. All tutorials exist in an ISPC and
C version to demonstrate the two versions of the API. Look for files
named `tutorialXX_device.ispc` for the ISPC implementation of the
tutorial, and files named `tutorialXX_device.cpp` for the single ray C++
version of the tutorial. To start the C++ version use the `tutorialXX`
executables, to start the ISPC version use the `tutorialXX_ispc`
executables.

Under Linux Embree also comes with an ISPC version of all tutorials for
the Intel® Xeon Phi™ coprocessor. The executables of this version of the
tutorials are named `tutorialXX_xeonphi` and only work if a Xeon Phi
coprocessor is present in the system. The Xeon Phi version of the
tutorials get started on the host CPU, just like all other tutorials,
and will connect automatically to one installed Xeon Phi coprocessor in
the system.

For all tutorials, you can select an initial camera using the `-vp`
(camera position), `-vi` (camera look-at point), `-vu` (camera up
vector), and `-fov` (vertical field of view) command line parameters:

    ./tutorial00 -vp 10 10 10 -vi 0 0 0

You can select the initial windows size using the `-size` command line
parameter, or start the tutorials in fullscreen using the `-fullscreen`
parameter:

    ./tutorial00 -size 1024 1024
    ./tutorial00 -fullscreen

Implementation specific parameters can be passed to the ray tracing core
through the `-rtcore` command line parameter, e.g.:

    ./tutorial00 -rtcore verbose=2,threads=1,accel=bvh4.triangle1

The navigation in the interactive display mode follows the camera orbit
model, where the camera revolves around the current center of interest.
With the left mouse button you can rotate around the center of interest
(the point initially set with `-vi`). Holding Control pressed while
clicking the left mouse button rotates the camera around its location.
You can also use the arrow keys for navigation.

You can use the following keys:

F1
:   Default shading

F2
:   Gray EyeLight shading

F3
:   Wireframe shading

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
:   Exists the tutorial.

q
:   Exists the tutorial.

Tutorial00
----------

![](images/tutorial00.jpg)

This tutorial demonstrates the creation of a static cube and ground
plane using triangle meshes. It also demonstrates the use of the
`rtcIntersect` and `rtcOccluded` functions to render primary visibility
and hard shadows. The cube sides are colored based on the ID of the hit
primitive.

Tutorial01
----------

![](images/tutorial01.jpg)

This tutorial demonstrates the creation of a dynamic scene, consisting
of several deformed spheres. Half of the spheres use the
RTC\_GEOMETRY\_DEFORMABLE flag, which allows Embree to use a refitting
strategy for these spheres, the other half uses the
RTC\_GEOMETRY\_DYNAMIC flag, causing a rebuild of their spatial data
structure each frame. The spheres are colored based on the ID of the hit
sphere geometry.

Tutorial02
----------

![](images/tutorial02.jpg)

This tutorial shows the use of user defined geometry, to re-implement
instancing and to add analytic spheres. A two level scene is created,
with a triangle mesh as ground plane, and several user geometries, that
instance other scenes with a small number of spheres of different kind.
The spheres are colored using the instance ID and geometry ID of the hit
sphere, to demonstrate how the same geometry, instanced in different
ways can be distinguished.

Tutorial03
----------

![](images/tutorial03.jpg)

This tutorial demonstrates a simple OBJ viewer that traces primary
visibility rays only. A scene consisting of multiple meshes is created,
each mesh sharing the index and vertex buffer with the application.
Demonstrated is also how to support additional per vertex data, such as
shading normals.

You need to specify an OBJ file at the command line for this tutorial to
work: `./tutorial03 -i model.obj`

Tutorial04
----------

![](images/tutorial04.jpg)

This tutorial demonstrates the in-build instancing feature of Embree, by
instancing a number of other scenes build from triangulated spheres. The
spheres are again colored using the instance ID and geometry ID of the
hit sphere, to demonstrate how the same geometry, instanced in different
ways can be distinguished.

Tutorial05
----------

![](images/tutorial05.jpg)

This tutorial demonstrates the use of filter callback functions to
efficiently implement transparent objects. The filter function used for
primary rays, lets the ray pass through the geometry if it is entirely
transparent. Otherwise the shading loop handles the transparency
properly, by potentially shooting secondary rays. The filter function
used for shadow rays accumulates the transparency of all surfaces along
the ray, and terminates traversal if an opaque occluder is hit.

Tutorial06
----------

![](images/tutorial06.jpg)

This tutorial is a simple path tracer, building on tutorial03.

You need to specify an OBJ file and light source at the command line for
this tutorial to work:

    ./tutorial06 -i model.obj -ambientlight 1 1 1

Tutorial07
----------

![](images/tutorial07.jpg)

This tutorial demonstrates the use of the hair geometry to render a
hairball.

Tutorial08
----------

<!-- ![](images/tutorial08.jpg) -->

This tutorial demonstrates the use of Catmull Clark subdivision
surfaces. Per default the edge tessellation level is set adaptively
based on the distance to the camera origin. Embree currently supports
three different modes for efficiently handling subdivision surfaces in
various rendering scenarios. This three modes can be selected at the
command line, e.g. '-lazy' builds internal per subdivision patch data
structures on demand, '-cache' uses a small (per thread) tessellation
cache for caching per patch data, and '-pregenerate' to generate and
store most per patch data during the initial build process. The
'cache' mode is most effective for coherent rays while providing a
fixed memory footprint. The 'pregenerate' modes is most effecitve for
incoherent ray distributions while requiring more memory. The 'lazy'
mode works similar to the 'pregenerate' mode but provides a middle
ground in terms of memory consumption as it only builds and stores
data only when the corresponding patch is accessed during the ray
traversal.

