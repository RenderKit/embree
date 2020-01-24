% rtcCollide(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcCollide - intersects one BVH with another

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCCollision {
      unsigned int geomID0, primID0;
      unsigned int geomID1, primID1;
    };
    
    typedef void (*RTCCollideFunc) (
      void* userPtr,
      RTCCollision* collisions,
      size_t num_collisions);

    void rtcCollide (
        RTCScene hscene0, 
        RTCScene hscene1, 
        RTCCollideFunc callback, 
        void* userPtr
    );

#### DESCRIPTION

The `rtcCollide` function intersects the BVH of `hscene0` with the BVH of 
scene `hscene1` and calls a user defined callback function (e.g `callback` 
argument) for each pair of intersecting primitives between the two scenes.
A user defined data pointer (`userPtr` argument) can also be passed in.

For every pair of primitives that may intersect each other, the
callback function (`callback` argument) is called. The user will be
provided with the primID's and geomID's of multiple potentially
intersecting primitive pairs. Currently, only scene entirely composed
of user geometries are supported, thus the user is expected to
implement a primitive/primitive intersection to filter out false
positives in the callback function. The `userPtr` argument can be used
to input geometry data of the scene or output results of the
intersection query.

#### SUPPORTED PRIMITIVES

Currently, the only supported type is the user geometry type 
(see [RTC_GEOMETRY_TYPE_USER]).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO
