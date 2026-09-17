% rtcSetGeometryOrientedBoundsFunction(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcSetGeometryOrientedBoundsFunction - sets a callback to query the
      oriented bounds of user-defined primitives

#### SYNOPSIS

    #include <embree4/rtcore.h>

    struct RTCOrientedBounds
    {
      float center_x, center_y, center_z, align0;
      float axis0_x, axis0_y, axis0_z, align1;
      float axis1_x, axis1_y, axis1_z, align2;
      float axis2_x, axis2_y, axis2_z, align3;
    };

    struct RTCOrientedBoundsFunctionArguments
    {
      void* geometryUserPtr;
      unsigned int primID;
      unsigned int timeStep;
      struct RTCOrientedBounds* bounds_o;
      void* boundsUserPtr;
    };

    typedef void (*RTCOrientedBoundsFunction)(
      const struct RTCOrientedBoundsFunctionArguments* args
    );

    void rtcSetGeometryOrientedBoundsFunction(
      RTCGeometry geometry,
      RTCOrientedBoundsFunction bounds,
      void* userPtr
    );

#### DESCRIPTION

The `rtcSetGeometryOrientedBoundsFunction` function registers an oriented
bounding box callback for an `RTC_GEOMETRY_TYPE_USER_ORIENTED` geometry.
The callback is invoked for every primitive and time step during acceleration
structure construction.

The callback writes the box center and three mutually orthogonal half-axis
vectors to `bounds_o`. The lengths of the vectors are the half-extents, and
their directions specify the primitive orientation. Embree uses the complete
orientation when evaluating and constructing oriented BVH nodes.
Bounds containing NaN or infinite values are ignored. Zero-length axes are
supported and do not require special handling by the callback.

The `geometryUserPtr` member contains the pointer set with
`rtcSetGeometryUserData`. The `primID` and `timeStep` members identify the
requested primitive and time step. The `boundsUserPtr` member contains the
callback payload passed as `userPtr`.

In SYCL mode BVH construction is performed on the host, and the callback must
be a host-side function pointer.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_USER_ORIENTED], [rtcSetGeometryUserData],
[rtcSetGeometryIntersectFunction], [rtcSetGeometryOccludedFunction]
