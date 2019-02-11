% rtcSetGeometryBoundsFunction(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryBoundsFunction - sets a callback to query the
      bounding box of user-defined primitives

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCBoundsFunctionArguments
    {
      void* geometryUserPtr;
      unsigned int primID;
      unsigned int timeStep;
      struct RTCBounds* bounds_o;
    };
  
    typedef void (*RTCBoundsFunction)(
      const struct RTCBoundsFunctionArguments* args
    );

    void rtcSetGeometryBoundsFunction(
      RTCGeometry geometry,
      RTCBoundsFunction bounds,
      void* userPtr
    );

#### DESCRIPTION

The `rtcSetGeometryBoundsFunction` function registers a bounding box
callback function (`bounds` argument) with payload (`userPtr` argument)
for the specified user geometry (`geometry` argument).

Only a single callback function can be registered per geometry, and
further invocations overwrite the previously set callback function.
Passing `NULL` as function pointer disables the registered callback
function.

The registered bounding box callback function is invoked to calculate
axis-aligned bounding boxes of the primitives of the user-defined
geometry during spatial acceleration structure construction. The
bounding box callback of `RTCBoundsFunction` type is invoked with a
pointer to a structure of type `RTCBoundsFunctionArguments` which
contains various arguments, such as: the user data of the geometry
(`geometryUserPtr` member), the ID of the primitive to calculate the
bounds for (`primID` member), the time step at which to calculate the
bounds (`timeStep` member), and a memory location to write the
calculated bound to (`bounds_o` member).

In a typical usage scenario one would store a pointer to the internal
representation of the user geometry object using
`rtcSetGeometryUserData`. The callback function can then read that
pointer from the `geometryUserPtr` field and calculate the proper bounding
box for the requested primitive and time, and store that bounding box
to the destination structure (`bounds_o` member).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[RTC_GEOMETRY_TYPE_USER]

