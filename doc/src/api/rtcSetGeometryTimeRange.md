% rtcSetGeometryTimeRange(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryTimeRange - sets the time range for a motion blur geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcSetGeometryTimeRange(
      RTCGeometry geometry,
      float startTime,
      float endTime
    );

#### DESCRIPTION

The `rtcSetGeometryTimeRange` function sets a time range which defines
the start (and end time) of the first (and last) time step of a motion
blur geometry. The time range is defined relative to the camera
shutter interval [0,1] but it can be arbitrary. Thus the startTime can
be smaller, equal, or larger 0, indicating a geometry whose animation
definition start before, at, or after the camera shutter
opens. Similar the endTime can be smaller, equal, or larger than 1,
indicating a geometry whose animation definition ends after, at, or
before the camera shutter closes. The startTime has to be smaller or
equal to the endTime.

The default time range when this function is not called is the entire
camera shutter [0,1]. For best performance at most one time segment of
the piece wise linear definition of the motion should fall outside the
shutter window to the left and to the right. Thus do not set the
startTime or endTime too far outside the [0,1] interval for best
performance.

This time range feature will also allow geometries to appear and
disappear during the camera shutter time if the specified time range
is a sub range of [0,1].

Please also have a look at the `rtcSetGeometryTimeStepCount` function
to see how to define the time steps for the specified time range.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryTimeStepCount]

