% rtcGetSceneDevice(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetSceneDevice - returns the device the scene got created in

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCDevice rtcGetSceneDevice(RTCScene scene);

#### DESCRIPTION

This function returns the device object the scene got created in. The
returned handle own one additional reference to the device object,
thus you should need to call `rtcReleaseDevice` when the returned
handle is no longer required.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcReleaseDevice]
