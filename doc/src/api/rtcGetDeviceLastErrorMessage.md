% rtcGetDeviceLastErrorMessage(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetDeviceLastErrorMessage - returns a message corresponding
      to the last error code

#### SYNOPSIS

    #include <embree4/rtcore.h>

    const char* rtcGetDeviceLastErrorMessage(RTCDevice device);

#### DESCRIPTION

This function can be used to get a message corresponding to the last error code
(returned by `rtcGetDeviceError`) which often provides details about the error
that happened. The message is the same as the message that will written to
console when verbosity is > 0 or which is passed as the `str` argument of the
`RTCErrorFunction` (see [rtcSetDeviceErrorFunction]). However, when device
construction fails this is the only way to get additional information about
the error. In this case, `rtcNewDevice` returns `NULL` as device. To query the
error message for such a failed device construction, pass `NULL` as device to the
`rtcGetDeviceLastErrorMessage` function. For all other invocations of
`rtcGetDeviceLastErrorMessage`, a proper device pointer must be specified.

#### EXIT STATUS

Returns a message corresponding to the last error code.

#### SEE ALSO

[rtcGetDeviceError], [rtcSetDeviceErrorFunction]
