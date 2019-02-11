% rtcSetDeviceErrorFunction(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetDeviceErrorFunction - sets an error callback function for the device

#### SYNOPSIS

    #include <embree3/rtcore.h>

    typedef void (*RTCErrorFunction)(
      void* userPtr,
      RTCError code,
      const char* str
    );

    void rtcSetDeviceErrorFunction(
      RTCDevice device,
      RTCErrorFunction error,
      void* userPtr
    );

#### DESCRIPTION

Using the `rtcSetDeviceErrorFunction` call, it is possible to set a
callback function (`error` argument) with payload (`userPtr` argument),
which is called whenever an error occurs for the specified device
(`device` argument).

Only a single callback function can be registered per device, and
further invocations overwrite the previously set callback function.
Passing `NULL` as function pointer disables the registered callback
function.

When the registered callback function is invoked, it gets passed the
user-defined payload (`userPtr` argument as specified at registration
time), the error code (`code` argument) of the occurred error, as well
as a string (`str` argument) that further describes the error.

The error code is also set if an error callback function is
registered.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetDeviceError]
