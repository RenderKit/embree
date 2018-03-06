% rtcGetDeviceError(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcGetDeviceError - returns the error code of the device

#### SYNOPSIS

    #include <embree3/rtcore.h>

    RTCError rtcGetDeviceError(RTCDevice device);

#### DESCRIPTION

Each thread has its own error code per device. If an error occurs when
calling an API function, this error code is set to the occurred error
if it stores no previous error. The `rtcGetDeviceError` function reads
and returns the currently stored error and clears the error code. This
assures that the returned error code is always the first error occurred
since the last invocation of `rtcGetDeviceError`.

Possible error codes returned by `rtcGetDeviceError` are:

+ `RTC_ERROR_NONE`: No error occurred.

+ `RTC_ERROR_UNKNOWN`: An unknown error has occurred.

+ `RTC_ERROR_INVALID_ARGUMENT`: An invalid argument was specified.

+ `RTC_ERROR_INVALID_OPERATION`: The operation is not allowed for the
  specified object.

+ `RTC_ERROR_OUT_OF_MEMORY`: There is not enough memory left to complete
  the operation.

+ `RTC_ERROR_UNSUPPORTED_CPU`: The CPU is not supported as it does not
  support the lowest ISA Embree is compiled for.

+ `RTC_ERROR_CANCELLED`: The operation got canceled by a memory
  monitor callback or progress monitor callback function.

When the device construction fails, `rtcNewDevice` returns `NULL` as
device. To detect the error code of a such a failed device
construction, pass `NULL` as device to the `rtcGetDeviceError`
function. For all other invocations of `rtcGetDeviceError`, a proper
device pointer must be specified.

#### EXIT STATUS

Returns the error code for the device.

#### SEE ALSO

[rtcSetDeviceErrorFunction]
