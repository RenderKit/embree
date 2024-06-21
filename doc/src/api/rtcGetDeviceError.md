% rtcGetDeviceError(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetDeviceError - returns the error code of the device

#### SYNOPSIS

    #include <embree4/rtcore.h>

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

+ `RTC_ERROR_LEVEL_ZERO_RAYTRACING_SUPPORT_MISSING`: This error can occur when
  creating an Embree device with SYCL support using `rtcNewSYCLDevice` fails.
  This error probably means that the GPU driver is to old or not installed
  properly. Install a new GPU driver and on Linux make sure that the package
  `intel-level-zero-gpu-raytracing` is installed. For general driver installation
  information for Linux refer to [https://dgpu-docs.intel.com](https://dgpu-docs.intel.com).

When the device construction fails, `rtcNewDevice` returns `NULL` as
device. To detect the error code of a such a failed device
construction, pass `NULL` as device to the `rtcGetDeviceError`
function. For all other invocations of `rtcGetDeviceError`, a proper
device pointer must be specified.

The API function `rtcGetDeviceLastErrorMessage` can be used to get more details
about the last `RTCError` a `RTCDevice` encountered.

For convenient reporting of a `RTCError`, the API function `rtcGetErrorString`
can be used, which returns a string representation of a given `RTCError`.

#### EXIT STATUS

Returns the error code for the device.

#### SEE ALSO

[rtcSetDeviceErrorFunction], [rtcGetDeviceLastErrorMessage], [rtcGetErrorString]
