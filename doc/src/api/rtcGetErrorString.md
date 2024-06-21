% rtcGetErrorString(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetErrorString - returns a string representation
      of a given RTCError

#### SYNOPSIS

    #include <embree4/rtcore.h>

    const char* rtcGetErrorString(RTCError code);

#### DESCRIPTION

Returns a string representation for a `RTCError` error code. For example, for the
`RTCError` RTC_ERROR_UNKNOWN this function will return the string "Unknown Error".
This is purely a convenience function for printing error information on the user
side.

The returned strings should not be used for comparing different `RTCError` error
codes or make other decisions based on the type of error that occurred. For such
things only the `RTCError` enum values should be used.

#### EXIT STATUS

Returns a string representation of a given `RTCError` error code.

#### SEE ALSO

[rtcGetDeviceError]
