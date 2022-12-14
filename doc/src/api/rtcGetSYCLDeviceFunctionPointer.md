% rtcGetSYCLDeviceFunctionPointer(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcGetSYCLDeviceFunctionPointer - obtains a device side
      function pointer for some SYCL function

#### SYNOPSIS

    #include <embree4/rtcore.h>

    template<auto F>
    inline decltype(F) rtcGetSYCLDeviceFunctionPointer(sycl::queue& queue);

#### DESCRIPTION

This function returns a device side function pointer for some function
F. This function F must be defined using the
`RTC_SYCL_INDIRECTLY_CALLABLE` attribute, e.g.:

    RTC_SYCL_INDIRECTLY_CALLABLE void filter(
      const RTCFilterFunctionNArguments* args) { ... }

    RTCFilterFunctionN fptr = rtcGetSYCLDeviceFunctionPointer<filter>(queue);

Such a device side function pointers of some filter callbacks can get
assigned to a geometry using the `rtcSetGeometryIntersectFilterFunction` and
`rtcSetGeometryOccludedFilterFunction` API functions.

Further, device side function pointers for user geometry callbacks can
be assigned to geometries using the `rtcSetGeometryIntersectFunction` and 
`rtcSetGeometryOccludedFunction` API calls.

These geometry versions of the callback functions are disabled in SYCL
by default, and we recommend not using them for performance reasons.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcSetGeometryIntersectFunction], [rtcSetGeometryOccludedFunction], [rtcSetGeometryIntersectFilterFunction], [rtcSetGeometryOccludedFilterFunction]
