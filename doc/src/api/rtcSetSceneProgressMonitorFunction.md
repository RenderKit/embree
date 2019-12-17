% rtcSetSceneProgressMonitorFunction(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetSceneProgressMonitorFunction - registers a callback
      to track build progress

#### SYNOPSIS

    #include <embree3/rtcore.h>

    typedef bool (*RTCProgressMonitorFunction)(
      void* ptr,
      double n
    );

    void rtcSetSceneProgressMonitorFunction(
      RTCScene scene,
      RTCProgressMonitorFunction progress,
      void* userPtr
    );

#### DESCRIPTION

Embree supports a progress monitor callback mechanism that can be
used to report progress of hierarchy build operations and to cancel
build operations.

The `rtcSetSceneProgressMonitorFunction` registers a progress monitor
callback function (`progress` argument) with payload (`userPtr` argument)
for the specified scene (`scene` argument).

Only a single callback function can be registered per scene, and
further invocations overwrite the previously set callback function.
Passing `NULL` as function pointer disables the registered callback
function.

Once registered, Embree will invoke the callback function multiple
times during hierarchy build operations of the scene, by passing the
payload as set at registration time (`userPtr` argument), and a double
in the range $[0, 1]$ which estimates the progress of the operation
(`n` argument). The callback function might be called from multiple
threads concurrently.

When returning `true` from the callback function, Embree will continue
the build operation normally. When returning `false`, Embree will
cancel the build operation with the `RTC_ERROR_CANCELLED` error
code. Issuing multiple cancel requests for the same build operation is
allowed.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewScene]
