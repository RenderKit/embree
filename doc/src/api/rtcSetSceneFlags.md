% rtcSetSceneFlags(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcSetSceneFlags - sets the flags for the scene

#### SYNOPSIS

    #include <embree4/rtcore.h>

    enum RTCSceneFlags
    {
      RTC_SCENE_FLAG_NONE                    = 0,
      RTC_SCENE_FLAG_DYNAMIC                 = (1 << 0),
      RTC_SCENE_FLAG_COMPACT                 = (1 << 1),
      RTC_SCENE_FLAG_ROBUST                  = (1 << 2),
      RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS = (1 << 3)
    };

    void rtcSetSceneFlags(RTCScene scene, enum RTCSceneFlags flags);

#### DESCRIPTION

The `rtcSetSceneFlags` function sets the scene flags (`flags`
argument) for the specified scene (`scene` argument). Possible scene
flags are:

+ `RTC_SCENE_FLAG_NONE`: No flags set.

+ `RTC_SCENE_FLAG_DYNAMIC`: Provides better build performance for
  dynamic scenes (but also higher memory consumption).

+ `RTC_SCENE_FLAG_COMPACT`: Uses compact acceleration structures
  and avoids algorithms that consume much memory.

+ `RTC_SCENE_FLAG_ROBUST`: Uses acceleration structures that allow
  for robust traversal, and avoids optimizations that reduce arithmetic
  accuracy. This mode is typically used for avoiding artifacts caused
  by rays shooting through edges of neighboring primitives.

+ `RTC_SCENE_FLAG_FILTER_FUNCTION_IN_ARGUMENTS`: Enables scene support
  for filter functions passed as argument to the traversal
  functions. See Section [rtcInitIntersectArguments] and
  [rtcInitOccludedArguments] for more details.

Multiple flags can be enabled using an `or` operation,
e.g. `RTC_SCENE_FLAG_COMPACT | RTC_SCENE_FLAG_ROBUST`.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcGetSceneFlags]
