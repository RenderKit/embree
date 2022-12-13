% rtcInitIntersectContext(3) | Embree Ray Tracing Kernels 4

#### NAME

    rtcInitIntersectContext - initializes the intersection context

#### SYNOPSIS

    #include <embree4/rtcore.h>

    struct RTCIntersectContext
    {
      #if RTC_MAX_INSTANCE_LEVEL_COUNT > 1
        unsigned int instStackSize;
      #endif
      
      unsigned int instID[RTC_MAX_INSTANCE_LEVEL_COUNT];
    };

    void rtcInitIntersectContext(
      struct RTCIntersectContext* context
    );

#### DESCRIPTION

The `rtcInitIntersectContext` function initializes the intersection
context to default values and should be called to initialize every
intersection context.

It is guaranteed that the pointer to the intersection context
(`RTCIntersectContext` type) is passed to the registered callback
functions. This way it is possible to attach arbitrary data to the end
of the intersection context, such as a per-ray payload.

Inside the user geometry callback the intersection context can get
used to access the `instID` stack to know which instance the user
geometry object resides.

If not intersection context is specified when tracing a ray, a default
context is used.

#### EXIT STATUS

No error code is set by this function.

#### SEE ALSO

[rtcIntersect1], [rtcIntersect4/8/16], [rtcOccluded1], [rtcOccluded4/8/16]
