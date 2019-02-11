% rtcCommitScene(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcCommitScene - commits scene changes

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcCommitScene(RTCScene scene);

#### DESCRIPTION

The `rtcCommitScene` function commits all changes for the specified
scene (`scene` argument). This internally triggers building of a
spatial acceleration structure for the scene using all available
worker threads. Ray queries can be performed only after committing
all scene changes.

The kind of acceleration structure built can be influenced using scene
flags (see `rtcSetSceneFlags`), and the quality can be specified
using the `rtcSetSceneBuildQuality` function.

Embree silently ignores primitives during spatial acceleration
structure construction that would cause numerical issues,
e.g. primitives containing NaNs, INFs, or values greater
than 1.844E18f (as no reasonable calculations can be performed with
such values without causing overflows).

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcJoinCommitScene]
