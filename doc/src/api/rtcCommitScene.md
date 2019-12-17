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

If the application uses TBB 2019 Update 9 or later for parallelization
of rendering, lazy scene construction during rendering is supported by
`rtcCommitScene`. Therefore `rtcCommitScene` can get called from
multiple TBB worker threads concurrently for the same scene. The
`rtcCommitScene` function will then internally isolate the scene
construction using a tbb::isolated_task_group. The alternative
approach of using `rtcJoinCommitScene` which uses an tbb:task_arena
internally, is not recommended due to it's high runtime overhead.

If scene geometries get modified or attached or detached, the
`rtcCommitScene` call must be invoked before performing any further
ray queries for the scene; otherwise the effect of the ray query is
undefined. The modification of a geometry, committing the scene, and
tracing of rays must always happen sequentially, and never at the same
time. Any API call that sets a property of the scene or geometries
contained in the scene count as scene modification, e.g. including
setting of intersection filter functions.

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
