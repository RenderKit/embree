% rtcJoinCommitScene(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcJoinCommitScene - commits the scene from multiple threads

#### SYNOPSIS

    #include <embree3/rtcore.h>

    void rtcJoinCommitScene(RTCScene scene);

#### DESCRIPTION

The `rtcJoinCommitScene` function commits all changes for the
specified scene (`scene` argument). The scene commit internally triggers building of a spatial
acceleration structure for the scene. Ray queries can be performed
after scene changes got properly committed.

The `rtcJoinCommitScene` function can get called from
multiple user threads which will all cooperate in the build operation.
All threads calling into this function will return from
`rtcJoinCommitScene` after the scene commit is finished. All threads
must consistently call `rtcJoinCommitScene` and not `rtcCommitScene`.

In contrast to the `rtcCommitScene` function, the `rtcJoinCommitScene`
function can be called from multiple user threads, while the `rtcCommitScene`
can only get called from multiple TBB worker threads when used concurrently. For
optimal performance we strongly recommend using TBB inside the application
together with the `rtcCommitScene` function and to avoid using the
`rtcJoinCommitScene` function.

The `rtcJoinCommitScene` feature allows a flexible way to lazily
create hierarchies during rendering. A thread reaching a
not-yet-constructed sub-scene of a two-level scene can generate the
sub-scene geometry and call `rtcJoinCommitScene` on that just generated
scene. During construction, further threads reaching the not-yet-built
scene can join the build operation by also invoking
`rtcJoinCommitScene`. A thread that calls `rtcJoinCommitScene` after
the build finishes will directly return from the `rtcJoinCommitScene`
call.

Multiple scene commit operations on different scenes can be running at
the same time, hence it is possible to commit many small scenes in
parallel, distributing the commits to many threads.

When using Embree with the Intel® Threading Building Blocks (which is
the default), threads that call `rtcJoinCommitScene` will join the
build operation, but other TBB worker threads might also participate
in the build. To avoid thread oversubscription, we recommend using TBB
also inside the application. Further, the join mode only works properly
starting with TBB v4.4 Update 1. For earlier TBB versions, threads that
call `rtcJoinCommitScene` to join a running build will just trigger the
build and wait for the build to finish. Further, old TBB versions with
`TBB_INTERFACE_VERSION_MAJOR < 8` do not support `rtcJoinCommitScene`,
and invoking this function will result in an error.

When using Embree with the internal tasking system, only threads that
call `rtcJoinCommitScene` will perform the build operation, and no
additional worker threads will be scheduled.

When using Embree with the Parallel Patterns Library (PPL),
`rtcJoinCommitScene` is not supported and calling that function will
result in an error.

To detect whether `rtcJoinCommitScene` is supported, use the
`rtcGetDeviceProperty` function.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcCommitScene], [rtcGetDeviceProperty]
