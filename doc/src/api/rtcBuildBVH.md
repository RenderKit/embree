% rtcBuildBVH(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcBuildBVH - builds a BVH

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTC_ALIGN(32) RTCBuildPrimitive
    {
      float lower_x, lower_y, lower_z; 
      unsigned int geomID;
      float upper_x, upper_y, upper_z;
      unsigned int primID;
    };

    typedef void* (*RTCCreateNodeFunction) (
      RTCThreadLocalAllocator allocator,
      unsigned int childCount,
      void* userPtr
    );
    
    typedef void (*RTCSetNodeChildrenFunction) (
      void* nodePtr,
      void** children,
      unsigned int childCount,
      void* userPtr
    );
    
    typedef void (*RTCSetNodeBoundsFunction) (
      void* nodePtr,
      const struct RTCBounds** bounds,
      unsigned int childCount,
      void* userPtr
    );
    
    typedef void* (*RTCCreateLeafFunction) (
      RTCThreadLocalAllocator allocator,
      const struct RTCBuildPrimitive* primitives,
      size_t primitiveCount,
      void* userPtr
    );
    
    typedef void (*RTCSplitPrimitiveFunction) (
      const struct RTCBuildPrimitive* primitive,
      unsigned int dimension,
      float position,
      struct RTCBounds* leftBounds,
      struct RTCBounds* rightBounds,
      void* userPtr
    );

    typedef bool (*RTCProgressMonitorFunction)(
      void* userPtr, double n
    );

    enum RTCBuildFlags
    {
      RTC_BUILD_FLAG_NONE,
      RTC_BUILD_FLAG_DYNAMIC
    };

    struct RTCBuildArguments
    {
      size_t byteSize;
    
      enum RTCBuildQuality buildQuality;
      enum RTCBuildFlags buildFlags;
      unsigned int maxBranchingFactor;
      unsigned int maxDepth;
      unsigned int sahBlockSize;
      unsigned int minLeafSize;
      unsigned int maxLeafSize;
      float traversalCost;
      float intersectionCost;
    
      RTCBVH bvh;
      struct RTCBuildPrimitive* primitives;
      size_t primitiveCount;
      size_t primitiveArrayCapacity;
      
      RTCCreateNodeFunction createNode;
      RTCSetNodeChildrenFunction setNodeChildren;
      RTCSetNodeBoundsFunction setNodeBounds;
      RTCCreateLeafFunction createLeaf;
      RTCSplitPrimitiveFunction splitPrimitive;
      RTCProgressMonitorFunction buildProgress;
      void* userPtr;
    };

    struct RTCBuildArguments rtcDefaultBuildArguments();

    void* rtcBuildBVH(
      const struct RTCBuildArguments* args
    );

#### DESCRIPTION

The `rtcBuildBVH` function can be used to build a BVH in a
user-defined format over arbitrary primitives. All arguments to the
function are provided through the `RTCBuildArguments` structure. The
first member of that structure must be set to the size of the structure
in bytes (`bytesSize` member) which allows future extensions of the
structure. It is recommended to initialize the build arguments
structure using the `rtcDefaultBuildArguments` function.

The `rtcBuildBVH` function gets passed the BVH to build (`bvh` member),
the array of primitives (`primitives` member), the capacity of that
array (`primitiveArrayCapacity` member), the number of primitives
stored inside the array (`primitiveCount` member), callback function
pointers, and a user-defined pointer (`userPtr` member) that is passed
to all callback functions when invoked. The `primitives` array can be
freed by the application after the BVH is built. All callback functions
are typically called from multiple threads, thus their implementation
must be thread-safe.

Four callback functions must be registered, which are invoked during
build to create BVH nodes (`createNode` member), to set the pointers to
all children (`setNodeChildren` member), to set the bounding boxes of
all children (`setNodeBounds` member), and to create a leaf node
(`createLeaf` member).

The function pointer to the primitive split function (`splitPrimitive`
member) may be `NULL`, however, then no spatial splitting in high
quality mode is possible. The function pointer used to report the
build progress (`buildProgress` member) is optional and may also be
`NULL`.

Further, some build settings are passed to configure the BVH build.
Using the build quality settings (`buildQuality` member), one can
select between a faster, low quality build which is good for dynamic
scenes, and a standard quality build for static scenes. One can also
specify the desired maximum branching factor of the BVH
(`maxBranchingFactor` member), the maximum depth the BVH should have
(`maxDepth` member), the block size for the SAH heuristic
(`sahBlockSize` member), the minimum and maximum leaf size
(`minLeafSize` and `maxLeafSize` member), and the estimated costs of
one traversal step and one primitive intersection (`traversalCost`
and `intersectionCost` members). When enabling the
`RTC_BUILD_FLAG_DYNAMIC` build flags (`buildFlags` member), re-build
performance for dynamic scenes is improved at the cost of higher
memory requirements.

To spatially split primitives in high quality mode, the builder needs
extra space at the end of the build primitive array to store splitted
primitives. The total capacity of the build primitive array is passed
using the `primitiveArrayCapacity` member, and should be about twice
the number of primitives when using spatial splits.

The `RTCCreateNodeFunc` and `RTCCreateLeafFunc` callbacks are passed
a thread local allocator object that should be used for fast
allocation of nodes using the `rtcThreadLocalAlloc` function. We
strongly recommend using this allocation mechanism, as alternative
approaches like standard `malloc` can be over 10× slower. The
allocator object passed to the create callbacks may be used only
inside the current thread. Memory allocated using `rtcThreadLocalAlloc`
is automatically freed when the `RTCBVH` object is deleted. If you use
your own memory allocation scheme you have to free the memory yourself
when the `RTCBVH` object is no longer used.

The `RTCCreateNodeFunc` callback additionally gets the number of
children for this node in the range from 2 to `maxBranchingFactor`
(`childCount` argument).

The `RTCSetNodeChildFunc` callback function gets a pointer to the node
as input (`nodePtr` argument), an array of pointers to the children
(`childPtrs` argument), and the size of this array (`childCount`
argument).

The `RTCSetNodeBoundsFunc` callback function gets a pointer to the
node as input (`nodePtr` argument), an array of pointers to the
bounding boxes of the children (`bounds` argument), and the size of
this array (`childCount` argument).

The `RTCCreateLeafFunc` callback additionally gets an array of
primitives as input (`primitives` argument), and the size of this
array (`primitiveCount` argument). The callback should read the
`geomID` and `primID` members from the passed primitives to construct
the leaf.

The `RTCSplitPrimitiveFunc` callback is invoked in high quality mode
to split a primitive (`primitive` argument) at the specified position
(`position` argument) and dimension (`dimension` argument). The callback
should return bounds of the clipped left and right parts of the
primitive (`leftBounds` and `rightBounds` arguments).

The `RTCProgressMonitorFunction` callback function is called with the
estimated completion rate `n` in the range $[0,1]$. Returning `true`
from the callback lets the build continue; returning `false` cancels
the build.

#### EXIT STATUS

On failure an error code is set that can be queried using
`rtcGetDeviceError`.

#### SEE ALSO

[rtcNewBVH]
