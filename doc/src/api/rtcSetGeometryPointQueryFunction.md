% rtcSetGeometryPointQueryFunction(3) | Embree Ray Tracing Kernels 3

#### NAME

    rtcSetGeometryPointQueryFunction - sets the point query callback function
      for a geometry

#### SYNOPSIS

    #include <embree3/rtcore.h>

    struct RTCPointQueryFunctionArguments
    {
      // the (world space) query object that was passed as an argument of rtcPointQuery.
      struct RTCPointQuery* query;

      // used for user input/output data. Will not be read or modified internally.
      void* userPtr;

      // primitive and geometry ID of primitive
      unsigned int  primID;        
      unsigned int  geomID;    

      // the context with transformation and instance ID stack
      struct RTCPointQueryContext* context;

      // scaling factor indicating whether the current instance transformation
      // is a similarity transformation.
      float similarityScale;
    };

    typedef bool (*RTCPointQueryFunction)(
      struct RTCPointQueryFunctionArguments* args
    );

    void rtcSetGeometryPointQueryFunction(
      RTCGeometry geometry,
      RTCPointQueryFunction queryFunc
    );

#### DESCRIPTION

The `rtcSetGeometryPointQueryFunction` function registers a point query
callback function (`queryFunc` argument) for the specified geometry
(`geometry` argument).

Only a single callback function can be registered per geometry and
further invocations overwrite the previously set callback function.
Passing `NULL` as function pointer disables the registered callback
function.

The registered callback function is invoked by [rtcPointQuery] for every
primitive of the geometry that intersects the corresponding point query
domain. The callback function of type `RTCPointQueryFunction` gets passed a
number of arguments through the `RTCPointQueryFunctionArguments` structure.
The `query` object is the original point query object passed into
[rtcPointQuery], `usrPtr` is an arbitrary pointer to pass input into and
store results of the callback function. The `primID`, `geomID` and
`context` (see [rtcInitPointQueryContext] for details) can be used to
identify the geometry data of the primitive.

A `RTCPointQueryFunction` can also be passed directly as an argument to
[rtcPointQuery]. In this case the callback is invoked for all primitives in
the scene that intersect the query domain. If a callback function is passed
as an argument to [rtcPointQuery] and (a potentially different) callback
function is set for a geometry with [rtcSetGeometryPointQueryFunction] both
callback functions are invoked and the callback function passed to
[rtcPointQuery] will be called before the geometry specific callback
function.

If instancing is used, the parameter `simliarityScale` indicates whether the
current instance transform (top element of the stack in `context`) is a
similarity transformation or not. Similarity transformations are composed of
translation, rotation and uniform scaling and if a matrix M defines a
similarity transformation, there is a scaling factor D such that for all x,y:
dist(Mx, My) = D * dist(x, y). In this case the parameter `scalingFactor` is
this scaling factor D and otherwise it is 0. A valid similarity scale
(`similarityScale` > 0) allows to compute distance information in instance
space and scale the distances into world space (for example, to update the
query radius, see below) by dividing the instance space distance with the
similarity scale. If the current instance transform is not a similarity
transform (`similarityScale` is 0), the distance computation has to be
performed in world space to ensure correctness. In this case the instance to
world transformations given with the `context` should be used to transform
the primitive data into world space. Otherwise, the query location can be
transformed into instance space which can be more efficient. If there is no
instance transform, the similarity scale is 1.

The callback function will potentially be called for primitives outside the
query domain for two resons: First, the callback is invoked for all
primitives inside a BVH leaf node since no geometry data of primitives is
determined internally and therefore individual primitives are not culled
(only their (aggregated) bounding boxes). Second, in case non similarity
transformations are used, the resulting ellipsoidal query domain (in instance
space) is approximated by its axis aligned bounding box internally and
therefore inner nodes that do not intersect the original domain might
intersect the approximative bounding box which results in unneccessary
callbacks. In any case, the callbacks are conservative, i.e. if a primitive
is inside the query domain a callback will be invoked but the reverse is not
neccessarily true.

For efficiency, the radius of the `query` object can be decreased (in world
space) inside the callback function to improve culling of geometry during BVH
traversal. If the query radius was updated, the callback function should
return `true` to issue an update of internal traversal information.
Increasing the radius or modifying the time or position of the query results
in undefined behaviour.

Within the callback function, it is safe to call [rtcPointQuery] again, for
example when implementing instancing manually. In this case the instance
transformation should be pushed onto the stack in `context`. Embree will
internally compute the point query information in instance space using the
top element of the stack in `context` when [rtcPointQuery] is called.

For a reference implementation of a closest point traversal of triangle
meshes using instancing and user defined instancing see the tutorial
[ClosestPoint].

#### SEE ALSO

[rtcPointQuery], [rtcInitPointQueryContext]
