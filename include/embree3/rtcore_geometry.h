// ======================================================================== //
// Copyright 2009-2017 Intel Corporation                                    //
//                                                                          //
// Licensed under the Apache License, Version 2.0 (the "License");          //
// you may not use this file except in compliance with the License.         //
// You may obtain a copy of the License at                                  //
//                                                                          //
//     http://www.apache.org/licenses/LICENSE-2.0                           //
//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "rtcore_buffer.h"

/*! \ingroup embree_kernel_api */
/*! \{ */

#if defined(__cplusplus)
extern "C" {
#endif
  
/*! invalid geometry ID */
#define RTC_INVALID_GEOMETRY_ID ((unsigned)-1)

/*! maximal number of time steps */
#define RTC_MAX_TIME_STEP_COUNT 129

/*! \brief Supported types of matrix layout for functions involving matrices */
enum RTCMatrixType {
  RTC_MATRIX_ROW_MAJOR = 0,
  RTC_MATRIX_COLUMN_MAJOR = 1,
  RTC_MATRIX_COLUMN_MAJOR_ALIGNED16 = 2,
};

/*! Geometry type */
enum RTCGeometryType
{
  /*! \brief Creates a new triangle mesh. The number of triangles
    (numTriangles), number of vertices (numVertices), and number of time
    steps (1 for normal meshes, and up to RTC_MAX_TIME_STEP_COUNT for multi
    segment motion blur), have to get specified. The triangle indices
    can be set by mapping and writing to the index buffer
    (RTC_BUFFER_TYPE_INDEX) and the triangle vertices can be set by mapping
    and writing into the vertex buffer (RTC_BUFFER_TYPE_VERTEX). In case of
    multi-segment motion blur, multiple vertex buffers have to get filled
    (RTC_VERTEX_BUFFER0, RTC_VERTEX_BUFFER1, etc.), one for each time
    step. The index buffer has the default layout of three 32 bit
    integer indices for each triangle. An index points to the ith
    vertex. The vertex buffer stores single precision x,y,z floating
    point coordinates aligned to 16 bytes. The value of the 4th float
    used for alignment can be arbitrary. */
  RTC_GEOMETRY_TYPE_TRIANGLE,

  /*! \brief Creates a new quad mesh. The number of quads (numQuads),
    number of vertices (numVertices), and number of time steps (1 for
    normal meshes, and up to RTC_MAX_TIME_STEP_COUNT for multi-segment motion
    blur), have to get specified. The quad indices can be set by mapping
    and writing to the index buffer (RTC_BUFFER_TYPE_INDEX) and the quad
    vertices can be set by mapping and writing into the vertex buffer
    (RTC_BUFFER_TYPE_VERTEX). In case of multi-segment motion blur, multiple
    vertex buffers have to get filled (RTC_VERTEX_BUFFER0,
    RTC_VERTEX_BUFFER1, etc.), one for each time step. The index buffer has
    the default layout of three 32 bit integer indices for each quad. An
    index points to the ith vertex. The vertex buffer stores single
    precision x,y,z floating point coordinates aligned to 16 bytes. The
    value of the 4th float used for alignment can be arbitrary. */
  RTC_GEOMETRY_TYPE_QUAD,

  /*! \brief Creates a new subdivision mesh. The number of faces
    (numFaces), edges/indices (numEdges), vertices (numVertices), edge
    creases (numEdgeCreases), vertex creases (numVertexCreases), holes
    (numHoles), and time steps (numTimeSteps) have to get speficied at
    construction time.

    The following buffers have to get filled by the application: the face
    buffer (RTC_BUFFER_TYPE_FACE) contains the number edges/indices (3 or 4)
    of each of the numFaces faces, the index buffer (RTC_BUFFER_TYPE_INDEX)
    contains multiple (3 or 4) 32bit vertex indices for each face and
    numEdges indices in total, the vertex buffer (RTC_BUFFER_TYPE_VERTEX)
    stores numVertices vertices as single precision x,y,z floating point
    coordinates aligned to 16 bytes. The value of the 4th float used for
    alignment can be arbitrary. In case of multi-segment motion blur,
    multiple vertex buffers have to get filled (RTC_VERTEX_BUFFER0,
    RTC_VERTEX_BUFFER1, etc.), one for each time step.

    Optionally, the application can fill the hole buffer
    (RTC_BUFFER_TYPE_HOLE) with numHoles many 32 bit indices of faces that
    should be considered non-existing.

    Optionally, the application can fill the level buffer
    (RTC_BUFFER_TYPE_LEVEL) with a tessellation level for each of the numEdges
    edges. The subdivision level is a positive floating point value, that
    specifies how many quads along the edge should get generated during
    tessellation. The tessellation level is a lower bound, thus the
    implementation is free to choose a larger level. If no level buffer
    is specified a level of 1 is used.

    Optionally, the application can fill the sparse edge crease buffers
    to make some edges appear sharper. The edge crease index buffer
    (RTC_BUFFER_TYPE_EDGE_CREASE_INDEX) contains numEdgeCreases many pairs of
    32 bit vertex indices that specify unoriented edges. The edge crease
    weight buffer (RTC_BUFFER_TYPE_EDGE_CREASE_WEIGHT) stores for each of
    theses crease edges a positive floating point weight. The larger this
    weight, the sharper the edge. Specifying a weight of infinify is
    supported and marks an edge as infinitely sharp. Storing an edge
    multiple times with the same crease weight is allowed, but has lower
    performance. Storing the an edge multiple times with different
    crease weights results in undefined behaviour. For a stored edge
    (i,j), the reverse direction edges (j,i) does not have to get stored,
    as both are considered the same edge.

    Optionally, the application can fill the sparse vertex crease buffers
    to make some vertices appear sharper. The vertex crease index buffer
    (RTC_BUFFER_TYPE_VERTEX_CREASE_INDEX), contains numVertexCreases many 32
    bit vertex indices to speficy a set of vertices. The vertex crease
    weight buffer (RTC_BUFFER_TYPE_VERTEX_CREASE_WEIGHT) specifies for each of
    these vertices a positive floating point weight. The larger this
    weight, the sharper the vertex. Specifying a weight of infinity is
    supported and makes the vertex infinitely sharp. Storing a vertex
    multiple times with the same crease weight is allowed, but has lower
    performance. Storing a vertex multiple times with different crease
    weights results in undefined behaviour. */
  RTC_GEOMETRY_TYPE_SUBDIVISION,

  /*! \brief Creates a new hair geometry, consisting of multiple hairs
    represented as cubic bezier curves with varying radii. The number of
    curves (numCurves), number of vertices (numVertices), and number of
    time steps have to get specified at construction time (1 for normal
    meshes, and up to RTC_MAX_TIME_STEP_COUNT for multi-segment motion
    blur). Further, the curve index buffer (RTC_BUFFER_TYPE_INDEX) and the
    curve vertex buffer (RTC_BUFFER_TYPE_VERTEX) have to get set by mapping
    and writing to the appropiate buffers. In case of multi-segment
    motion blur multiple vertex buffers have to get filled
    (RTC_VERTEX_BUFFER0, RTC_VERTEX_BUFFER1, etc.), one for each time
    step. The index buffer has the default layout of a single 32 bit
    integer index for each curve, that references the start vertex of
    the curve. The vertex buffer stores 4 control points per curve, each
    such control point consists of a single precision (x,y,z) position
    and radius, stored in that order in memory. Individual hairs are
    considered to be subpixel sized which allows the implementation to
    approximate the intersection calculation. This in particular means
    that zooming onto one hair might show geometric artefacts. */
  RTC_GEOMETRY_TYPE_LINEAR_CURVE,
  RTC_GEOMETRY_TYPE_BEZIER_CURVE,
  RTC_GEOMETRY_TYPE_BSPLINE_CURVE,

  /*! Creates a new user geometry object. This feature makes it possible
    to add arbitrary types of geometry to the scene by providing
    appropiate bounding, intersect and occluded functions. A user
    geometry object is a set of user geometries. As the rtcIntersect
    and rtcOccluded functions support different ray packet sizes, the
    user also has to provide different versions of intersect and
    occluded function pointers for these packet sizes. However, the
    ray packet size of the called function pointer always matches the
    packet size of the originally invoked rtcIntersect and rtcOccluded
    functions. A user data pointer, that points to a user specified
    representation of the geometry, is passed to each intersect and
    occluded function invokation, as well as the index of the geometry
    of the set to intersect. */
  RTC_GEOMETRY_TYPE_USER,

  RTC_GEOMETRY_TYPE_INSTANCE
};

/*! Geometry subtype, which can be changed for already existing geometries */
enum RTCGeometrySubtype
{
  RTC_GEOMETRY_SUBTYPE_DEFAULT = 0, //!< default geometry subtype
  RTC_GEOMETRY_SUBTYPE_SURFACE = 1, //!< render curves as real geometric surfaces
  RTC_GEOMETRY_SUBTYPE_RIBBON = 2   //!< render curves as ray facing ribbons
};

/*! \brief Interpolation mode for subdivision surfaces. The modes are
 *  ordered to interpolate successively more linear. */
enum RTCSubdivisionMode
{
  RTC_SUBDIVISION_MODE_NO_BOUNDARY     = 0, //!< ignores border patches
  RTC_SUBDIVISION_MODE_SMOOTH_BOUNDARY = 1, //!< smooth border (default)
  RTC_SUBDIVISION_MODE_PIN_CORNERS     = 2, //!< smooth border with fixed corners
  RTC_SUBDIVISION_MODE_PIN_BOUNDARY    = 3, //!< linearly interpolation along border
  RTC_SUBDIVISION_MODE_PIN_ALL         = 4, //!< pin every vertex (interpolates every patch linearly)
};

enum RTCCurveFlags { 
  RTC_CURVE_FLAG_NEIGHBOR_LEFT  = (1 << 0), 
  RTC_CURVE_FLAG_NEIGHBOR_RIGHT = (1 << 1) 
};

/*! Arguments for RTCBoundsFunction */
struct RTCBoundsFunctionArguments
{
  void* geomUserPtr;          //!< pointer to geometry user data
  unsigned int primID;        //!< item to calculate bounds for
  unsigned int time;          //!< time to calculate bounds for
  struct RTCBounds* bounds_o; //!< returns calculated bounds
};
  
/*! Type of bounding function */
typedef void (*RTCBoundsFunction)(const struct RTCBoundsFunctionArguments* const args);

/*! Arguments for RTCIntersectFunctionN */
struct RTCIntersectFunctionNArguments
{
  int* valid;                          //!< pointer to valid mask
  void* geomUserPtr;                   //!< pointer to geometry user data
  unsigned int primID;                 //!< ID of primitive to intersect
  struct RTCIntersectContext* context; //!< intersection context as passed to rtcIntersect/rtcOccluded
  struct RTCRayN* ray;                 //!< ray packet to intersect
  unsigned int N;                      //!< number of rays in packet
};

/*! Type of intersect function pointer for ray packets of size N */
typedef void (*RTCIntersectFunctionN)(const struct RTCIntersectFunctionNArguments* const args);

/*! Arguments for RTCOccludedFunctionN */
struct RTCOccludedFunctionNArguments
{
  int* valid;                          //!< pointer to valid mask
  void* geomUserPtr;                   //!< pointer to geometry user data
  unsigned int primID;                 //!< ID if primitive to intersect
  struct RTCIntersectContext* context; //!< intersection context as passed to rtcIntersect/rtcOccluded
  struct RTCRayN* ray;                 //!< ray packet to intersect
  unsigned int N;                      //!< number of rays in packet
};
  
/*! Type of occlusion function pointer for ray packets of size N. */
typedef void (*RTCOccludedFunctionN)(const struct RTCOccludedFunctionNArguments* const args);

/*! report intersection from intersect function */
RTCORE_API void rtcFilterIntersection(const struct RTCIntersectFunctionNArguments* const args, const struct RTCFilterFunctionNArguments* filterArgs);

/*! report intersection from occluded function */
RTCORE_API void rtcFilterOcclusion(const struct RTCOccludedFunctionNArguments* const args, const struct RTCFilterFunctionNArguments* filterArgs);

/*! \brief Defines an opaque geometry type */
typedef struct __RTCGeometry* RTCGeometry;

/*! Arguments for RTCDisplacementFunction callback */
struct RTCDisplacementFunctionArguments
{
  void* geomUserPtr;    //!< pointer to user data of geometry
  RTCGeometry geometry; //!< geometry handle to displace
  unsigned int primID;  //!< ID of primitive of geometry to displace
  unsigned int time;    //!< time step to calculate displacement for
  const float* u;       //!< u coordinates (source)
  const float* v;       //!< v coordinates (source)
  const float* nx;      //!< x coordinates of normalized normal at point to displace (source)
  const float* ny;      //!< y coordinates of normalized normal at point to displace (source)
  const float* nz;      //!< z coordinates of normalized normal at point to displace (source)
  float* px;            //!< x coordinates of points to displace (source and target)
  float* py;            //!< y coordinates of points to displace (source and target)
  float* pz;            //!< z coordinates of points to displace (source and target)
  unsigned int N;       //!< number of points to displace
};
 
/*! Displacement mapping function */
typedef void (*RTCDisplacementFunction)(const struct RTCDisplacementFunctionArguments* const args);

/*! Creates a new geometry. */
RTCORE_API RTCGeometry rtcNewGeometry(RTCDevice device, enum RTCGeometryType type, enum RTCGeometrySubtype subtype);

/*! Sets the bounding function to calculate bounding boxes of the user
 *  geometry items when building spatial index structures. The
 *  calculated bounding box have to be conservative and should be
 *  tight. */
RTCORE_API void rtcSetGeometryBoundsFunction(RTCGeometry geometry, RTCBoundsFunction bounds, void* userPtr);

/*! Set intersect function for ray packets of size N. The rtcIntersectN function
 *  will call the passed function for intersecting the user
 *  geometry. */
RTCORE_API void rtcSetGeometryIntersectFunction(RTCGeometry geometry, RTCIntersectFunctionN intersect);

/*! Set occlusion function for ray packets of size N. The rtcOccludedN function
 *  will call the passed function for intersecting the user
 *  geometry. */
RTCORE_API void rtcSetGeometryOccludedFunction(RTCGeometry geometry, RTCOccludedFunctionN occluded);

/*! \brief Creates a new scene instance. 

  A scene instance contains a reference to a scene to instantiate and
  the transformation to instantiate the scene with. For motion blurred
  instances, a number of timesteps can get specified. An
  implementation will typically transform the ray with the inverse of
  the provided transformation (or inverse of linearly interpolated
  transformation in case of multi-segment motion blur) and continue
  traversing the ray through the provided scene. If any geometry is
  hit, the instance ID (instID) member of the ray will get set to the
  geometry ID of the instance. */
RTCORE_API RTCGeometry rtcNewInstance(RTCDevice device,
                                      RTCScene source,                  //!< the scene to instantiate
                                      unsigned int numTimeSteps);       //!< number of timesteps, one matrix per timestep

/*! \brief Sets transformation of the instance for specified timestep */
RTCORE_API void rtcSetGeometryTransform(RTCGeometry geometry,                   //!< ID of geometry 
                                        enum RTCMatrixType layout,              //!< layout of transformation matrix
                                        const float* xfm,                       //!< pointer to transformation matrix
                                        unsigned int timeStep                   //!< timestep to set the matrix for 
  );

/*! Sets the number of primitives. */
RTCORE_API void rtcSetGeometryUserPrimitiveCount(RTCGeometry geometry, unsigned int userPrimCount);

/*! Sets the number of time steps. */
RTCORE_API void rtcSetGeometryTimeStepCount(RTCGeometry geometry, unsigned int timeStepCount);
 
/*! Sets a uniform tessellation rate for subdiv meshes and hair
 *  geometry. For subdivision meshes the RTC_BUFFER_TYPE_LEVEL can also be used
 *  optionally to set a different tessellation rate per edge.*/
RTCORE_API void rtcSetGeometryTessellationRate(RTCGeometry geometry, float tessellationRate);

/*! Sets the build quality of the geometry. */
RTCORE_API void rtcSetGeometryBuildQuality(RTCGeometry geometry, enum RTCBuildQuality quality);

/*! \brief Sets 32 bit ray mask. */
RTCORE_API void rtcSetGeometryMask(RTCGeometry geometry, unsigned int mask);

/*! \brief Sets subdivision interpolation mode for specified subdivision surface topology */
RTCORE_API void rtcSetGeometrySubdivisionMode(RTCGeometry geometry, unsigned int topologyID, enum RTCSubdivisionMode mode);

/*! \brief Binds a vertex attribute to some topology. */
RTCORE_API void rtcSetGeometryVertexAttributeTopology(RTCGeometry geometry, unsigned int vertexAttributeID, unsigned int topologyID);

RTCORE_API void rtcSetGeometryBuffer(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format,
                                     RTCBuffer buffer, size_t byteOffset, size_t byteStride, unsigned int itemCount);

/*! \brief Shares a data buffer between the application and
  Embree. The data has to remain valid as long as the mesh exists,
  and the user is responsible to free the data when the mesh gets
  deleted. For sharing the buffer, one has to specify the number of
  elements of the buffer, a byte offset to the first element, and
  byte stride of elements stored inside the buffer. The addresses
  ptr+offset+i*stride have to be aligned to 4 bytes. For vertex
  buffers the buffer has to be padded with 0 to a size of a multiple
  of 16 bytes, as Embree always accesses vertex buffers using SSE
  instructions. If this function is not called, Embree will allocate
  and manage buffers of the default layout. */
RTCORE_API void rtcSetSharedGeometryBuffer(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format,
                                           const void* ptr, size_t byteOffset, size_t byteStride, unsigned int itemCount);

RTCORE_API void* rtcSetNewGeometryBuffer(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot, enum RTCFormat format,
                                         size_t byteStride, unsigned int itemCount);

/*! Returns a pointer to the buffer data. */
RTCORE_API void* rtcGetGeometryBufferData(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot);

/*! \brief Enable geometry. Enabled geometry can be hit by a ray. */
RTCORE_API void rtcEnableGeometry(RTCGeometry geometry);

/*! \brief Update specific geometry buffer. 

  Each time geometry buffers got modified, the user has to call some
  update function to tell the ray tracing engine which buffers got
  modified. The rtcUpdateGeometryBuffer function tags a specific buffer of
  some geometry as modified. */
RTCORE_API void rtcUpdateGeometryBuffer(RTCGeometry geometry, enum RTCBufferType type, unsigned int slot);

/*! \brief Disable geometry. 

  Disabled geometry is not hit by any ray. Disabling and enabling
  geometry gives higher performance than deleting and recreating
  geometry. */
RTCORE_API void rtcDisableGeometry(RTCGeometry geometry);

/*! \brief Sets the displacement function. */
RTCORE_API void rtcSetGeometryDisplacementFunction(RTCGeometry geometry, RTCDisplacementFunction func);

/*! \brief Sets the intersection filter function for single rays. */
RTCORE_API void rtcSetGeometryIntersectFilterFunction(RTCGeometry geometry, RTCFilterFunctionN func);

/*! \brief Sets the occlusion filter function for single rays. */
RTCORE_API void rtcSetGeometryOccludedFilterFunction(RTCGeometry geometry, RTCFilterFunctionN func);

/*! Set pointer for user defined data per geometry. Invokations
 *  of the various user intersect and occluded functions get passed
 *  this data pointer when called. */
RTCORE_API void rtcSetGeometryUserData(RTCGeometry geometry, void* ptr);

/*! Get pointer for user defined data per geometry based on geomID. */
RTCORE_API void* rtcGetGeometryUserData(RTCGeometry geometry);

/*! Interpolates user data to some u/v location. The data buffer
 *  specifies per vertex data to interpolate and can be one of the
 *  RTC_VERTEX_BUFFER0/1 or RTC_USER_VERTEX_BUFFER0/1 and has to
 *  contain valueCount floating point values to interpolate for each
 *  vertex of the geometry. The P array will get filled with the
 *  interpolated datam the dPdu and dPdv arrays with the u and v
 *  derivative of the interpolation, and the ddPdudu, ddPdvdv, and
 *  ddPdudv arrays with the respective second derivatives. One can
 *  disable 1) the calculation of the interpolated value by setting P
 *  to NULL, 2) the calculation of the 1st order derivatives by
 *  setting dPdu and dPdv to NULL, 3) the calculation of the second
 *  order derivatives by setting ddPdudu, ddPdvdv, and ddPdudv to
 *  NULL. The buffers have to be padded at the end such that the last
 *  element can be read or written safely using SSE instructions. */
struct RTCInterpolateArguments
{
  RTCGeometry geometry;
  unsigned int primID;
  float u, v;
  enum RTCBufferType bufferType;
  unsigned int bufferSlot;
  float* P;
  float* dPdu;
  float* dPdv;
  float* ddPdudu;
  float* ddPdvdv;
  float* ddPdudv;
  unsigned int valueCount;
};
  
RTCORE_API void rtcInterpolate(const struct RTCInterpolateArguments* const args);

RTCORE_FORCEINLINE void rtcInterpolate0(RTCGeometry geometry, unsigned int primID, float u, float v, enum RTCBufferType bufferType, unsigned int bufferSlot, float* P, unsigned int valueCount)
{
  struct RTCInterpolateArguments args;
  args.geometry = geometry;
  args.primID = primID;
  args.u = u;
  args.v = v;
  args.bufferType = bufferType;
  args.bufferSlot = bufferSlot;
  args.P = P;
  args.dPdu = NULL;
  args.dPdv = NULL;
  args.ddPdudu = NULL;
  args.ddPdvdv = NULL;
  args.ddPdudv = NULL;
  args.valueCount = valueCount;
  rtcInterpolate(&args);
}

RTCORE_FORCEINLINE void rtcInterpolate1(RTCGeometry geometry, unsigned int primID, float u, float v, enum RTCBufferType bufferType, unsigned int bufferSlot,
                                         float* P, float* dPdu, float* dPdv, unsigned int valueCount)
{
  struct RTCInterpolateArguments args;
  args.geometry = geometry;
  args.primID = primID;
  args.u = u;
  args.v = v;
  args.bufferType = bufferType;
  args.bufferSlot = bufferSlot;
  args.P = P;
  args.dPdu = dPdu;
  args.dPdv = dPdv;
  args.ddPdudu = NULL;
  args.ddPdvdv = NULL;
  args.ddPdudv = NULL;
  args.valueCount = valueCount;
  rtcInterpolate(&args);
}

RTCORE_FORCEINLINE void rtcInterpolate2(RTCGeometry geometry, unsigned int primID, float u, float v, enum RTCBufferType bufferType, unsigned int bufferSlot,
                                         float* P, float* dPdu, float* dPdv, float* ddPdudu, float* ddPdvdv, float* ddPdudv, unsigned int valueCount)
{
  struct RTCInterpolateArguments args;
  args.geometry = geometry;
  args.primID = primID;
  args.u = u;
  args.v = v;
  args.bufferType = bufferType;
  args.bufferSlot = bufferSlot;
  args.P = P;
  args.dPdu = dPdu;
  args.dPdv = dPdv;
  args.ddPdudu = ddPdudu;
  args.ddPdvdv = ddPdvdv;
  args.ddPdudv = ddPdudv;
  args.valueCount = valueCount;
  rtcInterpolate(&args);
}

/*! Interpolates user data to an array of u/v locations. The valid
 *  pointer points to an integer array that specified which entries in
 *  the u/v arrays are valid (-1 denotes valid, and 0 invalid). If the
 *  valid pointer is NULL all elements are considers valid. The data
 *  buffer specifies per vertex data to interpolate and can be one of
 *  the RTC_VERTEX_BUFFER0/1 or RTC_USER_VERTEX_BUFFER0/1 and has to
 *  contain valueCount floating point values to interpolate for each
 *  vertex of the geometry. The P array will get filled with the
 *  interpolated datam the dPdu and dPdv arrays with the u and v
 *  derivative of the interpolation, and the ddPdudu, ddPdvdv, and
 *  ddPdudv arrays with the respective second derivatives. One can
 *  disable 1) the calculation of the interpolated value by setting P
 *  to NULL, 2) the calculation of the 1st order derivatives by
 *  setting dPdu and dPdv to NULL, 3) the calculation of the second
 *  order derivatives by setting ddPdudu, ddPdvdv, and ddPdudv to
 *  NULL. These destination arrays are filled in structure of array
 *  (SoA) layout. The buffer has to be padded at the end such that
 *  the last element can be read safely using SSE
 *  instructions. */
struct RTCInterpolateNArguments
{
  RTCGeometry geometry;
  const void* valid;
  const unsigned int* primIDs;
  const float* u;
  const float* v;
  unsigned int numUVs;
  enum RTCBufferType bufferType;
  unsigned int bufferSlot;
  float* P;
  float* dPdu;
  float* dPdv;
  float* ddPdudu;
  float* ddPdvdv;
  float* ddPdudv;
  unsigned int valueCount;
};

RTCORE_API void rtcInterpolateN(const struct RTCInterpolateNArguments* const args);

/*! Commits the geometry. */
RTCORE_API void rtcCommitGeometry(RTCGeometry geometry);

/*! \brief Attaches the geometry to some scene. */
RTCORE_API unsigned int rtcAttachGeometry(RTCScene scene, RTCGeometry geometry);

/*! \brief Attaches the geometry to some scene using the specified geometry ID. */
RTCORE_API void rtcAttachGeometryByID(RTCScene scene, RTCGeometry geometry, unsigned int geomID);

/*! \brief Detaches the geometry from the scene. */
RTCORE_API void rtcDetachGeometry(RTCScene scene, unsigned int geomID);

/*! Retains the geometry (increments reference count). */
RTCORE_API void rtcRetainGeometry(RTCGeometry geometry);

/*! Releases the geometry (decrements reference count) */
RTCORE_API void rtcReleaseGeometry(RTCGeometry geometry);

/*! Returns RTCGeometry from scene and geomID. */
RTCORE_API RTCGeometry rtcGetGeometry(RTCScene scene, unsigned int geomID);

#if defined(__cplusplus)
}
#endif
  
/*! @} */
