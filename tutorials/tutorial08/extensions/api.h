// ======================================================================== //
// Copyright 2009-2014 Intel Corporation                                    //
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

#include "tutorial/tutorial_device.h"

/*! Specifies the buffer type when mapping buffers into user-space. */
enum RTCXBufferType {

    RTCX_INDEX_BUFFER    = 0x01000000,
    RTCX_VERTEX_BUFFER   = 0x02000000,
    RTCX_VERTEX_BUFFER0  = 0x02000000,
    RTCX_VERTEX_BUFFER1  = 0x02000001,
    RTCX_OFFSET_BUFFER   = 0x03000000,
    RTCX_DATA_BUFFER0    = 0x04000000,
    RTCX_DATA_BUFFER1    = 0x04000001,

};

/*!
 *  Create a new subdivision mesh.
 *
 *  This mesh requires manifold surfaces with a consistent vertex winding
 *  order.  Faces can contain a varying number of vertices, up to a limit
 *  of pow(2, HALF_EDGE_VERTEX_BITS).
 *
 *  Data can be written into the mesh by mapping the corresponding buffer.
 *  The mesh data layers, element types, and buffer names are as follows.
 *
 *      Vertex coordinates   3 32-bit floats    RTC_VERTEX_BUFFER
 *      Vertex indices       1 32-bit integer   RTC_INDEX_BUFFER
 *      Vertex offsets       1 32-bit integer   RTC_OFFSET_BUFFER
 *      Crease weights       1 32-bit float     RTC_DATA_BUFFER0
 *      Face holes           1  8-bit integer   RTC_DATA_BUFFER1
 *
 *  Since the number of vertices per face is allowed to vary, the offset
 *  buffer is used to delimit entries in the index buffer per face.  For
 *  example, the ith entry in the offset buffer is the offset into the
 *  index buffer of the first vertex index of face i.  The size of the
 *  offset buffer is numFaces + 1.  Face holes are boolean values that
 *  denote whether a face is a hole (1) or not (0).
 */
unsigned rtcxNewSubdivisionMesh(RTCScene sceneID, RTCGeometryFlags flags, size_t numFaces, size_t numEdges, size_t numVertices);

/*!
 *  Commit the geometry of the scene.  After initializing or modifying
 *  geometries, this function must be called before tracing rays.
 */
void rtcxCommit(RTCScene sceneID);

/*!
 *  Return a pointer to a buffer associated with a mesh.  This function
 *  can be used to access the index, offset, vertex, and data buffers.
 */
void *rtcxMapBuffer(RTCScene sceneID, unsigned geomID, RTCXBufferType type);

/*!
 *  Return control of a mesh buffer to Embree.  A buffer must be unmapped
 *  before calls to rtcEnable, rtcDisable, rtcUpdate, or rtcDeleteGeometry.
 */
inline void rtcxUnmapBuffer(RTCScene sceneID, unsigned geomID, RTCXBufferType type) {}

