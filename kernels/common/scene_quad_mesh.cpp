// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "scene_quad_mesh.h"
#include "scene.h"

namespace embree
{
#if defined(EMBREE_LOWEST_ISA)

  QuadMesh::QuadMesh (Device* device)
    : Geometry(device,GTY_QUAD_MESH,0,1)
  {
    vertices.resize(numTimeSteps);
  }

  void QuadMesh::setMask (unsigned mask) 
  {
    this->mask = mask; 
    Geometry::update();
  }

  void QuadMesh::setNumTimeSteps (unsigned int numTimeSteps)
  {
    vertices.resize(numTimeSteps);
    Geometry::setNumTimeSteps(numTimeSteps);
  }

  void QuadMesh::setVertexAttributeCount (unsigned int N)
  {
    vertexAttribs.resize(N);
    Geometry::update();
  }
  
  void QuadMesh::setBuffer(RTCBufferType type, unsigned int slot, RTCFormat format, const Ref<Buffer>& buffer, size_t offset, size_t stride, unsigned int num)
  { 
    /* verify that all accesses are 4 bytes aligned */
    if (((size_t(buffer->getPtr()) + offset) & 0x3) || (stride & 0x3)) 
      throw_RTCError(RTC_ERROR_INVALID_OPERATION, "data must be 4 bytes aligned");

    if (type == RTC_BUFFER_TYPE_VERTEX) 
    {
      if (format != RTC_FORMAT_FLOAT3)
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid vertex buffer format");

      /* if buffer is larger than 16GB the premultiplied index optimization does not work */
      if (stride*num > 16ll*1024ll*1024ll*1024ll)
       throw_RTCError(RTC_ERROR_INVALID_OPERATION, "vertex buffer can be at most 16GB large");

      if (slot >= vertices.size())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid vertex buffer slot");

      vertices[slot].set(buffer, offset, stride, num, format);
      vertices[slot].checkPadding16();
      vertices0 = vertices[0];
    } 
    else if (type >= RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE)
    {
      if (format < RTC_FORMAT_FLOAT || format > RTC_FORMAT_FLOAT16)
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid vertex attribute buffer format");

      if (slot >= vertexAttribs.size())
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid vertex attribute buffer slot");
      
      vertexAttribs[slot].set(buffer, offset, stride, num, format);
      vertexAttribs[slot].checkPadding16();
    }
    else if (type == RTC_BUFFER_TYPE_INDEX)
    {
      if (slot != 0)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      if (format != RTC_FORMAT_UINT4)
        throw_RTCError(RTC_ERROR_INVALID_OPERATION, "invalid index buffer format");

      quads.set(buffer, offset, stride, num, format);
      setNumPrimitives(num);
    }
    else
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "unknown buffer type");
  }

  void* QuadMesh::getBuffer(RTCBufferType type, unsigned int slot)
  {
    if (type == RTC_BUFFER_TYPE_INDEX)
    {
      if (slot != 0)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      return quads.getPtr();
    }
    else if (type == RTC_BUFFER_TYPE_VERTEX)
    {
      if (slot >= vertices.size())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      return vertices[slot].getPtr();
    }
    else if (type == RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE)
    {
      if (slot >= vertexAttribs.size())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      return vertexAttribs[slot].getPtr();
    }
    else
    {
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "unknown buffer type");
      return nullptr;
    }
  }

  void QuadMesh::updateBuffer(RTCBufferType type, unsigned int slot)
  {
    if (type == RTC_BUFFER_TYPE_INDEX)
    {
      if (slot != 0)
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      quads.setModified();
    }
    else if (type == RTC_BUFFER_TYPE_VERTEX)
    {
      if (slot >= vertices.size())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      vertices[slot].setModified();
    }
    else if (type == RTC_BUFFER_TYPE_VERTEX_ATTRIBUTE)
    {
      if (slot >= vertexAttribs.size())
        throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "invalid buffer slot");
      vertexAttribs[slot].setModified();
    }
    else
    {
      throw_RTCError(RTC_ERROR_INVALID_ARGUMENT, "unknown buffer type");
    }

    Geometry::update();
  }

  void QuadMesh::commit() 
  {
    /* verify that stride of all time steps are identical */
    for (unsigned int t=0; t<numTimeSteps; t++)
      if (vertices[t].getStride() != vertices[0].getStride())
        throw_RTCError(RTC_ERROR_INVALID_OPERATION,"stride of vertex buffers have to be identical for each time step");

    Geometry::commit();
  }

  void QuadMesh::addElementsToCount (GeometryCounts & counts) const
  {
    if (numTimeSteps == 1) counts.numQuads += numPrimitives;
    else                   counts.numMBQuads += numPrimitives;
  }

  bool QuadMesh::verify() 
  {
    /*! verify consistent size of vertex arrays */
    if (vertices.size() == 0) return false;
    for (const auto& buffer : vertices)
      if (buffer.size() != numVertices())
        return false;

    /*! verify quad indices */
    for (size_t i=0; i<size(); i++) {     
      if (quads[i].v[0] >= numVertices()) return false; 
      if (quads[i].v[1] >= numVertices()) return false; 
      if (quads[i].v[2] >= numVertices()) return false; 
      if (quads[i].v[3] >= numVertices()) return false; 
    }

    /*! verify vertices */
    for (const auto& buffer : vertices)
      for (size_t i=0; i<buffer.size(); i++)
	if (!isvalid(buffer[i])) 
	  return false;

    return true;
  }

  void QuadMesh::interpolate(const RTCInterpolateArguments* const args) {
    interpolate_impl<4>(args);
  }

  size_t QuadMesh::getGeometryDataDeviceByteSize() const {
    size_t byte_size = sizeof(QuadMesh);
    byte_size += numTimeSteps * sizeof(BufferView<Vec3fa>);
    //if (vertexAttribs.size() > 0)
    //  byte_size += numTimeSteps * sizeof(RawBufferView);
    return 16 * ((byte_size + 15) / 16);
  }

  void QuadMesh::convertToDeviceRepresentation(size_t offset, char* data_host, char* data_device) const {
    QuadMesh* mesh = (QuadMesh*)(data_host + offset);
    std::memcpy(data_host + offset, (void*)this, sizeof(QuadMesh));
    offset += sizeof(QuadMesh);

    // store offset for overriding vertices pointer with device pointer after copying
    const size_t offsetVertices = offset;
    // copy vertices BufferViews for each time step
    for (size_t t = 0; t < numTimeSteps; ++t) {
      std::memcpy(data_host + offset, &(vertices[t]), sizeof(BufferView<Vec3fa>));
      offset += sizeof(BufferView<Vec3fa>);
    }
    // override vertices pointer with device ptr
    mesh->vertices.setDataPtr((BufferView<Vec3fa>*)(data_device + offsetVertices));

    //if (vertexAttribs.size() > 0) {
    //  const size_t offsetVertexAttribs = offset;
    //  for (size_t t = 0; t < numTimeSteps; ++t) {
    //    std::memcpy(data_host + offset, &(vertexAttribs[t]), sizeof(RawBufferView));
    //    offset += sizeof(RawBufferView);
    //  }
    //  mesh->vertexAttribs.setDataPtr((RawBufferView*)(data_device + offsetVertexAttribs));
    //}
  }

#endif

  namespace isa
  {
    QuadMesh* createQuadMesh(Device* device) {
      return new QuadMeshISA(device);
    }
  }
}
