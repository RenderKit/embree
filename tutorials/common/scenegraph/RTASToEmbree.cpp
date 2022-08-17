// ======================================================================== //
//                                                                          //
// Copyright 2009-2019 Intel Corporation                                    //
//                                                                          //
// This program is the sole property of the Intel Corporation and           //
// contains proprietary and confidential information. Unauthorized          //
// use or distribution will be subject to action as prescribed by           //
// the license agreement.                                                   //
//                                                                          //
// ======================================================================== //

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "RTASToEmbree.h"
#include <vector>
#include <algorithm>
#include <assert.h>
#include <math.h>

#include <stdlib.h>
#if defined(__WIN32__)
#include <intrin.h>
#endif


namespace RTASFile
{
  
  Ref<SceneGraph::TriangleMeshNode> GeoToEmbree(  RTASFile::Geo& geo )
  {
      if (geo.Type == RTASFile::GEOMETRY_TYPE_PROCEDURAL)
          return Ref<SceneGraph::TriangleMeshNode>(nullptr);

      RTASFile::GeometryTriangles& tris = geo.Desc.Triangles;

      avector< SceneGraph::TriangleMeshNode::Vertex > mesh_verts;
      std::vector< SceneGraph::TriangleMeshNode::Triangle > mesh_tris;

      size_t nTris = RTASFile::CountGeoPrimitives(geo);
      mesh_verts.resize(tris.VertexCount);
      mesh_tris.resize(nTris);

      static_assert( sizeof( SceneGraph::TriangleMeshNode::Vertex) == 16, "Congratulations!  You get to maintain this code!" );
      RTASFile::ConvertVerticesToF32a((float*)mesh_verts.data(), tris.vertexFormat, tris.pVertexBuffer, tris.VertexCount, tris.VertexBufferByteStride );
      RTASFile::ConvertIndicesToU32((uint32_t*)mesh_tris.data(), tris.indexFormat, tris.pIndexBuffer, 3 * nTris);

      if (geo.Desc.Triangles.pTransform != nullptr)
      {
          for( size_t i=0; i<geo.Desc.Triangles.VertexCount; i++ )
          {
              float* p = &mesh_verts[i].x;
              RTASFile::TransformPosition( p, p, *geo.Desc.Triangles.pTransform );
          }
          geo.Desc.Triangles.pTransform = nullptr;
      }

      avector<SceneGraph::TriangleMeshNode::Vertex> normals;
      std::vector<Vec2f> texcoords;
      SceneGraph::TriangleMeshNode* pNode = new SceneGraph::TriangleMeshNode(mesh_verts, normals, texcoords, mesh_tris, new OBJMaterial("default"));
      return Ref<SceneGraph::TriangleMeshNode>(pNode);

  }
   
}
