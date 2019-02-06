// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "geometry_creation.h"

namespace embree
{
  Ref<SceneGraph::Node> SceneGraph::createTrianglePlane (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, Ref<MaterialNode> material)
  {
    Ref<SceneGraph::TriangleMeshNode> mesh = new SceneGraph::TriangleMeshNode(material,BBox1f(0,1),1);
    mesh->positions[0].resize((width+1)*(height+1));
    mesh->triangles.resize(2*width*height);

    for (size_t y=0; y<=height; y++) {
      for (size_t x=0; x<=width; x++) {
        Vec3fa p = p0+float(x)/float(width)*dx+float(y)/float(height)*dy;
        size_t i = y*(width+1)+x;
        mesh->positions[0][i].x = p.x;
        mesh->positions[0][i].y = p.y;
        mesh->positions[0][i].z = p.z;
      }
    }
    for (size_t y=0; y<height; y++) {
      for (size_t x=0; x<width; x++) {
        size_t i = 2*y*width+2*x;
        size_t p00 = (y+0)*(width+1)+(x+0);
        size_t p01 = (y+0)*(width+1)+(x+1);
        size_t p10 = (y+1)*(width+1)+(x+0);
        size_t p11 = (y+1)*(width+1)+(x+1);
        mesh->triangles[i+0].v0 = unsigned(p00); mesh->triangles[i+0].v1 = unsigned(p01); mesh->triangles[i+0].v2 = unsigned(p10);
        mesh->triangles[i+1].v0 = unsigned(p11); mesh->triangles[i+1].v1 = unsigned(p10); mesh->triangles[i+1].v2 = unsigned(p01);
      }
    }
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createQuadPlane (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, Ref<MaterialNode> material)
  {
    Ref<SceneGraph::QuadMeshNode> mesh = new SceneGraph::QuadMeshNode(material,BBox1f(0,1),1);
    mesh->positions[0].resize((width+1)*(height+1));
    mesh->quads.resize(width*height);

    for (size_t y=0; y<=height; y++) {
      for (size_t x=0; x<=width; x++) {
        Vec3fa p = p0+float(x)/float(width)*dx+float(y)/float(height)*dy;
        size_t i = y*(width+1)+x;
        mesh->positions[0][i].x = p.x;
        mesh->positions[0][i].y = p.y;
        mesh->positions[0][i].z = p.z;
      }
    }
    for (size_t y=0; y<height; y++) {
      for (size_t x=0; x<width; x++) {
        size_t i = y*width+x;
        size_t p00 = (y+0)*(width+1)+(x+0);
        size_t p01 = (y+0)*(width+1)+(x+1);
        size_t p10 = (y+1)*(width+1)+(x+0);
        size_t p11 = (y+1)*(width+1)+(x+1);
        mesh->quads[i].v0 = unsigned(p00); 
        mesh->quads[i].v1 = unsigned(p01); 
        mesh->quads[i].v2 = unsigned(p11); 
        mesh->quads[i].v3 = unsigned(p10);
      }
    }
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createGridPlane (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, Ref<MaterialNode> material)
  {
    Ref<SceneGraph::GridMeshNode> mesh = new SceneGraph::GridMeshNode(material,BBox1f(0,1),1);
    mesh->positions[0].resize((width+1)*(height+1));
    mesh->grids.push_back(SceneGraph::GridMeshNode::Grid(0,width+1,width+1,height+1));

    for (size_t y=0; y<=height; y++) {
      for (size_t x=0; x<=width; x++) {
        Vec3fa p = p0+float(x)/float(width)*dx+float(y)/float(height)*dy;
        size_t i = y*(width+1)+x;
        mesh->positions[0][i].x = p.x;
        mesh->positions[0][i].y = p.y;
        mesh->positions[0][i].z = p.z;
      }
    }
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createSubdivPlane (const Vec3fa& p0, const Vec3fa& dx, const Vec3fa& dy, size_t width, size_t height, float tessellationRate, Ref<MaterialNode> material)
  {
    Ref<SceneGraph::SubdivMeshNode> mesh = new SceneGraph::SubdivMeshNode(material,BBox1f(0,1),1);
    mesh->tessellationRate = tessellationRate;
    mesh->positions[0].resize((width+1)*(height+1));
    mesh->position_indices.resize(4*width*height);
    mesh->verticesPerFace.resize(width*height);

    for (size_t y=0; y<=height; y++) {
      for (size_t x=0; x<=width; x++) {
        Vec3fa p = p0+float(x)/float(width)*dx+float(y)/float(height)*dy;
        size_t i = y*(width+1)+x;
        mesh->positions[0][i].x = p.x;
        mesh->positions[0][i].y = p.y;
        mesh->positions[0][i].z = p.z;
      }
    }
    for (size_t y=0; y<height; y++) {
      for (size_t x=0; x<width; x++) {
        size_t i = y*width+x;
        size_t p00 = (y+0)*(width+1)+(x+0);
        size_t p01 = (y+0)*(width+1)+(x+1);
        size_t p10 = (y+1)*(width+1)+(x+0);
        size_t p11 = (y+1)*(width+1)+(x+1);
        mesh->position_indices[4*i+0] = unsigned(p00); 
        mesh->position_indices[4*i+1] = unsigned(p01); 
        mesh->position_indices[4*i+2] = unsigned(p11); 
        mesh->position_indices[4*i+3] = unsigned(p10);
        mesh->verticesPerFace[i] = 4;
      }
    }
    mesh->position_subdiv_mode = RTC_SUBDIVISION_MODE_PIN_CORNERS;
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createTriangleSphere (const Vec3fa& center, const float radius, size_t N, Ref<MaterialNode> material)
  {
    unsigned numPhi = unsigned(N);
    unsigned numTheta = 2*numPhi;
    unsigned numVertices = numTheta*(numPhi+1);
    Ref<SceneGraph::TriangleMeshNode> mesh = new SceneGraph::TriangleMeshNode(material,BBox1f(0,1),1);
    mesh->positions[0].resize(numVertices);

    /* create sphere geometry */
    const float rcpNumTheta = rcp(float(numTheta));
    const float rcpNumPhi   = rcp(float(numPhi));
    for (unsigned int phi=0; phi<=numPhi; phi++)
    {
      for (unsigned int theta=0; theta<numTheta; theta++)
      {
	const float phif   = phi*float(pi)*rcpNumPhi;
	const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
	mesh->positions[0][phi*numTheta+theta].x = center.x + radius*sin(phif)*sin(thetaf);
        mesh->positions[0][phi*numTheta+theta].y = center.y + radius*cos(phif);
	mesh->positions[0][phi*numTheta+theta].z = center.z + radius*sin(phif)*cos(thetaf);
      }
      if (phi == 0) continue;
      
      if (phi == 1)
      {
	for (unsigned int theta=1; theta<=numTheta; theta++) 
	{
	  unsigned int p00 = numTheta-1;
	  unsigned int p10 = phi*numTheta+theta-1;
	  unsigned int p11 = phi*numTheta+theta%numTheta;
          mesh->triangles.push_back(TriangleMeshNode::Triangle(p10,p00,p11));
	}
      }
      else if (phi == numPhi)
      {
	for (unsigned int theta=1; theta<=numTheta; theta++) 
	{
	  unsigned int p00 = (phi-1)*numTheta+theta-1;
	  unsigned int p01 = (phi-1)*numTheta+theta%numTheta;
	  unsigned int p10 = numPhi*numTheta;
          mesh->triangles.push_back(TriangleMeshNode::Triangle(p10,p00,p01));
	}
      }
      else
      {
	for (unsigned int theta=1; theta<=numTheta; theta++) 
	{
	  unsigned int p00 = (phi-1)*numTheta+theta-1;
	  unsigned int p01 = (phi-1)*numTheta+theta%numTheta;
	  unsigned int p10 = phi*numTheta+theta-1;
	  unsigned int p11 = phi*numTheta+theta%numTheta;
          mesh->triangles.push_back(TriangleMeshNode::Triangle(p10,p00,p11));
          mesh->triangles.push_back(TriangleMeshNode::Triangle(p01,p11,p00));
	}
      }
    }
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createQuadSphere (const Vec3fa& center, const float radius, size_t N, Ref<MaterialNode> material)
  {
    unsigned numPhi = unsigned(N);
    unsigned numTheta = 2*numPhi;
    unsigned numVertices = numTheta*(numPhi+1);
    Ref<SceneGraph::QuadMeshNode> mesh = new SceneGraph::QuadMeshNode(material,BBox1f(0,1),1);
    mesh->positions[0].resize(numVertices);

    /* create sphere geometry */
    const float rcpNumTheta = rcp(float(numTheta));
    const float rcpNumPhi   = rcp(float(numPhi));
    for (unsigned int phi=0; phi<=numPhi; phi++)
    {
      for (unsigned int theta=0; theta<numTheta; theta++)
      {
	const float phif   = phi*float(pi)*rcpNumPhi;
	const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
	mesh->positions[0][phi*numTheta+theta].x = center.x + radius*sin(phif)*sin(thetaf);
        mesh->positions[0][phi*numTheta+theta].y = center.y + radius*cos(phif);
	mesh->positions[0][phi*numTheta+theta].z = center.z + radius*sin(phif)*cos(thetaf);
      }
      if (phi == 0) continue;
      
      if (phi == 1)
      {
	for (unsigned int theta=1; theta<=numTheta; theta++) 
	{
	  unsigned int p00 = numTheta-1;
	  unsigned int p10 = phi*numTheta+theta-1;
	  unsigned int p11 = phi*numTheta+theta%numTheta;
          mesh->quads.push_back(QuadMeshNode::Quad(p10,p00,p11,p11));
	}
      }
      else if (phi == numPhi)
      {
	for (unsigned int theta=1; theta<=numTheta; theta++) 
	{
	  unsigned int p00 = (phi-1)*numTheta+theta-1;
	  unsigned int p01 = (phi-1)*numTheta+theta%numTheta;
	  unsigned int p10 = numPhi*numTheta;
          mesh->quads.push_back(QuadMeshNode::Quad(p10,p00,p01,p01));
	}
      }
      else
      {
	for (unsigned int theta=1; theta<=numTheta; theta++) 
	{
	  unsigned int p00 = (phi-1)*numTheta+theta-1;
	  unsigned int p01 = (phi-1)*numTheta+theta%numTheta;
	  unsigned int p10 = phi*numTheta+theta-1;
	  unsigned int p11 = phi*numTheta+theta%numTheta;
          mesh->quads.push_back(QuadMeshNode::Quad(p10,p00,p01,p11));
	}
      }
    }
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createGridSphere (const Vec3fa& center, const float radius, size_t N, Ref<MaterialNode> material)
  {
    size_t grid_size = (N+1)*(N+1);
    Ref<SceneGraph::GridMeshNode> mesh = new SceneGraph::GridMeshNode(material,BBox1f(0,1),1);
    mesh->positions[0].resize(grid_size*6);

    for (size_t i=0; i<6; i++)
    {
      mesh->grids.push_back(SceneGraph::GridMeshNode::Grid(i*grid_size,N+1,N+1,N+1));
      Vec3fa p0, dx, dy;
      switch (i) {
      case 0: p0 = Vec3fa(-0.5f,-0.5f,-0.5f); dx = Vec3fa(+1,0, 0); dy = Vec3fa(0,1,0); break;
      case 1: p0 = Vec3fa(+0.5f,-0.5f,-0.5f); dx = Vec3fa( 0,0,+1); dy = Vec3fa(0,1,0); break;
      case 2: p0 = Vec3fa(+0.5f,-0.5f,+0.5f); dx = Vec3fa(-1,0, 0); dy = Vec3fa(0,1,0); break;
      case 3: p0 = Vec3fa(-0.5f,-0.5f,+0.5f); dx = Vec3fa( 0,0,-1); dy = Vec3fa(0,1,0); break;
      case 4: p0 = Vec3fa(-0.5f,-0.5f,-0.5f); dx = Vec3fa( 0,0,+1); dy = Vec3fa(1,0,0); break;
      case 5: p0 = Vec3fa(-0.5f,+0.5f,-0.5f); dx = Vec3fa( 1,0, 0); dy = Vec3fa(0,0,1); break;
      default: assert(false);
      }
      
      for (size_t y=0; y<=N; y++) {
        for (size_t x=0; x<=N; x++) {
          size_t j = i*grid_size + y*(N+1) + x;
          Vec3fa p = p0+float(x)/float(N)*dx+float(y)/float(N)*dy;
          mesh->positions[0][j] = center + radius*normalize(p);
        }
      }
    }
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createSubdivSphere (const Vec3fa& center, const float radius, size_t N, float tessellationRate, Ref<MaterialNode> material)
  {
    unsigned numPhi = unsigned(N);
    unsigned numTheta = 2*numPhi;
    unsigned numVertices = numTheta*(numPhi+1);
    Ref<SceneGraph::SubdivMeshNode> mesh = new SceneGraph::SubdivMeshNode(material,BBox1f(0,1),1);
    mesh->tessellationRate = tessellationRate;
    mesh->positions[0].resize(numVertices);

    /* create sphere geometry */
    const float rcpNumTheta = rcp((float)numTheta);
    const float rcpNumPhi   = rcp((float)numPhi);
    for (unsigned int phi=0; phi<=numPhi; phi++)
    {
      for (unsigned int theta=0; theta<numTheta; theta++)
      {
	const float phif   = phi*float(pi)*rcpNumPhi;
	const float thetaf = theta*2.0f*float(pi)*rcpNumTheta;
	mesh->positions[0][phi*numTheta+theta].x = center.x + radius*sin(phif)*sin(thetaf);
        mesh->positions[0][phi*numTheta+theta].y = center.y + radius*cos(phif);
	mesh->positions[0][phi*numTheta+theta].z = center.z + radius*sin(phif)*cos(thetaf);
      }
      if (phi == 0) continue;
      
      if (phi == 1)
      {
	for (unsigned int theta=1; theta<=numTheta; theta++) 
	{
	  unsigned int p00 = numTheta-1;
	  unsigned int p10 = phi*numTheta+theta-1;
	  unsigned int p11 = phi*numTheta+theta%numTheta;
          mesh->verticesPerFace.push_back(3);
          mesh->position_indices.push_back(p10);
          mesh->position_indices.push_back(p00);
          mesh->position_indices.push_back(p11);
	}
      }
      else if (phi == numPhi)
      {
	for (unsigned int theta=1; theta<=numTheta; theta++) 
	{
	  unsigned int p00 = (phi-1)*numTheta+theta-1;
	  unsigned int p01 = (phi-1)*numTheta+theta%numTheta;
	  unsigned int p10 = numPhi*numTheta;
          mesh->verticesPerFace.push_back(3);
          mesh->position_indices.push_back(p10);
          mesh->position_indices.push_back(p00);
          mesh->position_indices.push_back(p01);
	}
      }
      else
      {
	for (unsigned int theta=1; theta<=numTheta; theta++) 
	{
	  unsigned int p00 = (phi-1)*numTheta+theta-1;
	  unsigned int p01 = (phi-1)*numTheta+theta%numTheta;
	  unsigned int p10 = phi*numTheta+theta-1;
	  unsigned int p11 = phi*numTheta+theta%numTheta;
          mesh->verticesPerFace.push_back(4);
          mesh->position_indices.push_back(p10);
          mesh->position_indices.push_back(p00);
          mesh->position_indices.push_back(p01);
          mesh->position_indices.push_back(p11);
	}
      }
    }
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createSphereShapedHair(const Vec3fa& center, const float radius, Ref<MaterialNode> material)
  {
    Ref<SceneGraph::HairSetNode> mesh = new SceneGraph::HairSetNode(RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE,material,BBox1f(0,1),1);
    mesh->hairs.push_back(SceneGraph::HairSetNode::Hair(0,0));
    mesh->positions[0].push_back(Vec3fa(center+Vec3fa(-radius,0,0),radius));
    mesh->positions[0].push_back(Vec3fa(center+Vec3fa(0,0,0),radius));
    mesh->positions[0].push_back(Vec3fa(center+Vec3fa(0,0,0),radius));
    mesh->positions[0].push_back(Vec3fa(center+Vec3fa(+radius,0,0),radius));
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createPointSphere (const Vec3fa& center, const float radius, const float pointRadius,
                                                       size_t N, PointSubtype subtype, Ref<MaterialNode> material)
  {
    unsigned numPhi = unsigned(N);
    unsigned numTheta = 2 * numPhi;
    unsigned numVertices = numTheta * (numPhi + 1);

    RTCGeometryType type;
    switch (subtype) {
      case SPHERE:
        type = RTC_GEOMETRY_TYPE_SPHERE_POINT;
        break;
      case DISC:
        type = RTC_GEOMETRY_TYPE_DISC_POINT;
        break;
      case ORIENTED_DISC:
        type = RTC_GEOMETRY_TYPE_ORIENTED_DISC_POINT;
        break;
    }

    Ref<SceneGraph::PointSetNode> mesh = new SceneGraph::PointSetNode(type, material, BBox1f(0,1), 1);
    mesh->positions[0].resize(numVertices);
    if (subtype == ORIENTED_DISC) {
      mesh->normals.push_back(avector<PointSetNode::Vertex>());
      mesh->normals[0].resize(numVertices);
    }

    /* create sphere geometry */
    const float rcpNumTheta = rcp(float(numTheta));
    const float rcpNumPhi   = rcp(float(numPhi));
    for (unsigned int phi = 0; phi <= numPhi; phi++)
    {
      for (unsigned int theta = 0; theta < numTheta; theta++)
      {
        const float phif   = phi * float(pi) * rcpNumPhi;
        const float thetaf = theta * 2.0f * float(pi) * rcpNumTheta;
        mesh->positions[0][phi * numTheta + theta].x = center.x + radius * sin(phif) * sin(thetaf);
        mesh->positions[0][phi * numTheta + theta].y = center.y + radius * cos(phif);
        mesh->positions[0][phi * numTheta + theta].z = center.z + radius * sin(phif) * cos(thetaf);
        mesh->positions[0][phi * numTheta + theta].w = pointRadius;
        if (subtype == ORIENTED_DISC)
          mesh->normals[0][phi * numTheta + theta] =
            normalize(mesh->positions[0][phi * numTheta + theta] - center);
      }
    }
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createHairyPlane (int hash, const Vec3fa& pos, const Vec3fa& dx, const Vec3fa& dy, const float len, const float r, size_t numHairs, CurveSubtype subtype, Ref<MaterialNode> material)
  {
    RandomSampler sampler;
    RandomSampler_init(sampler,hash);

    RTCGeometryType type = (subtype == ROUND_CURVE) ? RTC_GEOMETRY_TYPE_ROUND_BEZIER_CURVE : RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE;
    Ref<SceneGraph::HairSetNode> mesh = new SceneGraph::HairSetNode(type,material,BBox1f(0,1),1);

    if (numHairs == 1) {
      const Vec3fa p0 = pos;
      const Vec3fa p1 = p0 + len*Vec3fa(1,0,0);
      const Vec3fa p2 = p0 + len*Vec3fa(0,1,1);
      const Vec3fa p3 = p0 + len*Vec3fa(0,1,0);
      mesh->hairs.push_back(HairSetNode::Hair(0,0));
      mesh->positions[0].push_back(Vec3fa(p0,r));
      mesh->positions[0].push_back(Vec3fa(p1,r));
      mesh->positions[0].push_back(Vec3fa(p2,r));
      mesh->positions[0].push_back(Vec3fa(p3,r));
      return mesh.dynamicCast<SceneGraph::Node>();
    }

    Vec3fa dz = cross(dx,dy);
    for (size_t i=0; i<numHairs; i++) 
    {
      const Vec3fa p0 = pos + RandomSampler_getFloat(sampler)*dx + RandomSampler_getFloat(sampler)*dy;
      const Vec3fa p1 = p0 + len*normalize(dx);
      const Vec3fa p2 = p0 + len*(normalize(dz)+normalize(dy));
      const Vec3fa p3 = p0 + len*normalize(dz);
      mesh->hairs.push_back(HairSetNode::Hair(unsigned(4*i),unsigned(i)));
      mesh->positions[0].push_back(Vec3fa(p0,r));
      mesh->positions[0].push_back(Vec3fa(p1,r));
      mesh->positions[0].push_back(Vec3fa(p2,r));
      mesh->positions[0].push_back(Vec3fa(p3,r));
    }
    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createGarbageTriangleMesh (int hash, size_t numTriangles, bool mblur, Ref<MaterialNode> material)
  {
    RandomSampler sampler;
    RandomSampler_init(sampler,hash);
    Ref<SceneGraph::TriangleMeshNode> mesh = new SceneGraph::TriangleMeshNode(material,BBox1f(0,1),mblur?2:1);

    mesh->triangles.resize(numTriangles);
    for (size_t i=0; i<numTriangles; i++) {
      const unsigned v0 = (RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(3*i+0);
      const unsigned v1 = (RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(3*i+1);
      const unsigned v2 = (RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(3*i+2);
      mesh->triangles[i] = TriangleMeshNode::Triangle(v0,v1,v2);
    }

    mesh->positions[0].resize(3*numTriangles);
    for (size_t i=0; i<3*numTriangles; i++) {
      const float x = cast_i2f(RandomSampler_getUInt(sampler));
      const float y = cast_i2f(RandomSampler_getUInt(sampler));
      const float z = cast_i2f(RandomSampler_getUInt(sampler));
      const float w = cast_i2f(RandomSampler_getUInt(sampler));
      mesh->positions[0][i] = Vec3fa(x,y,z,w);
    }

    if (mblur) 
    {
      mesh->positions[1].resize(3*numTriangles);
      for (size_t i=0; i<3*numTriangles; i++) {
        const float x = cast_i2f(RandomSampler_getUInt(sampler));
        const float y = cast_i2f(RandomSampler_getUInt(sampler));
        const float z = cast_i2f(RandomSampler_getUInt(sampler));
        const float w = cast_i2f(RandomSampler_getUInt(sampler));
        mesh->positions[1][i] = Vec3fa(x,y,z,w);
      }
    }

    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createGarbageQuadMesh (int hash, size_t numQuads, bool mblur, Ref<MaterialNode> material)
  {
    RandomSampler sampler;
    RandomSampler_init(sampler,hash);
    Ref<SceneGraph::QuadMeshNode> mesh = new SceneGraph::QuadMeshNode(material,BBox1f(0,1),mblur?2:1);

    mesh->quads.resize(numQuads);
    for (size_t i=0; i<numQuads; i++) {
      const unsigned v0 = (RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(4*i+0);
      const unsigned v1 = (RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(4*i+1);
      const unsigned v2 = (RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(4*i+2);
      const unsigned v3 = (RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(4*i+3);
      mesh->quads[i] = QuadMeshNode::Quad(v0,v1,v2,v3);
    }

    mesh->positions[0].resize(4*numQuads);
    for (size_t i=0; i<4*numQuads; i++) {
      const float x = cast_i2f(RandomSampler_getUInt(sampler));
      const float y = cast_i2f(RandomSampler_getUInt(sampler));
      const float z = cast_i2f(RandomSampler_getUInt(sampler));
      const float w = cast_i2f(RandomSampler_getUInt(sampler));
      mesh->positions[0][i] = Vec3fa(x,y,z,w);
    }

    if (mblur) 
    {
      mesh->positions[1].resize(4*numQuads);
      for (size_t i=0; i<4*numQuads; i++) {
        const float x = cast_i2f(RandomSampler_getUInt(sampler));
        const float y = cast_i2f(RandomSampler_getUInt(sampler));
        const float z = cast_i2f(RandomSampler_getUInt(sampler));
        const float w = cast_i2f(RandomSampler_getUInt(sampler));
        mesh->positions[1][i] = Vec3fa(x,y,z,w);
      }
    }

    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createGarbageGridMesh (int hash, size_t numGrids, bool mblur, Ref<MaterialNode> material)
  {
    RandomSampler sampler;
    RandomSampler_init(sampler,hash);
    Ref<SceneGraph::GridMeshNode> mesh = new SceneGraph::GridMeshNode(material,BBox1f(0,1),mblur?2:1);

    mesh->grids.resize(numGrids);
    for (size_t i=0; i<numGrids; i++) {
      const unsigned v0 = (RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(4*i+0);
      mesh->grids[i] = GridMeshNode::Grid(v0,2,2,2);
    }

    mesh->positions[0].resize(4*numGrids);
    for (size_t i=0; i<4*numGrids; i++) {
      const float x = cast_i2f(RandomSampler_getUInt(sampler));
      const float y = cast_i2f(RandomSampler_getUInt(sampler));
      const float z = cast_i2f(RandomSampler_getUInt(sampler));
      const float w = cast_i2f(RandomSampler_getUInt(sampler));
      mesh->positions[0][i] = Vec3fa(x,y,z,w);
    }

    if (mblur) 
    {
      mesh->positions[1].resize(4*numGrids);
      for (size_t i=0; i<4*numGrids; i++) {
        const float x = cast_i2f(RandomSampler_getUInt(sampler));
        const float y = cast_i2f(RandomSampler_getUInt(sampler));
        const float z = cast_i2f(RandomSampler_getUInt(sampler));
        const float w = cast_i2f(RandomSampler_getUInt(sampler));
        mesh->positions[1][i] = Vec3fa(x,y,z,w);
      }
    }

    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createGarbageLineSegments (int hash, size_t numLineSegments, bool mblur, Ref<MaterialNode> material)
  {
    RandomSampler sampler;
    RandomSampler_init(sampler,hash);
    Ref<SceneGraph::HairSetNode> mesh = new SceneGraph::HairSetNode(RTC_GEOMETRY_TYPE_FLAT_LINEAR_CURVE,material,BBox1f(0,1),mblur?2:1);

    mesh->hairs.resize(numLineSegments);
    for (size_t i=0; i<numLineSegments; i++) {
      mesh->hairs[i].vertex = (RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(2*i);
      mesh->hairs[i].id = 0;
    }

    mesh->positions[0].resize(2*numLineSegments);
    for (size_t i=0; i<2*numLineSegments; i++) {
      const float x = cast_i2f(RandomSampler_getUInt(sampler));
      const float y = cast_i2f(RandomSampler_getUInt(sampler));
      const float z = cast_i2f(RandomSampler_getUInt(sampler));
      const float r = cast_i2f(RandomSampler_getUInt(sampler));
      mesh->positions[0][i] = Vec3fa(x,y,z,r);
    }

    if (mblur) 
    {
      mesh->positions[1].resize(2*numLineSegments);
      for (size_t i=0; i<2*numLineSegments; i++) {
        const float x = cast_i2f(RandomSampler_getUInt(sampler));
        const float y = cast_i2f(RandomSampler_getUInt(sampler));
        const float z = cast_i2f(RandomSampler_getUInt(sampler));
        const float r = cast_i2f(RandomSampler_getUInt(sampler));
        mesh->positions[1][i] = Vec3fa(x,y,z,r);
      }
    }

    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createGarbageHair (int hash, size_t numHairs, bool mblur, Ref<MaterialNode> material)
  {
    RandomSampler sampler;
    RandomSampler_init(sampler,hash);
    Ref<SceneGraph::HairSetNode> mesh = new SceneGraph::HairSetNode(RTC_GEOMETRY_TYPE_FLAT_BEZIER_CURVE,material,BBox1f(0,1),mblur?2:1);

    mesh->hairs.resize(numHairs);
    for (size_t i=0; i<numHairs; i++) {
      const unsigned v0 = (RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(4*i);
      mesh->hairs[i] = HairSetNode::Hair(v0,0);
    }

    mesh->positions[0].resize(4*numHairs);
    for (size_t i=0; i<4*numHairs; i++) {
      const float x = cast_i2f(RandomSampler_getUInt(sampler));
      const float y = cast_i2f(RandomSampler_getUInt(sampler));
      const float z = cast_i2f(RandomSampler_getUInt(sampler));
      const float r = cast_i2f(RandomSampler_getUInt(sampler));
      mesh->positions[0][i] = Vec3fa(x,y,z,r);
    }

    if (mblur) 
    {
      mesh->positions[1].resize(4*numHairs);
      for (size_t i=0; i<4*numHairs; i++) {
        const float x = cast_i2f(RandomSampler_getUInt(sampler));
        const float y = cast_i2f(RandomSampler_getUInt(sampler));
        const float z = cast_i2f(RandomSampler_getUInt(sampler));
        const float r = cast_i2f(RandomSampler_getUInt(sampler));
        mesh->positions[1][i] = Vec3fa(x,y,z,r);
      }
    }

    return mesh.dynamicCast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::createGarbageSubdivMesh (int hash, size_t numFaces, bool mblur, Ref<MaterialNode> material)
  {
    RandomSampler sampler;
    RandomSampler_init(sampler,hash);
    Ref<SceneGraph::SubdivMeshNode> mesh = new SceneGraph::SubdivMeshNode(material,BBox1f(0,1),mblur?2:1);

    for (size_t i=0; i<numFaces; i++) 
    {
      const unsigned f = RandomSampler_getInt(sampler) % 20;
      mesh->verticesPerFace.push_back(f);
      for (size_t j=0; j<f; j++) 
      {
        mesh->position_indices.push_back((RandomSampler_getInt(sampler) % 32 == 0) ? RandomSampler_getUInt(sampler) : unsigned(mesh->numPositions()));

        const float x = cast_i2f(RandomSampler_getUInt(sampler));
        const float y = cast_i2f(RandomSampler_getUInt(sampler));
        const float z = cast_i2f(RandomSampler_getUInt(sampler));
        const float w = cast_i2f(RandomSampler_getUInt(sampler));
        mesh->positions[0].push_back(Vec3fa(x,y,z,w));

        if (mblur) 
        {
          const float x = cast_i2f(RandomSampler_getUInt(sampler));
          const float y = cast_i2f(RandomSampler_getUInt(sampler));
          const float z = cast_i2f(RandomSampler_getUInt(sampler));
          const float w = cast_i2f(RandomSampler_getUInt(sampler));
          mesh->positions[1].push_back(Vec3fa(x,y,z,w));
        }
      }
    }

    return mesh.dynamicCast<SceneGraph::Node>();
  }


  Ref<SceneGraph::Node> SceneGraph::createGarbagePointSet(int hash, size_t numPoints, bool mblur, Ref<MaterialNode> material)
  {
    RandomSampler sampler;
    RandomSampler_init(sampler,hash);

    Ref<SceneGraph::PointSetNode> mesh = new SceneGraph::PointSetNode(RTC_GEOMETRY_TYPE_SPHERE_POINT, material,BBox1f(0,1),mblur?2:1);

    for (size_t i = 0; i < numPoints; i++)
    {
      const float x = cast_i2f(RandomSampler_getUInt(sampler));
      const float y = cast_i2f(RandomSampler_getUInt(sampler));
      const float z = cast_i2f(RandomSampler_getUInt(sampler));
      const float r = cast_i2f(RandomSampler_getUInt(sampler));
      mesh->positions[0].push_back(Vec3fa(x, y, z, r));

      if (mblur)
      {
        const float x = cast_i2f(RandomSampler_getUInt(sampler));
        const float y = cast_i2f(RandomSampler_getUInt(sampler));
        const float z = cast_i2f(RandomSampler_getUInt(sampler));
        const float r = cast_i2f(RandomSampler_getUInt(sampler));
        mesh->positions[1].push_back(Vec3fa(x, y, z, r));
      }
    }

    return mesh.dynamicCast<SceneGraph::Node>();
  }
}
