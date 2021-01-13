// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/math/affinespace.h"
#include "../common/math/linearspace.h"
#include "../common/math/random_sampler.h"
#include "../common/tutorial/scene.h"
#include "scene.h"

/*
 * Our scene has multiple instantiated trees on the root level,
 * which in turn instantiate twigs.
 */

namespace embree {

/*
 * Combined quad/triangle meshes.
 */
struct Mesh
{
  const unsigned int  numVertices;
  const float*        vertices;
  const unsigned int  numTriangles;
  const unsigned int* triangleIndices;
  const unsigned int  numQuads;
  const unsigned int* quadIndices;
};

/*
 * Instances.
 */
struct Instances
{
  const unsigned int numInstances;

  // These are the column vectors of the 3x4 transformation
  // matrix from instance-local space to parent space.
  const float* localX;
  const float* localY;
  const float* localZ;
  const float* position;
};

/*
 * Create a triangle mesh geometry from the given mesh.
 */
RTCGeometry makeTriangleMesh(RTCDevice device, const Mesh& mesh)
{
  RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);

  const unsigned int timeStep = 0;

  rtcSetSharedGeometryBuffer(geom, 
                             RTC_BUFFER_TYPE_VERTEX, 
                             timeStep, 
                             RTC_FORMAT_FLOAT3, 
                             mesh.vertices, 
                             0, 
                             3*sizeof(float), 
                             mesh.numVertices);

  rtcSetSharedGeometryBuffer(geom, 
                             RTC_BUFFER_TYPE_INDEX, 
                             timeStep, 
                             RTC_FORMAT_UINT3, 
                             mesh.triangleIndices,
                             0, 
                             3*sizeof(unsigned int), 
                             mesh.numTriangles);

  rtcCommitGeometry(geom);
  return geom;
}

/*
 * Create a quad mesh geometry from the given mesh.
 */
RTCGeometry makeQuadMesh(RTCDevice device, const Mesh& mesh)
{
  RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_QUAD);

  const unsigned int timeStep = 0;

  rtcSetSharedGeometryBuffer(geom, 
                             RTC_BUFFER_TYPE_VERTEX, 
                             timeStep, 
                             RTC_FORMAT_FLOAT3, 
                             mesh.vertices, 
                             0, 
                             3*sizeof(float), 
                             mesh.numVertices);

  rtcSetSharedGeometryBuffer(geom, 
                             RTC_BUFFER_TYPE_INDEX, 
                             timeStep, 
                             RTC_FORMAT_UINT4, 
                             mesh.quadIndices,
                             0, 
                             4*sizeof(unsigned int), 
                             mesh.numQuads);

  rtcCommitGeometry(geom);
  return geom;
}

/*
 * Add a mesh to a scene.
 */
void addMesh(RTCDevice device, RTCScene scene, const Mesh& mesh)
{
  if (mesh.numTriangles > 0)
  {
    RTCGeometry geom = makeTriangleMesh(device, mesh);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
  }
  if (mesh.numQuads > 0)
  {
    RTCGeometry geom = makeQuadMesh(device, mesh);
    rtcAttachGeometry(scene, geom);
    rtcReleaseGeometry(geom);
  }
}

/*
 * Build a transformation matrix from individual vectors.
 */
__forceinline AffineSpace3fa buildMatrix(const float* localX,
                                         const float* localY,
                                         const float* localZ,
                                         const float* position)
{
  return AffineSpace3fa::translate(Vec3fa(position[0], position[1], position[2]))
       * AffineSpace3fa(LinearSpace3fa(Vec3fa(localX[0], localX[1], localX[2]),
                                       Vec3fa(localY[0], localY[1], localY[2]),
                                       Vec3fa(localZ[0], localZ[1], localZ[2])));
}

/*
 * Add instances to a scene.
 * This function also writes normal transforms to the given map.
 */
void addInstances(RTCDevice device,  
                  RTCScene scene,
                  RTCScene exemplar,
                  const Instances& instances,
                  LinearSpace3fa* normalTransforms)
{
  for (unsigned int i = 0; i < instances.numInstances; ++i)
  {
    const unsigned int idx = 3*i;
    auto M = buildMatrix(instances.localX + idx, 
                         instances.localY + idx,
                         instances.localZ + idx,
                         instances.position + idx);

    RTCGeometry geom = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_INSTANCE);
    rtcSetGeometryInstancedScene(geom, exemplar);
    rtcSetGeometryTimeStepCount(geom, 1);
    rtcSetGeometryTransform(geom,
                            0,
                            RTC_FORMAT_FLOAT4X4_COLUMN_MAJOR,
                            reinterpret_cast<float*>(&M));
    rtcCommitGeometry(geom);

    rtcAttachGeometryByID(scene, geom, i);
    rtcReleaseGeometry(geom);
    normalTransforms[i] = transposed(rcp(M.l));
  }
}

/*
 * Initialize the main scene. Note that we store complex geometry in
 * separate files purely for readability.
 */
#include "geometry/ground.cpp"
#include "geometry/twig.cpp"
#include "geometry/tree.cpp"
#include "geometry/trees.cpp"

static const unsigned int g_instancesOnLevel[] = { Trees::instances.numInstances, 
                                                   Twigs01::instances.numInstances };
static LinearSpace3fa** g_normalTransforms;

void cleanupScene()
{
  if ( g_normalTransforms )
  {
    alignedFree(g_normalTransforms[0]);
    alignedFree(g_normalTransforms[1]);
    alignedFree(g_normalTransforms);
    g_normalTransforms = nullptr;
  }
}

RTCScene initializeScene(RTCDevice device, InstanceLevels* levels)
{
  cleanupScene();

  g_normalTransforms = (LinearSpace3fa**)alignedMalloc(2*sizeof(LinearSpace3fa*), 16);
  g_normalTransforms[0] = (LinearSpace3fa*)alignedMalloc(g_instancesOnLevel[0]*sizeof(LinearSpace3fa), 16);
  g_normalTransforms[1] = (LinearSpace3fa*)alignedMalloc(g_instancesOnLevel[1]*sizeof(LinearSpace3fa), 16);
  levels->numLevels = 2;
  levels->numInstancesOnLevel = g_instancesOnLevel;
  levels->normalTransforms = g_normalTransforms;

  RandomSampler sampler;
  RandomSampler_init(sampler, 98248);

  RTCScene twig = rtcNewScene(device);
  rtcSetSceneFlags(twig, RTC_SCENE_FLAG_CONTEXT_FILTER_FUNCTION);
  addMesh(device, twig, Twig::mesh);
  rtcCommitScene(twig);

  RTCScene tree = rtcNewScene(device);
  rtcSetSceneFlags(tree, RTC_SCENE_FLAG_CONTEXT_FILTER_FUNCTION);
  addInstances(device, tree, twig, Twigs01::instances, g_normalTransforms[1]);

  addMesh(device, tree, Tree01::mesh);
  rtcCommitScene(tree);
  rtcReleaseScene(twig);

  RTCScene scene = rtcNewScene(device);
  rtcSetSceneFlags(scene, RTC_SCENE_FLAG_CONTEXT_FILTER_FUNCTION);
  addInstances(device, scene, tree, Trees::instances, g_normalTransforms[0]);
  addMesh(device, scene, Ground::mesh);

  rtcReleaseScene(tree);

  rtcCommitScene(scene);
  return scene;
}

// -----------------------------------------------------------------------------
} // namespace embree 
// -----------------------------------------------------------------------------

extern "C" RTCScene initializeScene(RTCDevice device,
                                    InstanceLevels* levels)
{
  return embree::initializeScene(device, levels);
}

extern "C" void cleanupScene()
{
  embree::cleanupScene();
}
