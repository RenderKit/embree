// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "twolevel_accel.h"
#include "virtual_accel.h"
#include "rtcore/scene.h"

#include "bvh4/bvh4.h"
#include "bvh4i/bvh4i.h"

namespace embree
{
  TwoLevelAccel::TwoLevelAccel (const std::string topAccel, Scene* scene, createTriangleMeshAccelTy createTriangleMeshAccel, bool buildUserGeometryAccel)
    : scene(scene), createTriangleMeshAccel(createTriangleMeshAccel), buildUserGeometryAccel(buildUserGeometryAccel),
      accel(new VirtualAccel(topAccel,enabled_accels)) {}

  TwoLevelAccel::~TwoLevelAccel () 
  {
    delete accel;
    for (size_t i=0; i<triangle_accels.size(); i++)
      delete triangle_accels[i];
  }

  void TwoLevelAccel::buildTriangleAccels(size_t threadIndex, size_t threadCount) // FIXME: no longer used
  {
    size_t N = scene->size();
    
    for (size_t i=N; i<triangle_accels.size(); i++)
      delete triangle_accels[i];

    if (triangle_accels.size() < N) 
      triangle_accels.resize(N);
      
    /* build bottom level accel */
    for (size_t i=0; i<N; i++) 
    {
      TriangleMeshScene::TriangleMesh* mesh = scene->getTriangleMeshSafe(i);
      if (mesh == NULL) {
        if (triangle_accels[i]) {
          delete triangle_accels[i];
          triangle_accels[i] = NULL;
        }
        continue;
      }
      
      if (triangle_accels[i] == NULL) {
        triangle_accels[i] = createTriangleMeshAccel(mesh);
      }

      if (mesh->isEnabled()) {
        enabled_accels.push_back(triangle_accels[i]);
      }

      if (mesh->isModified()) {
        triangle_accels[i]->build(threadIndex,threadCount);
        mesh->state = Geometry::ENABLED;
      }
    }
  }

  void TwoLevelAccel::buildUserGeometryAccels(size_t threadIndex, size_t threadCount)
  {
    if (scene->numUserGeometries == 0) 
      return;

    size_t N = scene->size();
    for (size_t i=0; i<N; i++) 
    {
      Geometry* geom = scene->getUserGeometrySafe(i);
      if (geom == NULL) continue;
      Accel* accel = scene->getUserGeometrySafe(i);

      if (geom->isEnabled()) {
        enabled_accels.push_back(accel);
      }

      if (geom->isModified()) {
        accel->build(threadIndex,threadCount);
        geom->state = Geometry::ENABLED;
      }
    }
  }

  void TwoLevelAccel::build(size_t threadIndex, size_t threadCount)
  {
    /* build all object accels */
    enabled_accels.clear();
    if (createTriangleMeshAccel) buildTriangleAccels(threadIndex,threadCount);
    if (buildUserGeometryAccel) buildUserGeometryAccels(threadIndex,threadCount);

    /* no top-level accel required for single geometry */
    if (enabled_accels.size() == 1) {
      bounds = enabled_accels[0]->bounds;
      intersectors = enabled_accels[0]->intersectors;
    }

    /* build toplevel accel */
    else {
      accel->build(threadIndex,threadCount);
      bounds = accel->bounds;
      intersectors = accel->intersectors;
    }
  }
}
