// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/statistics.h"
#include <set>
#include "../../common/sys/mutex.h"
#include "../common/core/ray.h"
#include "../../kernels/geometry/triangle_triangle_intersector.h"

namespace embree
{
  RTCDevice g_device = nullptr;
  RTCScene g_scene = nullptr;
  std::shared_ptr<TutorialScene> g_tutorial_scene = nullptr;
  size_t cur_time = 0;
  std::vector<std::shared_ptr<TutorialScene>> g_animation;
  std::set<std::pair<unsigned,unsigned>> collision_candidates;
  //RTCScene g_scene0 = nullptr;
  //RTCScene g_scene1 = nullptr;
  //TutorialScene g_tutorial_scene0;
  //TutorialScene g_tutorial_scene1;
  //std::set<std::pair<unsigned,unsigned>> set0;
  //std::set<std::pair<unsigned,unsigned>> set1;
  bool use_user_geometry = false;

  size_t skipBenchmarkRounds = 0;
  size_t numBenchmarkRounds = 0;
  std::atomic<size_t> numTotalCollisions(0);
  SpinLock mutex;

  bool intersect_triangle_triangle (TutorialScene* scene0, unsigned geomID0, unsigned primID0, TutorialScene* scene1, unsigned geomID1, unsigned primID1)
  {
    //CSTAT(bvh_collide_prim_intersections1++);
    const SceneGraph::TriangleMeshNode* mesh0 = (SceneGraph::TriangleMeshNode*) scene0->geometries[geomID0].ptr;
    const SceneGraph::TriangleMeshNode* mesh1 = (SceneGraph::TriangleMeshNode*) scene1->geometries[geomID1].ptr;
    const SceneGraph::TriangleMeshNode::Triangle& tri0 = mesh0->triangles[primID0];
    const SceneGraph::TriangleMeshNode::Triangle& tri1 = mesh1->triangles[primID1];
    
    /* special culling for scene intersection with itself */
    if (scene0 == scene1 && geomID0 == geomID1)
    {
      /* ignore self intersections */
      if (primID0 == primID1)
        return false;
    }
    //CSTAT(bvh_collide_prim_intersections2++);
    
    if (scene0 == scene1 && geomID0 == geomID1)
    {
      /* ignore intersection with topological neighbors */
      const vint4 t0(tri0.v0,tri0.v1,tri0.v2,tri0.v2);
      if (any(vint4(tri1.v0) == t0)) return false;
      if (any(vint4(tri1.v1) == t0)) return false;
      if (any(vint4(tri1.v2) == t0)) return false;
    }
    //CSTAT(bvh_collide_prim_intersections3++);
    
    const Vec3fa a0 = mesh0->positions[0][tri0.v0];
    const Vec3fa a1 = mesh0->positions[0][tri0.v1];
    const Vec3fa a2 = mesh0->positions[0][tri0.v2];
    const Vec3fa b0 = mesh1->positions[0][tri1.v0];
    const Vec3fa b1 = mesh1->positions[0][tri1.v1];
    const Vec3fa b2 = mesh1->positions[0][tri1.v2];
    
    return isa::TriangleTriangleIntersector::intersect_triangle_triangle(a0,a1,a2,b0,b1,b2);
  }
  
  void CollideFunc (void* userPtr, RTCCollision* collisions, size_t num_collisions)
  {
    if (use_user_geometry)
    {
      for (size_t i=0; i<num_collisions;)
      {
        bool intersect = intersect_triangle_triangle(g_tutorial_scene.get(),collisions[i].geomID0,collisions[i].primID0,
                                                     g_tutorial_scene.get(),collisions[i].geomID1,collisions[i].primID1);
        if (intersect) i++;
        else collisions[i] = collisions[--num_collisions];
      }
    }
    
    if (num_collisions == 0) 
      return;

    if (numBenchmarkRounds) 
      return;
  
    //numTotalCollisions+=num_collisions;

    Lock<SpinLock> lock(mutex);
    for (size_t i=0; i<num_collisions; i++)
    {
      const unsigned geomID0 = collisions[i].geomID0;
      const unsigned primID0 = collisions[i].primID0;
      const unsigned geomID1 = collisions[i].geomID1;
      const unsigned primID1 = collisions[i].primID1;
      //PRINT4(geomID0,primID0,geomID1,primID1);
      collision_candidates.insert(std::make_pair(geomID0,primID0));
      collision_candidates.insert(std::make_pair(geomID1,primID1));
      //set0.insert(std::make_pair(geomID0,primID0));
      //set1.insert(std::make_pair(geomID1,primID1));

#if 0
      /* verify result */
      Ref<TutorialScene::TriangleMesh> mesh0 = g_tutorial_scene->geometries[geomID0].dynamicCast<TutorialScene::TriangleMesh>();
      TutorialScene::Triangle tri0 = mesh0->triangles[primID0];
      BBox3fa bounds0 = empty;
      bounds0.extend(mesh0->positions[0][tri0.v0]);
      bounds0.extend(mesh0->positions[0][tri0.v1]);
      bounds0.extend(mesh0->positions[0][tri0.v2]);

      Ref<TutorialScene::TriangleMesh> mesh1 = g_tutorial_scene->geometries[geomID1].dynamicCast<TutorialScene::TriangleMesh>();
      TutorialScene::Triangle tri1 = mesh1->triangles[primID1];
      BBox3fa bounds1 = empty;
      bounds1.extend(mesh1->positions[0][tri1.v0]);
      bounds1.extend(mesh1->positions[0][tri1.v1]);
      bounds1.extend(mesh1->positions[0][tri1.v2]);

      if (disjoint(bounds0,bounds1)) 
        std::cout << "WARNING: bounds do not overlap!" << std::endl;
#endif
    }
  }

  void triangle_bounds_func(const struct RTCBoundsFunctionArguments* args)
  {
    const unsigned geomID = (unsigned) (size_t) args->geometryUserPtr;
    const SceneGraph::TriangleMeshNode* mesh = (SceneGraph::TriangleMeshNode*) g_tutorial_scene->geometries[geomID].ptr;
    const SceneGraph::TriangleMeshNode::Triangle& tri = mesh->triangles[args->primID];
    BBox3fa bounds = empty;
    bounds.extend(mesh->positions[0][tri.v0]);
    bounds.extend(mesh->positions[0][tri.v1]);
    bounds.extend(mesh->positions[0][tri.v2]);
    *(BBox3fa*) args->bounds_o = bounds;
  }
  
  void triangle_intersect_func(const RTCIntersectFunctionNArguments* args)
  {
    void* ptr  = args->geometryUserPtr;
    ::Ray* ray = (::Ray*)args->rayhit;
    unsigned int primID = args->primID;

    const unsigned geomID = (unsigned) (size_t) ptr;
    const SceneGraph::TriangleMeshNode* mesh = (SceneGraph::TriangleMeshNode*) g_tutorial_scene->geometries[geomID].ptr;
    const SceneGraph::TriangleMeshNode::Triangle& tri = mesh->triangles[primID];
    const Vec3fa v0 = mesh->positions[0][tri.v0];
    const Vec3fa v1 = mesh->positions[0][tri.v1];
    const Vec3fa v2 = mesh->positions[0][tri.v2];
    const Vec3fa e1 = v0-v1;
    const Vec3fa e2 = v2-v0;
    const Vec3fa Ng = cross(e1,e2);

    /* calculate denominator */
    const Vec3fa O = Vec3fa(ray->org);
    const Vec3fa D = Vec3fa(ray->dir);
    const Vec3fa C = v0 - O;
    const Vec3fa R = cross(D,C);
    const float den = dot(Ng,D);
    const float rcpDen = rcp(den);
        
    /* perform edge tests */
    const float u = dot(R,e2)*rcpDen;
    const float v = dot(R,e1)*rcpDen;
            
    /* perform backface culling */        
    bool valid = (den != 0.0f) & (u >= 0.0f) & (v >= 0.0f) & (u+v<=1.0f);
    if (likely(!valid)) return;
        
    /* perform depth test */
    const float t = dot(Vec3fa(Ng),C)*rcpDen;
    valid &= (t > ray->tnear()) & (t < ray->tfar);
    if (likely(!valid)) return;
    
    /* update hit */
    ray->tfar = t;
    ray->u = u;
    ray->v = v;
    ray->geomID = geomID;
    ray->primID = primID;
    ray->Ng = Ng;
  }

  struct Tutorial : public TutorialApplication
  {
    bool pause;
  
    Tutorial()
      : TutorialApplication("collide",FEATURE_RTCORE), pause(false)//, use_user_geometry(false)
    {

    }
    
  };

}

int main(int argc, char** argv) {
  return embree::Tutorial().main(argc,argv);
}
