// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "../common/tutorial/tutorial.h"
#include "../common/tutorial/benchmark_render.h"
#include "../common/tutorial/statistics.h"
#include <set>
#include "../../common/sys/mutex.h"
#include "../common/core/ray.h"
#include "../../kernels/geometry/triangle_triangle_intersector.h"
#include "clothModel.h"

namespace embree
{
  void updateScene ();
    
  // RTCDevice g_device = nullptr;
  RTCScene g_scene = nullptr;

  using SurfacePoint = std::pair<unsigned,unsigned>;
  using Collision = std::pair<SurfacePoint, SurfacePoint>;
  using Collisions = std::vector<Collision>;
  Collisions sim_collisions;
  // bool use_user_geometry = false;
  std::vector<std::unique_ptr<collide2::Mesh>> meshes;
  unsigned int clothID;
  bool benchmark = false;

  int numPhi = 45;
  int numTheta = 2*numPhi;

  size_t NX = 50; 
  size_t NZ = 50;

  SpinLock mutex;

  bool pause = false;

  bool intersect_triangle_triangle (unsigned geomID0, unsigned primID0, unsigned geomID1, unsigned primID1)
{
  //CSTAT(bvh_collide_prim_intersections1++);
  
  /* special culling for scene intersection with itself */
  if (geomID0 == geomID1 && primID0 == primID1) {
    return false;
  }
  //CSTAT(bvh_collide_prim_intersections2++);

  auto mesh0 = meshes[geomID0].get ();
  auto mesh1 = meshes[geomID1].get ();
  auto const & tri0 = (Triangle&) mesh0->tris_[primID0];
  auto const & tri1 = (Triangle&) mesh1->tris_[primID1];
  
  if (geomID0 == geomID1)
  {
    /* ignore intersection with topological neighbors */
    const vint4 t0(tri0.v0,tri0.v1,tri0.v2,tri0.v2);
    if (any(vint4(tri1.v0) == t0)) return false;
    if (any(vint4(tri1.v1) == t0)) return false;
    if (any(vint4(tri1.v2) == t0)) return false;
  }
  //CSTAT(bvh_collide_prim_intersections3++);
  
  const Vec3fa a0 = mesh0->x_[tri0.v0];
  const Vec3fa a1 = mesh0->x_[tri0.v1];
  const Vec3fa a2 = mesh0->x_[tri0.v2];
  const Vec3fa b0 = mesh1->x_[tri1.v0];
  const Vec3fa b1 = mesh1->x_[tri1.v1];
  const Vec3fa b2 = mesh1->x_[tri1.v2];
  
  return isa::TriangleTriangleIntersector::intersect_triangle_triangle(a0,a1,a2,b0,b1,b2);
}

void CollideFunc (void* userPtr, RTCCollision* collisions, unsigned int num_collisions)
{
    for (size_t i=0; i<num_collisions;)
    {
      bool intersect = intersect_triangle_triangle(collisions[i].geomID0,collisions[i].primID0,
                                                   collisions[i].geomID1,collisions[i].primID1);
      if (intersect) i++;
      else collisions[i] = collisions[--num_collisions];
    }
  
  if (num_collisions == 0) 
    return;

  Lock<SpinLock> lock(mutex);
  for (size_t i=0; i<num_collisions; i++)
  {
    const unsigned geomID0 = collisions[i].geomID0;
    const unsigned primID0 = collisions[i].primID0;
    const unsigned geomID1 = collisions[i].geomID1;
    const unsigned primID1 = collisions[i].primID1;

    static_cast<Collisions*>(userPtr)->push_back(std::make_pair(std::make_pair(geomID0,primID0),std::make_pair(geomID1,primID1)));
  }
}

void triangle_bounds_func(const struct RTCBoundsFunctionArguments* args)
{
  void* ptr  = args->geometryUserPtr;
  unsigned geomID = (unsigned) (size_t) ptr;
  auto const & mesh = *meshes[geomID];
  BBox3fa bounds = empty;
  bounds.extend(mesh.x_[mesh.tris_[args->primID].v0]);
  bounds.extend(mesh.x_[mesh.tris_[args->primID].v1]);
  bounds.extend(mesh.x_[mesh.tris_[args->primID].v2]);
  *(BBox3fa*) args->bounds_o = bounds;
}

void triangle_intersect_func(const RTCIntersectFunctionNArguments* args)
{
  void* ptr  = args->geometryUserPtr;
  ::Ray* ray = (::Ray*)args->rayhit;
  unsigned int primID = args->primID;
  unsigned geomID = (unsigned) (size_t) ptr;
  auto const & mesh = *meshes[geomID];
  
  auto & v0 = mesh.x_[mesh.tris_[args->primID].v0];
  auto & v1 = mesh.x_[mesh.tris_[args->primID].v1];
  auto & v2 = mesh.x_[mesh.tris_[args->primID].v2];
  auto e1 = v0-v1;
  auto e2 = v2-v0;
  auto Ng = cross(e1,e2);

  /* calculate denominator */
  auto O = Vec3fa(ray->org);
  auto D = Vec3fa(ray->dir);
  auto C = v0 - O;
  auto R = cross(D,C);
  float den = dot(Ng,D);
  float rcpDen = rcp(den);
      
  /* perform edge tests */
  float u = dot(R,e2)*rcpDen;
  float v = dot(R,e1)*rcpDen;
          
  /* perform backface culling */        
  bool valid = (den != 0.0f) & (u >= 0.0f) & (v >= 0.0f) & (u+v<=1.0f);
  if (likely(!valid)) return;
      
  /* perform depth test */
  float t = dot(Vec3fa(Ng),C)*rcpDen;
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
    Tutorial()
      : TutorialApplication("collide",FEATURE_RTCORE)
    {
      registerOption("benchmark-collision", [] (Ref<ParseStream> cin, const FileName& path) {
          benchmark = true;
        }, "--benchmark-collision: benchmarks collision detection");

      registerOption("complexity", [] (Ref<ParseStream> cin, const FileName& path) {
          int N = cin->getInt();
          numPhi = N;
          numTheta = 2*numPhi;
          NX = N;
          NZ = N;
        }, "--complexity: sets grid and sphere geometric complexity");
                       
      camera.from = Vec3fa(-2.5f,2.5f,-2.5f);
      camera.to   = Vec3fa(0.0f,0.0f,0.0f);
    }

#if defined(USE_GLFW)
    
    void keypressed(int key) override
    {
      if (key == 32  /* */) initializeClothPositions ((collide2::ClothModel &) (*meshes[clothID]));
      if (key == 80 /*p*/) { pause = !pause; }
      if (pause == true && key == 78 /*n*/) { updateScene (); }
      else TutorialApplication::keypressed(key);
    }
#endif
    
  };
}

int main(int argc, char** argv) {
  if (embree::TutorialBenchmark::benchmark(argc, argv)) {
    return embree::TutorialBenchmark(embree::renderBenchFunc<embree::Tutorial>).main(argc, argv, "collide");
  }
  return embree::Tutorial().main(argc,argv);
}
