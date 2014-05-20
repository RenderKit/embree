#include "bvh4hair.h"
#include "bvh4hair_builder.h"
#include "common/accelinstance.h"
#include "geometry/triangle1.h"

namespace embree
{
#define DBG(x)

  DECLARE_SYMBOL(Accel::Intersector1 ,BVH4HairIntersector1Bezier1i);
  DECLARE_SYMBOL(Accel::Intersector16,BVH4HairIntersector16Bezier1i);

  void BVH4HairRegister () 
  {
    int features = getCPUFeatures();

    SELECT_SYMBOL_KNC(features,BVH4HairIntersector1Bezier1i);
    SELECT_SYMBOL_KNC(features,BVH4HairIntersector16Bezier1i);

  }

  Accel::Intersectors BVH4HairIntersectors(BVH4i* bvh)
  {
    Accel::Intersectors intersectors;
    intersectors.ptr = bvh;
    intersectors.intersector1  = BVH4HairIntersector1Bezier1i;
    intersectors.intersector16 = BVH4HairIntersector16Bezier1i;
    return intersectors;
  }

  Accel* BVH4Hair::BVH4HairBinnedSAH(Scene* scene)
  {
    BVH4Hair* accel = new BVH4Hair(SceneTriangle1::type,scene);    
    Builder* builder = new BVH4HairBuilder(accel,NULL,scene);   
    Accel::Intersectors intersectors = BVH4HairIntersectors(accel);
    return new AccelInstance(accel,builder,intersectors);    
  }

  // =================================================================================
  // =================================================================================
  // =================================================================================


  __aligned(64) float BVH4Hair::UnalignedNode::identityMatrix[16] = {
    1,0,0,0,
    0,1,0,0,
    0,0,1,0,
    0,0,0,1
  };

  void BVH4Hair::UnalignedNode::convertFromBVH4iNode(const BVH4i::Node &node)
  {
    setIdentityMatrix();
    
  }




};
