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

#pragma once

#include "geometry/primitive.h"

namespace embree
{
  class BVH2HairBuilder : public Builder
  {
    ALIGNED_CLASS;
  public:

    /*! Type shortcuts */
    typedef BVH2Hair::Node    Node;
    typedef BVH2Hair::NodeRef NodeRef;
    typedef BVH2Hair::Bezier1 Bezier1;
    typedef BVH2Hair::NAABBox3fa NAABBox3fa;
    
  public:

    /*! builder entry point */
    void build(size_t threadIndex, size_t threadCount);

    /*! Constructor. */
    BVH2HairBuilder (BVH2Hair* bvh, Scene* scene);

    /*! Destructor. */
    ~BVH2HairBuilder ();

    /*! creates a leaf node */
    NodeRef leaf(size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds);

    /*! recursive build function */
    NodeRef recurse   (size_t threadIndex, size_t depth, size_t begin, size_t end, const NAABBox3fa& bounds);

  private:

    /*! Tries to split hair into two differently aligned hair strands */
    struct StrandSplit
    {
    public:
      StrandSplit (const NAABBox3fa& bounds0, const Vec3fa& axis0, const size_t num0,
                   const NAABBox3fa& bounds1, const Vec3fa& axis1, const size_t num1);

      /*! calculates surface area for the split */
      __forceinline float sah() const {
        return float(num0)*halfArea(bounds0.bounds) + float(num1)*halfArea(bounds1.bounds);
      }

      /*! finds the two hair strands */
      static const StrandSplit find(Bezier1* curves, size_t begin, size_t end);
      
      /*! splits hair list into the two strands */
      size_t split(Bezier1* curves, size_t begin, size_t end);

    public:
      NAABBox3fa bounds0, bounds1;  //!< bounds of the strands
      Vec3fa axis0, axis1;          //!< axis the strands are aligned into
      size_t num0, num1;            //!< number of hairs in the strands
    };

    /*! Performs standard object binning */
    struct ObjectSplit
    {
      /*! number of bins */
      static const size_t BINS = 16;

    public:

      __forceinline ObjectSplit ()
      : dim(0), pos(0), cost(inf), num0(0), num1(0) {}
      
      /*! calculates surface area for the split */
      __forceinline float sah() const {
        return float(num0)*halfArea(bounds0.bounds) + float(num1)*halfArea(bounds1.bounds);
      }

      /*! performs object binning to the the best partitioning */
      static const ObjectSplit find(Bezier1* curves, size_t begin, size_t end, const NAABBox3fa& pbounds);

      /*! splits hairs into two sets */
      size_t split(Bezier1* curves, size_t begin, size_t end);

  public:
    NAABBox3fa bounds;
    NAABBox3fa bounds0, bounds1;
    size_t dim;
    size_t pos;
    float cost;
    size_t num0,num1;
    ssef ofs,scale;
  };
    
    /*! try to find best non-axis aligned space, where the sum of all bounding areas is minimal */
    static const NAABBox3fa bestSpace(Bezier1* curves, size_t begin, size_t end);

  public:
    Scene* scene;          //!< source
    size_t minLeafSize;    //!< minimal size of a leaf
    size_t maxLeafSize;    //!< maximal size of a leaf
    BVH2Hair* bvh;         //!< output
    Bezier1* curves;       //!< array with all curves
  };
}
