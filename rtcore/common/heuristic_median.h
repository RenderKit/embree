// ======================================================================== //
// Copyright 2009-2012 Intel Corporation                                    //
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

#ifndef __EMBREE_HEURISTIC_MEDIAN_H__
#define __EMBREE_HEURISTIC_MEDIAN_H__

#include "accel.h"
#include "primref.h"

namespace embree
{
  /* Median cut heuristic. Finds the split with the best SAH
   * heuristic at the middle of the interval range by testing 
   * for all 3 dimensions. This is nothing more than an object 
   * binning heuristic with only 2 bins. */
  
  template<int logBlockSize>
    class HeuristicMedian
  {
    /*! Maximal number of bins. */
    static const size_t maxBins = 2; 

  public:

    /*! We build the tree depth first */
    static const bool depthFirst = true;
    
    /*! stores bounding information for a set of primitives */
    class PrimInfo
    {
    public:
      __forceinline PrimInfo () 
        : num(0), geomBounds(empty), centBounds(empty) {}

      __forceinline PrimInfo (size_t num, const BBox3f& geomBounds) 
        : num(num), geomBounds(geomBounds), centBounds(geomBounds) {}
      
      __forceinline PrimInfo (size_t num, const BBox3f& geomBounds, const BBox3f& centBounds) 
        : num(num), geomBounds(geomBounds), centBounds(centBounds) {}
      
      /*! returns the number of primitives */
      __forceinline size_t size() const { 
        return num; 
      }

      /*! return the surface area heuristic when creating a leaf */
      __forceinline float sah () const { 
        return halfArea(geomBounds)*blocks(num); 
      }

      /*! clears global data */
      __forceinline void clear () { 
      }
      
      /*! stream output */
      friend std::ostream& operator<<(std::ostream& cout, const PrimInfo& pinfo) {
        return cout << "PrimInfo { num = " << pinfo.num << ", geomBounds = " << pinfo.geomBounds << ", centBounds = " << pinfo.centBounds << "}";
      }
      
    public:
      size_t num;          //!< number of primitives
      BBox3f geomBounds;   //!< geometry bounds of primitives
      BBox3f centBounds;   //!< centroid bounds of primitives
    };
        
    /*! mapping from bounding boxes to bins */
    class Mapping
    {
    public:
      
      /*! default constructor */
      __forceinline Mapping () {}
      
      /*! construct from primitive info */
      __forceinline Mapping (const PrimInfo& pinfo)
      {
        ofs   = pinfo.centBounds.lower;
        scale = rcp(pinfo.centBounds.upper - pinfo.centBounds.lower) * Vec3f(float(maxBins));
      }
      
      /*! returns number of bins */
      __forceinline size_t size() const { return maxBins; }
      
      /*! Computes the bin numbers for each dimension for a box. */
      __forceinline Vec3i bin_unsafe(const BBox3f& box) const {
        return Vec3i((center2(box) - ofs)*scale-Vec3f(0.5f));
      }
      
      /*! Computes the bin numbers for each dimension for a box. */
      __forceinline Vec3i bin(const BBox3f& box) const {
        return clamp(bin_unsafe(box),Vec3i(0),Vec3i(int(maxBins-1)));
      }
      
    private:
      Vec3f ofs;        //!< offset to compute bin
      Vec3f scale;      //!< scaling factor to compute bin
    };
    
    /*! Stores information about an object split. */
    class Split
    {
    public:
      
      /*! create an invalid split by default */
      __forceinline Split () : dim(0), pos(0), cost(inf) {}
      
      /*! return SAH cost of performing the split */
      __forceinline float sah() const { return cost; } 
      
      /*! return true if this is a spatial split */
      __forceinline bool spatial() const { return false; }

      /*! tests if a primitive belongs to the left side of the split */
      __forceinline int left(const PrimRef& prim) const {
        return mapping.bin_unsafe(prim.bounds())[dim] < pos;
      }

      /*! splitting of the primitive if required */
      __forceinline void split(const PrimRef& prim, PrimRef& lprim_o, PrimRef& rprim_o) const {
        new (&lprim_o) PrimRef(empty,size_t(-1));
        new (&rprim_o) PrimRef(empty,size_t(-1));
        assert(false);
      }
      
      /*! stream output */
      friend std::ostream& operator<<(std::ostream& cout, const Split& split) {
        return cout << "Split { dim = " << split.dim << 
          ", pos = " << split.pos << 
          ", sah = " << split.cost << 
          ", linfo = " << split.linfo <<
          ", rinfo = " << split.rinfo << "}";
      }
      
    public:
      Mapping mapping;    //!< Mapping to bins
      int dim;            //!< Best object split dimension
      int pos;            //!< Best object split position
      float cost;         //!< SAH cost of performing best object split
      
    public:
      PrimInfo linfo;     //!< Left geometry information
      PrimInfo rinfo;     //!< Right geometry information
    };
    
    /*! default constructor */
    __forceinline HeuristicMedian () {}
    
    /*! construction from geometry info */
    __forceinline HeuristicMedian (const PrimInfo& pinfo, const BuildTriangle* triangles, const Vec3fa* vertices)
      : pinfo(pinfo), mapping(pinfo) { clear(); }
    
    /*! clear the binner */
    __forceinline void clear() 
    {
      for (size_t i=0; i<mapping.size(); i++) {
        counts[i] = 0;
        geomBounds[i][0] = geomBounds[i][1] = geomBounds[i][2] = empty;
        centBounds[i][0] = centBounds[i][1] = centBounds[i][2] = empty;
      }
    }
    
    /*! bin an array of primitives */
    void bin(const PrimRef* prim, size_t num);
    
    /*! merge multiple binning infos into one */
    static void reduce(const HeuristicMedian binners[], size_t num, HeuristicMedian& binner_o);
    
    /*! calculate the best possible split */
    void best(Split& split_o);
  
  private:
    
    /*! Compute the number of blocks occupied for each dimension. */
    __forceinline static ssei blocks(const ssei& a) { return (a+ssei((1 << logBlockSize)-1)) >> logBlockSize; }
    
    /*! Compute the number of blocks occupied in one dimension. */
    __forceinline static int  blocks(size_t a) { return (int)((a+((1LL << logBlockSize)-1)) >> logBlockSize); }
  
  public:
    PrimInfo pinfo;                //!< bounding information of geometry
    Mapping mapping;               //!< mapping from geometry to the bins
    
    /* initialize binning counter and bounds */
    ssei   counts    [maxBins];    //< number of primitives mapped to bin
    BBox3f geomBounds[maxBins][4]; //< bounds for every bin in every dimension
    BBox3f centBounds[maxBins][4]; //< centroid bounds for every bin in every dimension
  };
}

#endif
