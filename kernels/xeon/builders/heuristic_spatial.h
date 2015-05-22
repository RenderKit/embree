// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "../../common/scene.h"
#include "priminfo.h"

namespace embree
{
  namespace isa
  {
    /*! mapping into bins */
    template<size_t BINS>
      struct SpatialBinMapping
      {
      public:
        __forceinline SpatialBinMapping() {}
        
        /*! calculates the mapping */
        __forceinline SpatialBinMapping(const PrimInfo& pinfo)
        {
          const float4 lower = (float4) pinfo.geomBounds.lower;
          const float4 upper = (float4) pinfo.geomBounds.upper;
          const bool4 ulpsized = upper - lower <= max(float4(1E-19f),128.0f*float4(ulp)*max(abs(lower),abs(upper)));
          const float4 diag = (float4) pinfo.geomBounds.size();
          //scale = select(ulpsized,float4(0.0f),rcp(diag) * float4(BINS * 0.99f));
          scale = select(ulpsized,float4(0.0f),float4(BINS * 0.99f)/diag);
          ofs  = (float4) pinfo.geomBounds.lower;
        }
        
        /*! slower but safe binning */
        __forceinline int4 bin(const Vec3fa& p) const
        {
          const int4 i = floori((float4(p)-ofs)*scale);
          return clamp(i,int4(0),int4(BINS-1));
        }
        
        /*! calculates left spatial position of bin */
        __forceinline float pos(const int bin, const int dim) const {
          return float(bin)/scale[dim]+ofs[dim];
        }
        
        /*! returns true if the mapping is invalid in some dimension */
        __forceinline bool invalid(const int dim) const {
          return scale[dim] == 0.0f;
        }
        
      private:
        float4 ofs,scale;  //!< linear function that maps to bin ID
      };

    /*! stores all information required to perform some split */
    template<size_t BINS>
      struct SpatialBinSplit
      {
        /*! construct an invalid split by default */
        __forceinline SpatialBinSplit() 
          : sah(inf), dim(-1), pos(0) {}
        
        /*! constructs specified split */
        __forceinline SpatialBinSplit(float sah, int dim, int pos, const SpatialBinMapping<BINS>& mapping)
          : sah(sah), dim(dim), pos(pos), mapping(mapping) {}
        
        /*! tests if this split is valid */
        __forceinline bool valid() const { return dim != -1; }
        
        /*! calculates surface area heuristic for performing the split */
        __forceinline float splitSAH() const { return sah; }
        
        /*! stream output */
        friend std::ostream& operator<<(std::ostream& cout, const SpatialBinSplit& split) {
          return cout << "SpatialBinSplit { sah = " << split.sah << ", dim = " << split.dim << ", pos = " << split.pos << "}";
        }
        
      public:
        float sah;                 //!< SAH cost of the split
        int   dim;                 //!< split dimension
        int   pos;                 //!< split position
        SpatialBinMapping<BINS> mapping; //!< mapping into bins
      };    
    
    /*! stores all binning information */
    template<size_t BINS, typename PrimRef>
      struct __aligned(64) SpatialBinInfo
    {
      SpatialBinInfo() {
        clear(); // FIXME: dont initialize in default constructor, performance issue !!!!
      }

      __forceinline SpatialBinInfo(EmptyTy) {
	clear();
      }

      /*! clears the bin info */
      __forceinline void clear() 
      {
        for (size_t i=0; i<BINS; i++) { 
          bounds[i][0] = bounds[i][1] = bounds[i][2] = bounds[i][3] = empty;
          numBegin[i] = numEnd[i] = 0;
        }
      }
      
      /*! bins an array of triangles */
      template<typename SplitPrimitive>
        __forceinline void bin(const SplitPrimitive& splitPrimitive, const PrimRef* prims, size_t N, const PrimInfo& pinfo, const SpatialBinMapping<BINS>& mapping)
      {
        for (size_t i=0; i<N; i++)
        {
          const PrimRef prim = prims[i];
          //TriangleMesh* mesh = (TriangleMesh*) scene->get(prim.geomID());
          unsigned geomID = prim.geomID() & 0x00FFFFFF; // FIXME: hack !!
          unsigned splits = prim.geomID() >> 24;

          if (splits == 1)
          {
            const int4 bin = mapping.bin(center(prim.bounds()));
            for (size_t dim=0; dim<3; dim++) 
            {
              assert(bin[dim] >= 0 && bin[dim] < BINS);
              numBegin[bin[dim]][dim]++;
              numEnd  [bin[dim]][dim]++;
              bounds  [bin[dim]][dim].extend(prim.bounds());
            }
          } 
          else
          {
            //TriangleMesh* mesh = (TriangleMesh*) scene->get(geomID); 
            //TriangleMesh::Triangle tri = mesh->triangle(prim.primID());
            //const Vec3fa v0 = mesh->vertex(tri.v[0]);
            //const Vec3fa v1 = mesh->vertex(tri.v[1]);
            //const Vec3fa v2 = mesh->vertex(tri.v[2]);
            const int4 bin0 = mapping.bin(prim.bounds().lower);
            const int4 bin1 = mapping.bin(prim.bounds().upper);
            
            for (size_t dim=0; dim<3; dim++) 
            {
              size_t bin;
              PrimRef rest = prim;
              size_t l = bin0[dim];
              size_t r = bin1[dim];
              for (bin=bin0[dim]; bin<bin1[dim]; bin++) 
              {
                const float pos = mapping.pos(bin+1,dim);
                
                PrimRef left,right;
                //splitTriangle(rest,dim,pos,v0,v1,v2,left,right);
                splitPrimitive(rest,dim,pos,left,right);
                if (left.bounds().empty()) l++;
                
                bounds[bin][dim].extend(left.bounds());
                rest = right;
              }
              if (rest.bounds().empty()) r--;
              //numBegin[bin0[dim]][dim]++;
              //numEnd  [bin1[dim]][dim]++;
              numBegin[l][dim]++;
              numEnd  [r][dim]++;
              bounds  [bin][dim].extend(rest.bounds());
            }
          }
        }
      }
      
      /*! bins a range of primitives inside an array */
      template<typename SplitPrimitive>
        void bin(const SplitPrimitive& splitPrimitive, const PrimRef* prims, size_t begin, size_t end, const SpatialBinMapping<BINS>& mapping) {
	bin(splitPrimitive,prims+begin,end-begin,mapping);
      }
      
      /*! merges in other binning information */
      void merge (const SpatialBinInfo& other) // FIXME: dont iterate over all bin
      {
        for (size_t i=0; i<BINS; i++) 
        {
          numBegin[i] += other.numBegin[i];
          numEnd  [i] += other.numEnd  [i];
          bounds[i][0].extend(other.bounds[i][0]);
          bounds[i][1].extend(other.bounds[i][1]);
          bounds[i][2].extend(other.bounds[i][2]);
        }
      }

      /*! merges in other binning information */
      static __forceinline const SpatialBinInfo reduce (const SpatialBinInfo& a, const SpatialBinInfo& b) // FIXME: dont iterate over all bin
      {
        SpatialBinInfo c;
        for (size_t i=0; i<BINS; i++) 
        {
          c.numBegin[i] += a.numBegin[i]+b.numBegin[i];
          c.numEnd  [i] += a.numEnd  [i]+b.numEnd  [i];
          c.bounds[i][0] = embree::merge(a.bounds[i][0],b.bounds[i][0]);
          c.bounds[i][1] = embree::merge(a.bounds[i][1],b.bounds[i][1]);
          c.bounds[i][2] = embree::merge(a.bounds[i][2],b.bounds[i][2]);
        }
        return c;
      }
      
      /*! finds the best split by scanning binning information */
      SpatialBinSplit<BINS> best(const PrimInfo& pinfo, const SpatialBinMapping<BINS>& mapping, const size_t blocks_shift) const
      {
        /* sweep from right to left and compute parallel prefix of merged bounds */
        float4 rAreas[BINS];
        int4 rCounts[BINS];
        int4 count = 0; BBox3fa bx = empty; BBox3fa by = empty; BBox3fa bz = empty;
        for (size_t i=BINS-1; i>0; i--)
        {
          count += numEnd[i];
          rCounts[i] = count;
          bx.extend(bounds[i][0]); rAreas[i][0] = halfArea(bx);
          by.extend(bounds[i][1]); rAreas[i][1] = halfArea(by);
          bz.extend(bounds[i][2]); rAreas[i][2] = halfArea(bz);
          rAreas[3] = 0.0f;
        }
        
        /* sweep from left to right and compute SAH */
        int4 blocks_add = (1 << blocks_shift)-1;
        int4 ii = 1; float4 vbestSAH = pos_inf; int4 vbestPos = 0;
        count = 0; bx = empty; by = empty; bz = empty;
        for (size_t i=1; i<BINS; i++, ii+=1)
        {
          count += numBegin[i-1];
          bx.extend(bounds[i-1][0]); float Ax = halfArea(bx);
          by.extend(bounds[i-1][1]); float Ay = halfArea(by);
          bz.extend(bounds[i-1][2]); float Az = halfArea(bz);
          const float4 lArea = float4(Ax,Ay,Az,Az);
          const float4 rArea = rAreas[i];
          const int4 lCount = (count     +blocks_add) >> blocks_shift;
          const int4 rCount = (rCounts[i]+blocks_add) >> blocks_shift;
          const float4 sah = lArea*float4(lCount) + rArea*float4(rCount);
          vbestPos  = select(sah < vbestSAH,ii ,vbestPos);
          vbestSAH  = select(sah < vbestSAH,sah,vbestSAH);
        }
        
        /* find best dimension */
        float bestSAH = inf;
        int   bestDim = -1;
        int   bestPos = 0;
        for (size_t dim=0; dim<3; dim++) 
        {
          /* ignore zero sized dimensions */
          if (unlikely(mapping.invalid(dim)))
            continue;
          
          /* test if this is a better dimension */
          if (vbestSAH[dim] < bestSAH && vbestPos[dim] != 0) {
            bestDim = dim;
            bestPos = vbestPos[dim];
            bestSAH = vbestSAH[dim];
          }
        }
        
        /* return invalid split if no split found */
        if (bestDim == -1) 
          return SpatialBinSplit<BINS>(inf,-1,0,mapping);
        
        /* return best found split */
        return SpatialBinSplit<BINS>(bestSAH,bestDim,bestPos,mapping);
      }
      
    private:
      BBox3fa bounds[BINS][4];  //!< geometry bounds for each bin in each dimension // FIXME: 4 -> 3
      int4    numBegin[BINS];   //!< number of primitives starting in bin
      int4    numEnd[BINS];     //!< number of primitives ending in bin
    };
  }
}

