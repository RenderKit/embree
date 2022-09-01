// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "qbvh6.h"
#include "statistics.h"
#include "quadifier.h"

#include "../../common/alloc.h"
#include "../../builders/priminfo.h"
#include "../../builders/primrefgen_presplit.h"
#include "../../builders/heuristic_binning_array_aligned.h"
#include "../../../common/algorithms/parallel_for_for_prefix_sum.h"

namespace embree
{
  namespace isa
  {
    struct QBVH6BuilderSAH
    {
      static const size_t BVH_WIDTH = QBVH6::InternalNode6::NUM_CHILDREN;
      static const size_t MIN_LARGE_LEAF_LEVELS = 8; //!< create balanced tree of we are that many levels before the maximum tree depth
      
      /* the type of primitive that is referenced */
      enum Type { TRIANGLE=0, QUAD=1, PROCEDURAL=2, INSTANCE=3, UNKNOWN=4, NUM_TYPES=5 };
      
      /* triangle data for leaf creation */
      struct Triangle
      {
        Triangle ()
          : gmask(0) {}
        
        Triangle (uint32_t i0, uint32_t i1, uint32_t i2,
                  Vec3f p0, Vec3f p1, Vec3f p2, 
                  GeometryFlags gflags,
                  uint8_t gmask)
          : i0(i0), i1(i1), i2(i2), p0(p0), p1(p1), p2(p2), gflags(gflags), gmask(gmask) {}

        __forceinline bool valid() const {
          return gmask != 0;
        }
        
        uint32_t i0,i1,i2;
        Vec3f p0,p1,p2;
        GeometryFlags gflags;
        uint8_t gmask;
      };
      
      /* quad data for leaf creation */
      struct Quad
      {
        Quad (Vec3f p0, Vec3f p1, Vec3f p2, Vec3f p3, GeometryFlags gflags, uint8_t gmask)
          : p0(p0), p1(p1), p2(p2), p3(p3), gflags(gflags), gmask(gmask) {}
        
        Vec3f p0,p1,p2,p3;
        GeometryFlags gflags;
        uint8_t gmask;
      };
      
      /* procedural data for leaf creation */
      struct Procedural
      {
        Procedural (uint8_t gmask)
          : gmask(gmask) {}
        
        PrimLeafDesc desc(uint32_t geomID) const {
          return PrimLeafDesc(0,geomID,GeometryFlags::NONE,gmask,PrimLeafDesc::TYPE_OPACITY_CULLING_ENABLED);
        }
       
        uint8_t gmask;
      };
      
      /* instance data for leaf creation */
      struct Instance
      {
        Instance (AffineSpace3f local2world, void* accel, uint8_t imask)
          : local2world(local2world), accel(accel), imask(imask) {}
        
        AffineSpace3f local2world;
        void* accel;
        uint8_t imask;
      };
      
      /*! settings for SAH builder */
      struct Settings
      {
      public:
        size_t maxDepth = 32;        //!< maximum depth of BVH to build
        size_t sahBlockSize = 6;     //!< blocksize for SAH heuristic
        size_t leafSize[NUM_TYPES] = { 9,9,6,6,6 }; //!< target size of a leaf
        size_t typeSplitSize = 128;  //!< number of primitives when performing type splitting
      };
      
      /*! recursive state of builder */
      struct BuildRecord
      {
      public:
        __forceinline BuildRecord () {}
        
        __forceinline BuildRecord (size_t depth, const PrimInfoRange& prims, Type type)
          : depth(depth), prims(prims), type(type) {}
        
        __forceinline BBox3fa bounds() const { return prims.geomBounds; }
        
        __forceinline friend bool operator< (const BuildRecord& a, const BuildRecord& b) { return a.prims.size() < b.prims.size(); }
        __forceinline friend bool operator> (const BuildRecord& a, const BuildRecord& b) { return a.prims.size() > b.prims.size();  }
        
        __forceinline size_t begin() const { return prims.begin(); }
        __forceinline size_t end  () const { return prims.end(); }
        __forceinline size_t size () const { return prims.size(); }
        __forceinline bool   equalType() const { return type != UNKNOWN; }
        
        friend inline std::ostream& operator<<(std::ostream& cout, const BuildRecord& r) {
          return cout << "BuildRecord { depth = " << r.depth << ", pinfo = " << r.prims << ", type = " << r.type << " }";
        }
        
      public:
        size_t depth;        //!< Depth of the root of this subtree.
        PrimInfoRange prims; //!< The list of primitives.
        Type type;           //!< shared type when type of primitives are equal otherwise UNKNOWN
      };
      
      struct PrimRange
      {
        PrimRange () : block_delta(0), cur_prim(0) {}
        
        PrimRange (uint8_t block_delta, uint8_t start_prim = 0)
          : block_delta(block_delta), cur_prim(start_prim)
        {
          assert(block_delta < 4);
          assert(start_prim < 16);
        }
        
        friend std::ostream& operator<<(std::ostream& cout,const PrimRange& range) {
          return cout << "PrimRange { " << (int)range.block_delta << ", " << (int)range.cur_prim << " }";
        }
        
      public:
        uint8_t block_delta;
        uint8_t cur_prim;
      };
      
      struct ReductionTy
      {
        ReductionTy() {}
        ReductionTy (void* node, NodeType type, uint8_t nodeMask, PrimRange primRange)
          : node((char*)node), type(type), nodeMask(nodeMask), primRange(primRange) {}
        
      public:
        char* node;
        NodeType type;
        uint8_t nodeMask;
        PrimRange primRange;
      };
      
      class ProceduralLeafBuilder
      {
      public:
        
        ProceduralLeafBuilder (char* data, size_t numBlocks)
          : data(data), numBlocks(numBlocks), prevBlockID(0), currBlockID(0), currProcedural(nullptr) {}
        
        ProceduralLeaf* getCurProcedural()
        {
          if (!currProcedural)
          {
            assert(numBlocks);
            currProcedural = new (data) ProceduralLeaf();
            data += sizeof(ProceduralLeaf); numBlocks--;
          }
          return currProcedural;
        }
        
        PrimRange addProcedural(uint32_t geomID, uint32_t primID, const Procedural* procedural, bool last)
        {
          assert(currProcedural);
          
          if (!currProcedural->add(procedural->desc(geomID),primID,last))
          {
            assert(numBlocks);
            currProcedural = (ProceduralLeaf*) data;
            data += sizeof(ProceduralLeaf); numBlocks--;
            
            new (currProcedural) ProceduralLeaf(procedural->desc(geomID),primID,last);
            currBlockID+=1;
          }
          
          uint32_t blockDelta = currBlockID - prevBlockID;
          uint32_t currPrim = (uint32_t)currProcedural->size() - 1;
          prevBlockID = currBlockID;
          
          return PrimRange(blockDelta,currPrim);
        }
        
      protected:
        char*                data;
        size_t               numBlocks;
        uint32_t             prevBlockID;
        uint32_t             currBlockID;
        ProceduralLeaf*      currProcedural;
      };
      
      template<typename getSizeFunc,
               typename getTypeFunc,
               typename getNumTimeSegmentsFunc,
               typename createPrimRefArrayFunc,
               typename getTriangleFunc,
               typename getTriangleIndicesFunc,
               typename getQuadFunc,
               typename getProceduralFunc,
               typename getInstanceFunc>
      class BuilderT
      {
      public:
        static const size_t BINS = 32;
        typedef HeuristicArrayBinningSAH<PrimRef,BINS> CentroidBinner;
        
        FastAllocator::CachedAllocator createAlloc() {
          return allocator.getCachedAllocator();
        };
        
        BuilderT (Device* device,
                  const getSizeFunc& getSize,
                  const getTypeFunc& getType,
                  const getNumTimeSegmentsFunc& getNumTimeSegments,
                  const createPrimRefArrayFunc& createPrimRefArray,
                  const getTriangleFunc& getTriangle,
                  const getTriangleIndicesFunc& getTriangleIndices,
                  const getQuadFunc& getQuad,
                  const getProceduralFunc& getProcedural,
                  const getInstanceFunc& getInstance,
                  bool verbose)
          : getSize(getSize),
            getType(getType),
            getNumTimeSegments(getNumTimeSegments),
            createPrimRefArray(createPrimRefArray),
            getTriangle(getTriangle),
            getTriangleIndices(getTriangleIndices),
            getQuad(getQuad),
            getProcedural(getProcedural),
            getInstance(getInstance),
            prims(device),
            allocator(device, false, true, false),
            verbose(verbose) {} 
        
        ReductionTy setInternalNode(char* curAddr, size_t curBytes, NodeType nodeTy, char* childAddr,
                                    BuildRecord children[BVH_WIDTH], ReductionTy values[BVH_WIDTH], size_t numChildren)
        {
          assert(curBytes >= sizeof(QBVH6::InternalNode6));
          assert(numChildren <= QBVH6::InternalNode6::NUM_CHILDREN);
          
          BBox3f bounds = empty;
          for (size_t i=0; i<numChildren; i++)
            bounds.extend(children[i].bounds());
          
          QBVH6::InternalNode6* qnode = new (curAddr) QBVH6::InternalNode6(bounds,nodeTy);
          qnode->setChildOffset(childAddr);
          
          uint8_t nodeMask = 0;
          for (uint32_t i = 0; i < numChildren; i++)
          {
            qnode->setChild(i,children[i].bounds(),values[i].type,values[i].primRange.block_delta);
            nodeMask |= values[i].nodeMask;
          }
          qnode->nodeMask = nodeMask;
          
          return ReductionTy(curAddr, NODE_TYPE_INTERNAL, nodeMask, PrimRange(curBytes/64));
        }
        
        ReductionTy setNode(char* curAddr, size_t curBytes, NodeType nodeTy, char* childAddr,
                            BuildRecord children[BVH_WIDTH], ReductionTy values[BVH_WIDTH], size_t numChildren)
        {
          return setInternalNode(curAddr,curBytes,nodeTy,childAddr,children,values,numChildren);
        }
        
        QuadLeaf getTriangleInternal(unsigned int geomID, unsigned int primID)
        {
          QBVH6BuilderSAH::Triangle tri = getTriangle(geomID,primID);
          const Vec3f p0 = tri.p0;
          const Vec3f p1 = tri.p1;
          const Vec3f p2 = tri.p2;
          Vec3f p3 = p2;
          
          uint8_t lb0 = 0,lb1 = 0,lb2 = 0;
          uint16_t second = quadification[geomID][primID];
          
          /* handle paired triangle */
          if (second)
          {
            QBVH6BuilderSAH::Triangle tri1 = getTriangle(geomID,primID+second);
            assert(tri.gflags == tri1.gflags);
            assert(tri.gmask  == tri1.gmask );
            
            bool pair MAYBE_UNUSED = pair_triangles(Vec3<uint32_t>(tri.i0,tri.i1,tri.i2),Vec3<uint32_t>(tri1.i0,tri1.i1,tri1.i2),lb0,lb1,lb2);
            assert(pair);
            
            if (lb0 == 3) p3 = tri1.p0;
            if (lb1 == 3) p3 = tri1.p1;
            if (lb2 == 3) p3 = tri1.p2;
          }
          
          return QuadLeaf( p0,p1,p2,p3, lb0,lb1,lb2, 0, geomID, primID, primID+second, tri.gflags, tri.gmask, false );
        };
        
        QuadLeaf createQuadLeaf(Type ty, const PrimRef& prim)
        {
          const unsigned int geomID = prim.geomID();
          const unsigned int primID = prim.primID();
          
          if (ty == TRIANGLE)
            return getTriangleInternal(geomID, primID);
          else
          {
            assert(ty == QUAD);
            const Quad quad = getQuad(geomID,primID);
            return QuadLeaf(quad.p0,quad.p1,quad.p3,quad.p2, 3,2,1, 0, geomID, primID, primID, quad.gflags, quad.gmask, false );
          }
        }
        
        const ReductionTy createQuads(Type ty, const BuildRecord& curRecord, char* curAddr_)
        {
          QuadLeaf* curAddr = (QuadLeaf*) curAddr_;
          uint8_t nodeMask = 0;
          for (size_t i = curRecord.begin(); i < curRecord.end(); i++, curAddr++)
          {
            *curAddr = createQuadLeaf(ty,prims[i]);
            curAddr->last = (i+1) == curRecord.end();
            nodeMask |= curAddr->leafDesc.geomMask;
          }
          return ReductionTy(curAddr, NODE_TYPE_QUAD, nodeMask, PrimRange(curRecord.size()*sizeof(QuadLeaf)/64));
        }
        
        const ReductionTy createFatQuadLeaf(Type ty, FastAllocator::CachedAllocator alloc, const BuildRecord& curRecord, char* curAddr, size_t curBytes,
                                            BuildRecord children[BVH_WIDTH], size_t numChildren)
        {
          /*! allocate data for all children */
          char* childData = (char*) alloc.malloc1(curRecord.prims.size()*sizeof(QuadLeaf), 64);
          
          /* create each child */
          ReductionTy values[BVH_WIDTH];
          for (size_t i=0, j=0; i<numChildren; i++) {
            values[i] = createQuads(ty,children[i],childData+j*sizeof(QuadLeaf));
            j += children[i].size();
          }
          
          return setNode(curAddr,curBytes,NODE_TYPE_QUAD,childData,children,values,numChildren);
        }
        
        const ReductionTy createProcedurals(FastAllocator::CachedAllocator alloc, const BuildRecord& curRecord, char* curAddr, size_t curBytes)
        {
          const uint32_t numPrims MAYBE_UNUSED = curRecord.size();
          assert(numPrims <= QBVH6::InternalNode6::NUM_CHILDREN);
          
          PrimRange ranges[QBVH6::InternalNode6::NUM_CHILDREN+1];
          QBVH6::InternalNode6* qnode = new (curAddr) QBVH6::InternalNode6(curRecord.bounds(),NODE_TYPE_PROCEDURAL);
          
          /* allocate data for all procedural leaves */
          size_t numGeometries = 1;
          auto prim0 = prims[curRecord.begin()];
          auto desc0 = getProcedural(prim0.geomID(),prim0.primID()).desc(prim0.geomID());
          for (size_t i=curRecord.begin()+1; i<curRecord.end(); i++) {
            auto desc1 = getProcedural(prims[i].geomID(),prims[i].primID()).desc(prims[i].geomID());
            numGeometries += desc0 != desc1;
            desc0 = desc1;
          }
          
          char* childData = (char*) alloc.malloc1(numGeometries*sizeof(ProceduralLeaf), 64);
          
          ProceduralLeafBuilder procedural_leaf_builder(childData, numGeometries);
          ProceduralLeaf* first_procedural = procedural_leaf_builder.getCurProcedural();
          
          uint8_t nodeMask = 0;
          for (size_t i = curRecord.begin(), j=0; i < curRecord.end(); i++, j++)
          {
            const uint32_t geomID = prims[i].geomID();
            const uint32_t primID = prims[i].primID();
            auto procedural = getProcedural(geomID,primID);
            ranges[j] = procedural_leaf_builder.addProcedural(geomID,primID,&procedural,true);
            nodeMask |= procedural.gmask;
          }
          // FIXME: explicitely set ranges[numPrims]!
          
          qnode->setChildOffset(first_procedural + ranges[0].block_delta);
          qnode->nodeMask = nodeMask;
          ranges[0].block_delta = 0;
          
          for (size_t i = curRecord.begin(), j=0; i < curRecord.end(); i++, j++)
            qnode->setChild(j,prims[i].bounds(),NODE_TYPE_PROCEDURAL,ranges[j+1].block_delta,ranges[j].cur_prim);
          
          return ReductionTy(curAddr, NODE_TYPE_INTERNAL, nodeMask, PrimRange(curBytes/64));
        }
        
        const ReductionTy createInstances(FastAllocator::CachedAllocator alloc, const BuildRecord& curRecord, char* curAddr, size_t curBytes)
        {
          uint32_t numPrimitives = curRecord.size();
          assert(numPrimitives <= QBVH6::InternalNode6::NUM_CHILDREN);
          
          /* allocate data for all children */
          InstanceLeaf* childData = (InstanceLeaf*) alloc.malloc1(numPrimitives*sizeof(InstanceLeaf), 64);
          
          QBVH6::InternalNode6* qnode = new (curAddr) QBVH6::InternalNode6(curRecord.bounds(),NODE_TYPE_INSTANCE);
          qnode->setChildOffset(childData);
          
          uint8_t nodeMask = 0;
          for (size_t i=curRecord.begin(), c=0; i<curRecord.end(); i++, c++)
          {
            const uint32_t geomID = prims[i].geomID();
            const int64_t  rootOfs = (int32_t) prims[i].primID();
            const Instance instance = getInstance(geomID,0); //primID);
            
            uint64_t root = static_cast<QBVH6*>(instance.accel)->root();
            root += 64*rootOfs; // goto sub-BVH
            new (&childData[c]) InstanceLeaf(instance.local2world,root,geomID,instance.imask);
            
            qnode->setChild(c,prims[i].bounds(),NODE_TYPE_INSTANCE,sizeof(InstanceLeaf)/64,0);
            nodeMask |= instance.imask;
          }
          qnode->nodeMask = nodeMask;
          
          return ReductionTy(curAddr, NODE_TYPE_INTERNAL, nodeMask, PrimRange(curBytes/64));
        }
        
        /* finds the index of the child with largest surface area */
        int findChildWithLargestArea(BuildRecord children[BVH_WIDTH], size_t numChildren, size_t leafThreshold)
        {
          /*! find best child to split */
          float bestArea = neg_inf;
          int bestChild = -1;
          for (uint32_t i=0; i<(uint32_t)numChildren; i++)
          {
            /* ignore leaves as they cannot get split */
            if (children[i].prims.size() <= leafThreshold) continue;
            
            /* find child with largest surface area */
            const float area = halfArea(children[i].prims.geomBounds);
            if (area > bestArea)
            {
              bestArea = area;
              bestChild = i;
            }
          }
          return bestChild;
        }
        
        /* finds the index of the child with most primitives */
        int findChildWithMostPrimitives(BuildRecord children[BVH_WIDTH], size_t numChildren, size_t leafThreshold)
        {
          /* find best child with largest size */
          size_t  bestSize = 0;
          int bestChild = -1;
          for (uint32_t i=0; i<(uint32_t)numChildren; i++)
          {
            /* ignore leaves as they cannot get split */
            if (children[i].prims.size() <= leafThreshold) continue;
            
            /* remember child with largest size */
            if (children[i].prims.size() > bestSize)
            {
              bestSize = children[i].size();
              bestChild = i;
            }
          }
          return bestChild;
        }
        
        /* finds the index of the child with most primitives */
        int findChildWithNonEqualTypes(BuildRecord children[BVH_WIDTH], size_t numChildren)
        {
          for (uint32_t i=0; i<(uint32_t)numChildren; i++)
            if (!children[i].equalType())
              return i;
          
          return -1;
        }
        
        void SAHSplit(size_t depth, size_t sahBlockSize, int bestChild, BuildRecord children[BVH_WIDTH], size_t& numChildren)
        {
          PrimInfoRange linfo, rinfo;
          BuildRecord brecord = children[bestChild];
          
          /* first perform centroid binning */
          CentroidBinner centroid_binner(prims.data());
          CentroidBinner::Split bestSplit = centroid_binner.find_block_size(brecord.prims,sahBlockSize);
          
          /* now split the primitive list */
          if (bestSplit.valid())
            centroid_binner.split(bestSplit,brecord.prims,linfo,rinfo);
          
          /* the above techniques may fail, and we fall back to some brute force split in the middle */
          else
            centroid_binner.splitFallback(brecord.prims,linfo,rinfo);
          
          children[bestChild  ] = BuildRecord(depth+1, linfo, brecord.type);
          children[numChildren] = BuildRecord(depth+1, rinfo, brecord.type);
          numChildren++;
        }
        
        void TypeSplit(size_t depth, int bestChild, BuildRecord children[BVH_WIDTH], size_t& numChildren)
        {
          BuildRecord brecord = children[bestChild];
          
          PrimInfoRange linfo, rinfo;
          auto type = getType(prims[brecord.prims.begin()].geomID());
          performTypeSplit(getType,type,prims.data(),brecord.prims.get_range(),linfo,rinfo);
          
          for (size_t i=linfo.begin(); i<linfo.end(); i++)
            assert(getType(prims[i].geomID()) == getType(prims[linfo.begin()].geomID()));
          
          bool equalTy = true;
          Type rtype = getType(prims[rinfo.begin()].geomID());
          for (size_t i=rinfo.begin()+1; i<rinfo.end(); i++)
            equalTy &= rtype == getType(prims[i].geomID());
          
          children[bestChild  ] = BuildRecord(depth+1, linfo, type);
          children[numChildren] = BuildRecord(depth+1, rinfo, equalTy ? rtype : UNKNOWN);
          numChildren++;
        }
        
        void FallbackSplit(size_t depth, int bestChild, BuildRecord children[BVH_WIDTH], size_t& numChildren)
        {
          BuildRecord brecord = children[bestChild];
          
          PrimInfoRange linfo, rinfo;
          performFallbackSplit(prims.data(),brecord.prims,linfo,rinfo);
          
          children[bestChild  ] = BuildRecord(depth+1, linfo, brecord.type);
          children[numChildren] = BuildRecord(depth+1, rinfo, brecord.type);
          
          numChildren++;
        }
        
        /* creates a fat leaf, which is an internal node that only points to real leaves */
        const ReductionTy createFatLeaf(FastAllocator::CachedAllocator alloc, const BuildRecord& curRecord, char* curAddr, size_t curBytes)
        {
          /* there should be at least one primitive and not too many */
          assert(curRecord.size() > 0);
          assert(curRecord.size() <= cfg.leafSize[curRecord.type]);
          
          /* all primitives have to have the same type */
          Type ty = getType(prims[curRecord.begin()].geomID());
          for (size_t i=curRecord.begin(); i<curRecord.end(); i++)
            assert(getType(prims[i].geomID()) == ty);
          
          /*! initialize child list with first child */
          BuildRecord children[BVH_WIDTH];
          size_t numChildren = 0;
          
          /* fast path when we can put one primitive per child */
          if (curRecord.size() <= BVH_WIDTH)
          {
            for (size_t j=curRecord.begin(); j<curRecord.end(); j++)
            {
              CentGeomBBox3fa b(empty); b.extend_primref(prims[j]);
              children[numChildren++] = BuildRecord(curRecord.depth+1, PrimInfoRange(j,j+1,b), curRecord.type);
            }
          }
          
          else
          {
            /*! initialize child list with first child */
            children[0] = curRecord;
            numChildren = 1;
            
            /*! split until node is full */
            while (numChildren < BVH_WIDTH)
            {
              const int bestChild = findChildWithMostPrimitives(children,numChildren,1);
              if (bestChild == -1) break;
              SAHSplit(curRecord.depth,1,bestChild,children,numChildren);
            }
            
            /* fallback in case largest leaf if still too large */
            const int bestChild = findChildWithMostPrimitives(children,numChildren,1);
            if (bestChild != -1 && children[bestChild].size() > 3)
            {
              children[0] = curRecord;
              numChildren = 1;
              
              /*! perform fallback splits until node is full */
              while (numChildren < BVH_WIDTH)
              {
                const int bestChild = findChildWithMostPrimitives(children,numChildren,1);
                if (bestChild == -1) break;
                FallbackSplit(curRecord.depth,bestChild,children,numChildren);
              }
            }
          }
          
          /* sort build records for faster shadow ray traversal */
          std::sort(children,children+numChildren, [](const BuildRecord& a,const BuildRecord& b) {
                                                     return area(a.prims.geomBounds) > area(b.prims.geomBounds);
                                                   });
          
          /* create leaf of proper type */
          if (ty == TRIANGLE || ty == QUAD)
            return createFatQuadLeaf(ty, alloc, curRecord, curAddr, curBytes, children, numChildren);
          else if (ty == PROCEDURAL)
            return createProcedurals(alloc, curRecord,curAddr,curBytes);
          else if (ty == INSTANCE)
            return createInstances(alloc, curRecord,curAddr,curBytes);
          else
            assert(false);
          
          return ReductionTy();
        }
        
        const ReductionTy createLargeLeaf(FastAllocator::CachedAllocator alloc, const BuildRecord& curRecord, char* curAddr, size_t curBytes)
        {
          /* this should never occur but is a fatal error */
          assert(curRecord.depth <= cfg.maxDepth);
          
          /* all primitives have to have the same type */
          Type ty MAYBE_UNUSED = getType(prims[curRecord.begin()].geomID());
          for (size_t i=curRecord.begin(); i<curRecord.end(); i++)
            assert(getType(prims[i].geomID()) == ty);
          
          /* create leaf for few primitives */
          if (curRecord.prims.size() <= cfg.leafSize[curRecord.type])
            return createFatLeaf(alloc, curRecord,curAddr,curBytes);
          
          /*! initialize child list with first child */
          ReductionTy values[BVH_WIDTH];
          BuildRecord children[BVH_WIDTH];
          size_t numChildren = 1;
          children[0] = curRecord;
          
          /* fill all children by always splitting the largest one */
          while (numChildren < BVH_WIDTH)
          {
            const int bestChild = findChildWithMostPrimitives(children,numChildren,cfg.leafSize[curRecord.type]);
            if (bestChild == -1) break;
            FallbackSplit(curRecord.depth,bestChild,children,numChildren);
          }
          
          /*! allocate data for all children */
          size_t childrenBytes = numChildren*sizeof(QBVH6::InternalNode6);
          char* childBase = (char*) alloc.malloc0(childrenBytes, 64);
          
          /* recurse into each child  and perform reduction */
          char* childPtr = childBase;
          for (size_t i=0; i<numChildren; i++) {
            values[i] = createLargeLeaf(alloc,children[i],childPtr,sizeof(QBVH6::InternalNode6));
            childPtr += sizeof(QBVH6::InternalNode6);
          }
          
          return setNode(curAddr,curBytes,NODE_TYPE_INTERNAL,childBase,children,values,numChildren);
        }
        
        const ReductionTy createInternalNode(FastAllocator::CachedAllocator alloc, BuildRecord& curRecord, char* curAddr, size_t curBytes)
        {
          /* get thread local allocator */
          if (!alloc)
            alloc = createAlloc();
          
          /* create leaf when threshold reached or we are too deep */
          bool createLeaf = curRecord.prims.size() <= cfg.leafSize[curRecord.type] || 
            curRecord.depth+MIN_LARGE_LEAF_LEVELS >= cfg.maxDepth;
          
          bool performTypeSplit = !curRecord.equalType() && (createLeaf || curRecord.size() <= cfg.typeSplitSize);
          
          /* check if types are really not equal when we attempt to split by type */
          if (performTypeSplit)
          {
            /* check if types are already equal */
            bool equalTy = true;
            Type type = getType(prims[curRecord.begin()].geomID());
            for (size_t i=curRecord.begin()+1; i<curRecord.end(); i++)
              equalTy &= getType(prims[i].geomID()) == type;
            
            curRecord.type = equalTy ? type : UNKNOWN;
            performTypeSplit &= !curRecord.equalType();
          }
          
          /* create leaf node */
          if (!performTypeSplit && createLeaf)
            return createLargeLeaf(alloc,curRecord,curAddr,curBytes);
          
          /*! initialize child list with first child */
          ReductionTy values[BVH_WIDTH];
          BuildRecord children[BVH_WIDTH];
          children[0] = curRecord;
          size_t numChildren = 1;
          
          /*! perform type splitting */
          if (performTypeSplit)
          {
            /*! split until node is full */
            while (numChildren < BVH_WIDTH)
            {
              const int bestChild = findChildWithNonEqualTypes(children,numChildren);
              if (bestChild == -1) break;
              TypeSplit(curRecord.depth,bestChild,children,numChildren);
            }
          }
          
          /*! perform SAH splits until node is full */
          while (numChildren < BVH_WIDTH)
          {
            const int bestChild = findChildWithLargestArea(children,numChildren,cfg.leafSize[curRecord.type]);
            if (bestChild == -1) break;
            SAHSplit(curRecord.depth,cfg.sahBlockSize,bestChild,children,numChildren);
          }
          
          /* sort build records for faster shadow ray traversal */
          std::sort(children,children+numChildren,std::less<BuildRecord>());
          
          /*! allocate data for all children */
          size_t childrenBytes = numChildren*sizeof(QBVH6::InternalNode6);
          char* childBase = (char*) alloc.malloc0(childrenBytes, 64);
          
          /* spawn tasks */
          if (curRecord.size() > 1024) // cfg.singleThreadThreshold
          {
            parallel_for(size_t(0), numChildren, [&] (const range<size_t>& r) {
              for (size_t i=r.begin(); i<r.end(); i++) {
                values[i] = createInternalNode(nullptr,children[i],childBase+i*sizeof(QBVH6::InternalNode6),sizeof(QBVH6::InternalNode6));
              }
            });
            
            /* create node */
            return setNode(curAddr,curBytes,NODE_TYPE_INTERNAL,childBase,children,values,numChildren);
          }
          
          /* recurse into each child */
          else
          {
            /* recurse into each child */
            for (size_t i=0; i<numChildren; i++) {
              values[i] = createInternalNode(alloc,children[i],childBase+i*sizeof(QBVH6::InternalNode6),sizeof(QBVH6::InternalNode6));
            }
            
            /* create node */
            return setNode(curAddr,curBytes,NODE_TYPE_INTERNAL,childBase,children,values,numChildren);
          }
        }

        const ReductionTy createEmptyNode(char* addr)
        {
          const size_t curBytes = sizeof(QBVH6::InternalNode6);
          new (addr) QBVH6::InternalNode6(NODE_TYPE_INTERNAL);
          return ReductionTy(addr, NODE_TYPE_INTERNAL, 0x00, PrimRange(curBytes/64));
        }
        
        PrimInfo createTrianglePairPrimRefArray(PrimRef* prims, const range<size_t>& r, size_t k, unsigned int geomID)
        {
          PrimInfo pinfo(empty);
          for (size_t j=r.begin(); j<r.end(); j++)
          {
            uint16_t pair = quadification[geomID][j];
            if (pair == QUADIFIER_PAIRED) continue;
            
            BBox3fa bounds = empty;
            Triangle tri0 = getTriangle(geomID,j);
            bounds.extend(tri0.p0);
            bounds.extend(tri0.p1);
            bounds.extend(tri0.p2);
            if (!tri0.valid()) continue;
            
            if (pair != QUADIFIER_TRIANGLE)
            {
              Triangle tri1 = getTriangle(geomID,j+pair);
              bounds.extend(tri1.p0);
              bounds.extend(tri1.p1);
              bounds.extend(tri1.p2);
              if (!tri1.valid()) continue;
            }
            
            const PrimRef prim(bounds,geomID,unsigned(j));
            pinfo.add_center2(prim);
            prims[k++] = prim;
          }
          return pinfo;
        }

        void splitTrianglePair(const PrimRef& prim, const size_t dim, const float pos, PrimRef& left_o, PrimRef& right_o) const
        {
          const uint32_t geomID = prim.geomID();
          const uint32_t primID = prim.primID();
          const uint16_t pair = quadification[geomID][primID];
          assert(pair != QUADIFIER_PAIRED);

          const Triangle tri0 = getTriangle(geomID,primID);
          const Vec3fa v[4] = { tri0.p0, tri0.p1, tri0.p2, tri0.p0 };

          BBox3fa left,right;
          splitPolygon<3>(prim.bounds(),dim,pos,v,left,right);

          if (pair != QUADIFIER_TRIANGLE)
          {
            const Triangle tri1 = getTriangle(geomID,primID+pair);
            const Vec3fa v[4] = { tri1.p0, tri1.p1, tri1.p2, tri1.p0 };

            BBox3fa left1, right1;
            splitPolygon<3>(prim.bounds(),dim,pos,v,left1,right1);

            left.extend(left1);
            right.extend(right1);
          }

          left_o  = PrimRef(left , geomID, primID);
          right_o = PrimRef(right, geomID, primID);
        }

        void splitQuad(const PrimRef& prim, const size_t dim, const float pos, PrimRef& left_o, PrimRef& right_o) const
        {
          const uint32_t geomID = prim.geomID();
          const uint32_t primID = prim.primID();
          const Quad quad = getQuad(geomID,primID);
          const Vec3fa v[5] = { quad.p0, quad.p1, quad.p2, quad.p3, quad.p0 };
          splitPolygon<4>(prim,dim,pos,v,left_o,right_o);
        }

        void splitTriangleOrQuad(const PrimRef& prim, const size_t dim, const float pos, PrimRef& left_o, PrimRef& right_o) const
        {
          switch (getType(prim.geomID())) {
          case TRIANGLE: splitTrianglePair(prim,dim,pos,left_o,right_o); break;
          case QUAD    : splitQuad        (prim,dim,pos,left_o,right_o); break;
          default: assert(false); break;
          }
        }

        void openInstance(const PrimRef& prim,
                          const unsigned int splitprims,
                          PrimRef subPrims[MAX_PRESPLITS_PER_PRIMITIVE],
                          unsigned int& numSubPrims)
        {
          struct Item
          {
            QBVH6::InternalNode6* node;
            float priority;

            Item () {}
            
            Item (QBVH6::InternalNode6* node)
              : node(node), priority(halfArea(node->bounds()))
            {
              /* fat leaves cannot get opened */
              if (node->isFatLeaf())
                priority = 0.0f;
            }

            inline bool operator< ( const Item& other) const {
              return priority < other.priority;
            }
          };
          
          const uint32_t targetSubPrims = splitprims;
          const uint32_t geomID = prim.geomID();
          const uint32_t primID MAYBE_UNUSED = prim.primID();
          assert(primID == 0); // has to be zero as we encode root offset here

          const Instance instance = getInstance(geomID,0);
          QBVH6::InternalNode6* root = static_cast<QBVH6*>(instance.accel)->root().innerNode<QBVH6::InternalNode6>();
          
          darray_t<Item,MAX_PRESPLITS_PER_PRIMITIVE> heap;
          heap.push_back(root);

          while (heap.size() + (QBVH6::InternalNode6::NUM_CHILDREN-1) <= MAX_PRESPLITS_PER_PRIMITIVE)
          {
            /* terminate when budget exceeded */
            if (heap.size() >= targetSubPrims)
              break;

            /* get top heap element */
            std::pop_heap(heap.begin(), heap.end());
            auto top = heap.back();

            /* if that happens there are only leaf nodes left that cannot get opened */
            if (top.priority == 0.0f) break;
            heap.pop_back();

            /* add all children to the heap */
            for (uint32_t i=0; i<QBVH6::InternalNode6::NUM_CHILDREN; i++)
            {
              if (!top.node->valid(i)) continue;
              heap.push_back(top.node->child(i).template innerNode<QBVH6::InternalNode6>());
              std::push_heap(heap.begin(), heap.end());
            }
          }

          /* create primrefs */
          for (size_t i=0; i<heap.size(); i++)
          {
            QBVH6::InternalNode6* node = heap[i].node;
            BBox3fa bounds = xfmBounds(instance.local2world,node->bounds());
            int64_t ofs = ((int64_t)node-(int64_t)root)/64;
            assert(ofs >= INT_MIN && ofs <= INT_MAX);
            subPrims[numSubPrims++] = PrimRef(bounds,geomID,(int32_t)ofs);
          }
        }

        float primitiveAreaTrianglePair(const PrimRef& prim)
        {
          const uint32_t geomID = prim.geomID();
          const uint32_t primID = prim.primID();
          
          const uint16_t pair = quadification[geomID][primID];
          assert(pair != QUADIFIER_PAIRED);

          const Triangle tri0 = getTriangle(geomID,primID);
          float A = areaProjectedTriangle(tri0.p0,tri0.p1,tri0.p2);
          if (pair == QUADIFIER_TRIANGLE)
            return A;

          const Triangle tri1 = getTriangle(geomID,primID+pair);
          A += areaProjectedTriangle(tri1.p0,tri1.p1,tri1.p2);
          return A;
        }

        float primitiveAreaQuad(const PrimRef& prim)
        {
          const uint32_t geomID = prim.geomID();
          const uint32_t primID = prim.primID();
          const Quad quad = getQuad(geomID,primID);
          const float A0 = areaProjectedTriangle(quad.p0,quad.p1,quad.p3);
          const float A1 = areaProjectedTriangle(quad.p2,quad.p3,quad.p1);
          return A0+A1;
        }

        float primitiveAreaInstance(const PrimRef& prim) {
          return halfArea(prim.bounds());
        }

        float primitiveArea(const PrimRef& prim)
        {
          switch (getType(prim.geomID())) {
          case TRIANGLE: return primitiveAreaTrianglePair(prim);
          case QUAD    : return primitiveAreaQuad(prim);
          case INSTANCE: return primitiveAreaInstance(prim);
          default      : return 0.0f;
          }
        }

        ReductionTy build(uint32_t numGeometries, Device::avector<char,64>& accel, PrimInfo& pinfo_o, char* root)
        {
          PING;
          PRINT(numGeometries);
          
          double t1 = verbose ? getSeconds() : 0.0;

          auto getSizeStatic = [&]( unsigned int geomID ) {
            if (getNumTimeSegments(geomID) == 0) return getSize(geomID);
            else                                 return size_t(0);
          };
          
          /* quadify all triangles */
          ParallelForForPrefixSumState<PrimInfo> pstate;
          pstate.init(numGeometries,getSizeStatic,size_t(1024));
          PrimInfo pinfo = parallel_for_for_prefix_sum0_( pstate, size_t(1), getSizeStatic, PrimInfo(empty), [&](size_t geomID, const range<size_t>& r, size_t k) -> PrimInfo {
            if (getType(geomID) == QBVH6BuilderSAH::TRIANGLE)
              return PrimInfo(pair_triangles(geomID,(QuadifierType*) quadification[geomID].data(), r.begin(), r.end(), getTriangleIndices));
            else
              return PrimInfo(r.size());
          }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });

          double t2 = verbose ? getSeconds() : 0.0;
          if (verbose) std::cout << "quadification: " << std::setw(10) << (t2-t1)*1000.0 << "ms, " << std::endl; //<< std::setw(10) << 1E-6*double(numTriangles)/(t2-t1) << " Mtris/s" << std::endl;

          size_t numPrimitives = pinfo.size();
          
          /* first try */
          //pstate.init(numGeometries,getSizeStatic,size_t(1024));
          pinfo = parallel_for_for_prefix_sum1_( pstate, size_t(1), getSizeStatic, PrimInfo(empty), [&](size_t geomID, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo {
            if (getType(geomID) == QBVH6BuilderSAH::TRIANGLE)
              return createTrianglePairPrimRefArray(prims.data(),r,base.size(),(unsigned)geomID);
            else
              return createPrimRefArray(prims,BBox1f(0,1),r,base.size(),(unsigned)geomID);
          }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });

          double t3 = verbose ? getSeconds() : 0.0;
          if (verbose) std::cout << "primrefgen   : " << std::setw(10) << (t3-t2)*1000.0 << "ms, " << std::setw(10) << 1E-6*double(numPrimitives)/(t3-t2) << " Mprims/s" << std::endl;
          
          /* if we need to filter out geometry, run again */
          if (pinfo.size() != numPrimitives)
          {
            numPrimitives = pinfo.size();
            
            pinfo = parallel_for_for_prefix_sum1_( pstate, size_t(1), getSizeStatic, PrimInfo(empty), [&](size_t geomID, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo {
              if (getType(geomID) == QBVH6BuilderSAH::TRIANGLE) {
                return createTrianglePairPrimRefArray(prims.data(),r,base.size(),(unsigned)geomID);
              }
              else                                                                               
                return createPrimRefArray(prims,BBox1f(0,1),r,base.size(),(unsigned)geomID);
            }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
          }
          assert(pinfo.size() == numPrimitives);
          
          double t4 = verbose ? getSeconds() : 0.0;
          if (verbose) std::cout << "primrefgen2  : " << std::setw(10) << (t4-t3)*1000.0 << "ms, " << std::setw(10) << 1E-6*double(numPrimitives)/(t4-t3) << " Mprims/s" << std::endl;
          
          /* perform pre-splitting */
          if (numPrimitives)
          {
            auto splitter = [this] (const PrimRef& prim, const size_t dim, const float pos, PrimRef& left_o, PrimRef& right_o) {
              splitTriangleOrQuad(prim,dim,pos,left_o,right_o);
            };

            auto splitter1 = [&] (const PrimRef& prim,
                                  const unsigned int splitprims,
                                  const SplittingGrid& grid,
                                  PrimRef subPrims[MAX_PRESPLITS_PER_PRIMITIVE],
                                  unsigned int& numSubPrims)
            {
              if (getType(prim.geomID()) == QBVH6BuilderSAH::INSTANCE) {
                openInstance(prim,splitprims,subPrims,numSubPrims);
              } else {
                splitPrimitive(splitter,prim,splitprims,grid,subPrims,numSubPrims);
              }
            };

            auto primitiveArea1 = [this] (const PrimRef& prim) -> float {
              return primitiveArea(prim);
            };
            
            pinfo = createPrimRefArray_presplit(numPrimitives, prims, pinfo, splitter1, primitiveArea1);
          }
            
          /* exit early if scene is empty */
          if (pinfo.size() == 0) {
            pinfo_o = pinfo;
            return createEmptyNode(root);
          }
          
          /* build hierarchy */
          BuildRecord record(1,pinfo,UNKNOWN);
          FastAllocator::CachedAllocator thread_alloc = allocator.getCachedAllocator();
          ReductionTy r = createInternalNode(thread_alloc,record,root,sizeof(QBVH6::InternalNode6));
          
          if (verbose) {
            allocator.cleanup();
            FastAllocator::AllStatistics stats1(&allocator);
            stats1.print(numPrimitives);
          }

          double t5 = verbose ? getSeconds() : 0.0;
          if (verbose) std::cout << "bvh_build    : " << std::setw(10) << (t5-t4)*1000.0 << "ms, " << std::setw(10) << 1E-6*double(numPrimitives)/(t5-t4) << " Mprims/s" << std::endl;

          pinfo_o = pinfo;
          return r;
        }

        ReductionTy build_mblur(uint32_t numGeometries, Device::avector<char,64>& accel, PrimInfo& pinfo_o, char* root, BBox1f time_range)
        {
          auto getSizeMBlur = [&]( unsigned int geomID ) {
            if (getNumTimeSegments(geomID) == 0) return size_t(0);
            else                                 return getSize(geomID);
          };

          size_t numPrimitives = 0;
          for (size_t geomID=0; geomID<numGeometries; geomID++)
            numPrimitives += getSizeMBlur(geomID);

          double t2 = verbose ? getSeconds() : 0.0;
          
          /* first try */
          ParallelForForPrefixSumState<PrimInfo> pstate;
          pstate.init(numGeometries,getSizeMBlur,size_t(1024));
          PrimInfo pinfo = parallel_for_for_prefix_sum0_( pstate, size_t(1), getSizeMBlur, PrimInfo(empty), [&](size_t geomID, const range<size_t>& r, size_t k) -> PrimInfo {
             return createPrimRefArray(prims,time_range,r,k,(unsigned)geomID);
          }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
          
          double t3 = verbose ? getSeconds() : 0.0;
          if (verbose) std::cout << "primrefgen   : " << std::setw(10) << (t3-t2)*1000.0 << "ms, " << std::setw(10) << 1E-6*double(numPrimitives)/(t3-t2) << " Mprims/s" << std::endl;
          
          /* if we need to filter out geometry, run again */
          if (pinfo.size() != numPrimitives)
          {
            numPrimitives = pinfo.size();
            
            pinfo = parallel_for_for_prefix_sum1_( pstate, size_t(1), getSizeMBlur, PrimInfo(empty), [&](size_t geomID, const range<size_t>& r, size_t k, const PrimInfo& base) -> PrimInfo {
              return createPrimRefArray(prims,time_range,r,base.size(),(unsigned)geomID);
            }, [](const PrimInfo& a, const PrimInfo& b) -> PrimInfo { return PrimInfo::merge(a,b); });
          }
          assert(pinfo.size() == numPrimitives);
          
          double t4 = verbose ? getSeconds() : 0.0;
          if (verbose) std::cout << "primrefgen2  : " << std::setw(10) << (t4-t3)*1000.0 << "ms, " << std::setw(10) << 1E-6*double(numPrimitives)/(t4-t3) << " Mprims/s" << std::endl;
          
          /* exit early if scene is empty */
          if (pinfo.size() == 0) {
            pinfo_o = pinfo;
            return createEmptyNode(root);
          }
          
          /* build hierarchy */
          BuildRecord record(1,pinfo,UNKNOWN);
          FastAllocator::CachedAllocator thread_alloc = allocator.getCachedAllocator();
          ReductionTy r = createInternalNode(thread_alloc,record,root,sizeof(QBVH6::InternalNode6));
          
          if (verbose) {
            allocator.cleanup();
            FastAllocator::AllStatistics stats1(&allocator);
            stats1.print(numPrimitives);
          }

          double t5 = verbose ? getSeconds() : 0.0;
          if (verbose) std::cout << "bvh_build    : " << std::setw(10) << (t5-t4)*1000.0 << "ms, " << std::setw(10) << 1E-6*double(numPrimitives)/(t5-t4) << " Mprims/s" << std::endl;

          pinfo_o = pinfo;
          return r;
        }
        
        BBox3f build(size_t numGeometries, Device* device, Device::avector<char,64>& accel, void* dispatchGlobalsPtr)
        {
          double t0 = verbose ? getSeconds() : 0.0;
          
          /* calculate scene size and allocate quadification data */
          struct Stats
          {
            size_t numTriangles = 0;
            size_t numQuads = 0;
            size_t numProcedurals = 0;
            size_t numInstances = 0;

            /* assume some reasonable quadification rate */
            void estimate_quadification()
            {
              numQuads += (numTriangles+1)/2 + numTriangles/8;
              numTriangles = 0;
            }

            void estimate_presplits( double factor )
            {
              numTriangles = max(numTriangles, size_t(numTriangles*factor));
              numQuads     = max(numQuads    , size_t(numQuads*factor));
              numInstances = max(numInstances, size_t(numInstances*factor));
            }

            size_t size() {
              return numTriangles+numQuads+numProcedurals+numInstances;
            }

            size_t expected_bvh_bytes()
            {
              const size_t blocks = (size()+5)/6;
              const size_t expected_bytes   = 128 + 64*size_t(1+1.5*blocks) + numTriangles*64 + numQuads*64 + numProcedurals*8 + numInstances*128;
              return 2*4096 + size_t(1.1*expected_bytes); // FIXME: FastAllocator wastes memory and always allocates 4kB per thread
            }

            size_t worst_case_bvh_bytes()
            {
              const size_t numPrimitives = size();
              const size_t blocks = (numPrimitives+5)/6;
              const size_t worst_case_bytes = 128 + 64*(1+blocks + numPrimitives) + numTriangles*64 + numQuads*64 + numProcedurals*64 + numInstances*128;
              return 2*4096 + size_t(1.1*worst_case_bytes); // FIXME: FastAllocator wastes memory and always allocates 4kB per thread
            }
            
          } static_geom, mblur_geom;
          
          uint32_t maxTimeSegments = 0;
          
          quadification.resize(numGeometries);
          for (size_t geomID=0; geomID<numGeometries; geomID++)
          {
            const uint32_t N = getSize(geomID);
            if (N == 0) continue;

            const uint32_t segments = getNumTimeSegments(geomID);
            maxTimeSegments = max(maxTimeSegments, segments);

            Stats& geom = getNumTimeSegments(geomID) ? mblur_geom : static_geom;
            switch (getType(geomID)) {
            case QBVH6BuilderSAH::TRIANGLE  : geom.numTriangles += N; quadification[geomID].resize(N); break;
            case QBVH6BuilderSAH::QUAD      : geom.numQuads += N; break;
            case QBVH6BuilderSAH::PROCEDURAL: geom.numProcedurals += N; break;
            case QBVH6BuilderSAH::INSTANCE  : geom.numInstances += N; break;
            default: assert(false); break;
            }
          }
          
          double t1 = verbose ? getSeconds() : 0.0;
          if (verbose) std::cout << "scene_size   : " << std::setw(10) << (t1-t0)*1000.0 << "ms" << std::endl;

          size_t numPrimitivesStatic = static_geom.size();
          size_t numPrimitivesStaticExt = size_t(numPrimitivesStatic * 1.2); // 20% extra space for pre-splits of static geometry
          size_t numPrimitivesMB = mblur_geom.size();
          size_t numPrimitives = numPrimitivesStatic + numPrimitivesMB;
          
          size_t numPrimitivesAlloc = max(numPrimitivesStaticExt, numPrimitivesMB); 
          prims.resize(numPrimitivesAlloc);

          static_geom.estimate_quadification();
          static_geom.estimate_presplits(1.2);
          
          /* estimate required bytes for BVH */
          size_t expected_bytes   = static_geom.expected_bvh_bytes()   + maxTimeSegments*mblur_geom.expected_bvh_bytes();
          size_t worst_case_bytes = static_geom.worst_case_bvh_bytes() + maxTimeSegments*mblur_geom.worst_case_bvh_bytes();
          assert(expected_bytes <= worst_case_bytes);

          PrimInfo pinfo;
          BBox3f bounds = empty;

          /* we need potentially multiple tries in case or expected size estimate was too small */
          for (size_t bytes = expected_bytes;; bytes *= 1.2)
          {
            if (verbose) std::cout << "trying BVH build with " << bytes << " bytes" << std::endl;
            
            /* allocate BVH memory */
            allocator.clear();
            if (accel.size() < bytes) accel = std::move(Device::avector<char,64>(device,bytes));
            memset(accel.data(),0,accel.size()); // FIXME: not required
            
            allocator.addBlock(accel.data(),accel.size());
            FastAllocator::CachedAllocator thread_alloc = allocator.getCachedAllocator();
            thread_alloc.malloc0(128-FastAllocator::blockHeaderSize);

            try
            {
              /* allocate a separate root entry node, one for each time segment */
              QBVH6::InternalNode6* entry = nullptr;
              if (maxTimeSegments) {
                entry = (QBVH6::InternalNode6*) thread_alloc.malloc0(maxTimeSegments*sizeof(QBVH6::InternalNode6),64);
                assert(entry);
              }
                
              uint32_t numBVHs = 1+maxTimeSegments;
              ReductionTy r [numBVHs];
              BuildRecord br[numBVHs];

              uint32_t numRoots = maxTimeSegments ? 2*maxTimeSegments : 1;
              QBVH6::InternalNode6* roots = (QBVH6::InternalNode6*) thread_alloc.malloc0(numRoots*sizeof(QBVH6::InternalNode6),64);
              assert(roots);
              
              /* build BVH static BVH */
              r [0] = build(numGeometries,accel,pinfo,(char*)(roots+0));
              bounds.extend(pinfo.geomBounds);
              
              /* build separate BVH for each time segment */
              for (uint32_t t=0; t<maxTimeSegments; t++)
              {
                const float t0 = (t+0)/float(maxTimeSegments);
                const float t1 = (t+1)/float(maxTimeSegments);
                r [t+1] = build_mblur(numGeometries,accel,pinfo,(char*)(roots+2*t+1),BBox1f(t0,t1));
                bounds.extend(pinfo.geomBounds);
                roots->copy_to(roots+2*t+0); // copy static root node to t'th BVH
              }

              for (uint32_t i=0; i<numBVHs; i++) // FIXME: not optimal
                br[i].prims.geomBounds = bounds;

              ReductionTy r0 = r[0];
              if (maxTimeSegments)
              {
                for (uint32_t t=0; t<maxTimeSegments; t++)
                {
                  ReductionTy values[BVH_WIDTH];
                  values[0] = r[0];
                  values[1] = r[1+t];
                  BuildRecord children[BVH_WIDTH];
                  children[0] = br[0];
                  children[1] = br[1+t];
                  ReductionTy r = setNode((char*)(entry+t),sizeof(QBVH6::InternalNode6),NODE_TYPE_INTERNAL,(char*)(roots+2*t),children,values,2);
                  if (t == 0) r0 = r;
                }
              }

              /* fill QBVH6 header */
              allocator.clear();
              QBVH6* qbvh = new (accel.data()) QBVH6(QBVH6::SizeEstimate());
              qbvh->numPrims = numPrimitives;
              uint64_t rootNodeOffset = QBVH6::Node((char*)(r0.node - (char*)qbvh), r0.type, r0.primRange.cur_prim);
              assert(rootNodeOffset == QBVH6::rootNodeOffset);
              qbvh->bounds = bounds;
              qbvh->numTimeSegments = max(1u,maxTimeSegments);
              qbvh->dispatchGlobalsPtr = (uint64_t) dispatchGlobalsPtr;
              break;
            }
            catch (std::bad_alloc&)
            {
              if (verbose) {
                allocator.cleanup();
                FastAllocator::AllStatistics stats1(&allocator);
                stats1.print(numPrimitives);
              }
              
              if (bytes >= worst_case_bytes) {
                throw std::runtime_error("BVH build failed");
                return empty;
              }
            }
          }
          
          /* print BVH statistics in verbose mode */
          if (verbose)
          {
            QBVH6* qbvh = (QBVH6*) accel.data();
            BVHStatistics stats = qbvh->computeStatistics();
            stats.print(std::cout);
            stats.print_raw(std::cout);
            //qbvh->print();
          }
          
          return bounds;
        }
        
      private:
        const getSizeFunc getSize;
        const getTypeFunc getType;
        const getNumTimeSegmentsFunc getNumTimeSegments;
        const createPrimRefArrayFunc createPrimRefArray;
        const getTriangleFunc getTriangle;
        const getTriangleIndicesFunc getTriangleIndices;
        const getQuadFunc getQuad;
        const getProceduralFunc getProcedural;
        const getInstanceFunc getInstance;
        Settings cfg;
        mvector<PrimRef> prims;
        FastAllocator allocator;
        std::vector<std::vector<uint16_t>> quadification;
        bool verbose;
      };
      
      template<typename getSizeFunc,
               typename getTypeFunc,
               typename getNumTimeSegmentsFunc,
               typename createPrimRefArrayFunc,
               typename getTriangleFunc,
               typename getTriangleIndicesFunc,
               typename getQuadFunc,
               typename getProceduralFunc,
               typename getInstanceFunc>
      
      static BBox3f build(size_t numGeometries,
                          Device* device,
                          const getSizeFunc& getSize,
                          const getTypeFunc& getType,
                          const getNumTimeSegmentsFunc& getNumTimeSegments,
                          const createPrimRefArrayFunc& createPrimRefArray,
                          const getTriangleFunc& getTriangle,
                          const getTriangleIndicesFunc& getTriangleIndices,
                          const getQuadFunc& getQuad,
                          const getProceduralFunc& getProcedural,
                          const getInstanceFunc& getInstance,
                          Device::avector<char,64>& accel,
                          bool verbose,
                          void* dispatchGlobalsPtr)
      {
        BuilderT<getSizeFunc, getTypeFunc, getNumTimeSegmentsFunc, createPrimRefArrayFunc, getTriangleFunc, getTriangleIndicesFunc, getQuadFunc, getProceduralFunc, getInstanceFunc> builder
          (device, getSize, getType, getNumTimeSegments, createPrimRefArray, getTriangle, getTriangleIndices, getQuad, getProcedural, getInstance, verbose);
        
        return builder.build(numGeometries, device, accel, dispatchGlobalsPtr);
      }      
    };
  }
}
