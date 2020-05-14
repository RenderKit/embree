// Copyright 2009-2020 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "bvh.h"
#include "../common/primref.h"
#include "../builders/priminfo.h"
#include "../builders/primrefgen.h"

namespace embree
{
  namespace isa
  {
    template<int N, typename Mesh, typename Primitive>
    class BVHNBuilderTwoLevel : public Builder
    {
      typedef BVHN<N> BVH;
      typedef typename BVH::AlignedNode AlignedNode;
      typedef typename BVH::NodeRef NodeRef;

    public:

      typedef void (*createMeshAccelTy)(Scene* scene, unsigned int geomID, AccelData*& accel, Builder*& builder);

      struct BuildRef : public PrimRef
      {
      public:
        __forceinline BuildRef () {}

        __forceinline BuildRef (const BBox3fa& bounds, NodeRef node)
          : PrimRef(bounds,(size_t)node), node(node)
        {
          if (node.isLeaf())
            bounds_area = 0.0f;
          else
            bounds_area = area(this->bounds());
        }

        /* used by the open/merge bvh builder */
        __forceinline BuildRef (const BBox3fa& bounds, NodeRef node, const unsigned int geomID, const unsigned int numPrimitives)
          : PrimRef(bounds,geomID,numPrimitives), node(node)
        {
          /* important for relative buildref ordering */
          if (node.isLeaf())
            bounds_area = 0.0f;
          else
            bounds_area = area(this->bounds());
        }

        __forceinline size_t size() const {
          return primID();
        }

        friend bool operator< (const BuildRef& a, const BuildRef& b) {
          return a.bounds_area < b.bounds_area;
        }

        friend __forceinline embree_ostream operator<<(embree_ostream cout, const BuildRef& ref) {
          return cout << "{ lower = " << ref.lower << ", upper = " << ref.upper << ", center2 = " << ref.center2() << ", geomID = " << ref.geomID() << ", numPrimitives = " << ref.numPrimitives() << ", bounds_area = " << ref.bounds_area << " }";
        }

        __forceinline unsigned int numPrimitives() const { return primID(); }

      public:
        NodeRef node;
        float bounds_area;
      };


      __forceinline size_t openBuildRef(BuildRef &bref, BuildRef *const refs) {
        if (bref.node.isLeaf())
        {
          refs[0] = bref;
          return 1;
        }
        NodeRef ref = bref.node;
        unsigned int geomID   = bref.geomID();
        unsigned int numPrims = max((unsigned int)bref.numPrimitives() / N,(unsigned int)1);
        AlignedNode* node = ref.alignedNode();
        size_t n = 0;
        for (size_t i=0; i<N; i++) {
          if (node->child(i) == BVH::emptyNode) continue;
          refs[i] = BuildRef(node->bounds(i),node->child(i),geomID,numPrims);
          n++;
        }
        assert(n > 1);
        return n;        
      }
      
      /*! Constructor. */
      BVHNBuilderTwoLevel (BVH* bvh, Scene* scene, const createMeshAccelTy createMeshAcce, const size_t singleThreadThreshold = DEFAULT_SINGLE_THREAD_THRESHOLD);
      
      /*! Destructor */
      ~BVHNBuilderTwoLevel ();
      
      /*! builder entry point */
      void build();
      void deleteGeometry(size_t geomID);
      void clear();

      void open_sequential(const size_t extSize);

    public:
      
      struct BuilderState
      {
        BuilderState ()
        : builder(nullptr), quality(RTC_BUILD_QUALITY_LOW) {}

        BuilderState (const Ref<Builder>& builder, RTCBuildQuality quality)
        : builder(builder), quality(quality) {}
        
        void clear() {
          builder = nullptr;
          quality = RTC_BUILD_QUALITY_LOW;
        }
        
        Ref<Builder> builder;
        RTCBuildQuality quality;
      };
      
    private:

      class RefBuilderBase {
      public:
        virtual ~RefBuilderBase () {}
        virtual void clear () = 0;
        virtual void attachBuildRefs (BVHNBuilderTwoLevel* builder) = 0;
        virtual bool meshQualityChanged (RTCBuildQuality currQuality) = 0;
      };

      class RefBuilderSmall : public RefBuilderBase {
      public:

        RefBuilderSmall (size_t objectID) 
        :
          objectID_ (objectID)
        {}

        void clear () {}

        void attachBuildRefs (BVHNBuilderTwoLevel* topBuilder) {

          if (!topBuilder->scene->isGeometryModified(objectID_)) {
            return;
          }

          Mesh* mesh = topBuilder->scene->template getSafe<Mesh>(objectID_);
          size_t meshSize = mesh->size();
          assert(meshSize <= N);
          
          mvector<PrimRef> prims(topBuilder->scene->device, meshSize);
          auto pinfo = createPrimRefArray(mesh,objectID_,prims,topBuilder->bvh->scene->progressInterface);
          if (unlikely(pinfo.size() == 0)) {
            return;
          }

          Primitive* accel = (Primitive*) topBuilder->bvh->alloc.getCachedAllocator().malloc1(sizeof(Primitive),BVH::byteAlignment);
          typename BVH::NodeRef node = BVH::encodeLeaf((char*)accel,1);
          size_t begin (0);
          accel->fill(prims.data(),begin,pinfo.size(),topBuilder->bvh->scene);

          /* create build primitive */
#if ENABLE_DIRECT_SAH_MERGE_BUILDER
          topBuilder->refs[topBuilder->nextRef++] = BVHNBuilderTwoLevel::BuildRef(pinfo.geomBounds,node,(unsigned int)objectID_,(unsigned int)meshSize);
#else
          topBuilder->refs[topBuilder->nextRef++] = BVHNBuilderTwoLevel::BuildRef(pinfo.geomBounds,node);
#endif
        }

        bool meshQualityChanged (RTCBuildQuality /*currQuality*/) {
          return false;
        }

        private:
          size_t  objectID_;
      };

      class RefBuilderLarge : public RefBuilderBase {
      public:
        
        RefBuilderLarge (size_t objectID, BuilderState builder)
        :
          objectID_ (objectID)
        , builder_ (builder)
        {}

        void clear () {
          builder_.clear();
        }

        void attachBuildRefs (BVHNBuilderTwoLevel* topBuilder) {
          BVH*     object  = topBuilder->bvh->objects [objectID_]; assert(object);
          Ref<Builder>& builder = builder_.builder; assert(builder);

          /* build object if it got modified */
          if (topBuilder->scene->isGeometryModified(objectID_))
            builder->build();

          /* create build primitive */
          if (!object->getBounds().empty())
          {
#if ENABLE_DIRECT_SAH_MERGE_BUILDER
            topBuilder->refs[topBuilder->nextRef++] = BVHNBuilderTwoLevel::BuildRef(object->getBounds(),object->root,(unsigned int)objectID,(unsigned int)mesh->size());
#else
            topBuilder->refs[topBuilder->nextRef++] = BVHNBuilderTwoLevel::BuildRef(object->getBounds(),object->root);
#endif
          }
        }

        bool meshQualityChanged (RTCBuildQuality currQuality) {
          return currQuality != builder_.quality;
        }

      private:
        size_t       objectID_;
        BuilderState builder_;
      };

      void setupLargeBuildRefBuilder (size_t objectID, Mesh const * const mesh);
      void setupSmallBuildRefBuilder (size_t objectID, Mesh const * const mesh);

    public:
      BVH* bvh;
      std::vector<std::unique_ptr<RefBuilderBase>> builders_;
      
    public:
      Scene* scene;
      createMeshAccelTy createMeshAccel;
      
      mvector<BuildRef> refs;
      mvector<PrimRef> prims;
      std::atomic<int> nextRef;
      const size_t singleThreadThreshold;

      typedef mvector<BuildRef> bvector;

    };
  }
}
