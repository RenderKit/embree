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

#if defined(USE_TBB)

#define PROFILE_MORTON_GENERAL

#include "bvh4.h"
#include "bvh4_builder_morton_general.h"

#include "geometry/triangle1.h"
#include "geometry/triangle4.h"
#include "geometry/triangle8.h"
#include "geometry/triangle1v.h"
#include "geometry/triangle4v.h"
#include "geometry/triangle4i.h"


#define DBG(x) 


namespace embree 
{
  namespace isa
  {
    static double dt = 0.0f;
    
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    void BVH4BuilderMortonGeneral::createSmallLeaf(BuildRecord& current, Allocator* alloc, BBox3fa& box_o)
    {
      ssef lower(pos_inf);
      ssef upper(neg_inf);
      size_t items = current.size();
      size_t start = current.begin;
      assert(items<=4);
      
      /* allocate leaf node */
      Triangle4* accel = (Triangle4*) alloc->alloc1.malloc(sizeof(Triangle4));
      *current.parent = bvh->encodeLeaf((char*)accel,listMode ? listMode : 1);
      
      ssei vgeomID = -1, vprimID = -1, vmask = -1;
      sse3f v0 = zero, v1 = zero, v2 = zero;
      
      for (size_t i=0; i<items; i++)
      {
        const size_t index = morton[start+i].index;
        const size_t primID = index & encodeMask; 
        const size_t geomID = this->mesh ? this->mesh->id : (index >> encodeShift); 
        const TriangleMesh* mesh = scene->getTriangleMesh(geomID);
        const TriangleMesh::Triangle& tri = mesh->triangle(primID);
        const Vec3fa& p0 = mesh->vertex(tri.v[0]);
        const Vec3fa& p1 = mesh->vertex(tri.v[1]);
        const Vec3fa& p2 = mesh->vertex(tri.v[2]);
        lower = min(lower,(ssef)p0,(ssef)p1,(ssef)p2);
        upper = max(upper,(ssef)p0,(ssef)p1,(ssef)p2);
        vgeomID [i] = geomID;
        vprimID [i] = primID;
        vmask   [i] = mesh->mask;
        v0.x[i] = p0.x; v0.y[i] = p0.y; v0.z[i] = p0.z;
        v1.x[i] = p1.x; v1.y[i] = p1.y; v1.z[i] = p1.z;
        v2.x[i] = p2.x; v2.y[i] = p2.y; v2.z[i] = p2.z;
      }
      Triangle4::store_nt(accel,Triangle4(v0,v1,v2,vgeomID,vprimID,vmask,listMode));
      box_o = BBox3fa((Vec3fa)lower,(Vec3fa)upper);
    }


    //template<typename CreateLeafFunc>
    class BVH4BuilderMortonGeneral2 : public Builder
    {
      static const size_t LATTICE_BITS_PER_DIM = 10;
      static const size_t LATTICE_SIZE_PER_DIM = size_t(1) << LATTICE_BITS_PER_DIM;

    public:
      BVH4* bvh;               //!< Output BVH
      //LockStepTaskScheduler* scheduler;
      //std::unique_ptr<MortonBuilderState> state;

      Scene* scene;
      TriangleMesh* mesh;
      size_t logBlockSize;
      size_t blocks(size_t N) { return (N+((1<<logBlockSize)-1)) >> logBlockSize; }
      bool needVertices;
      size_t primBytes; 
      size_t minLeafSize;
      size_t maxLeafSize;
      size_t listMode;

      size_t topLevelItemThreshold;
      size_t encodeShift;
      size_t encodeMask;
            
    public:
      MortonID32Bit* __restrict__ morton;
      size_t bytesMorton;
      
    public:
      size_t numGroups;
      size_t numPrimitives;
      size_t numAllocatedPrimitives;
      size_t numAllocatedNodes;
      CentGeomBBox3fa global_bounds;
      Barrier barrier;

      BVH4BuilderMortonGeneral2 (BVH4* bvh, Scene* scene, TriangleMesh* mesh, size_t listMode, size_t logBlockSize, bool needVertices, size_t primBytes, const size_t minLeafSize, const size_t maxLeafSize)
        : bvh(bvh), scene(scene), mesh(mesh), listMode(listMode), logBlockSize(logBlockSize), needVertices(needVertices), primBytes(primBytes), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize),
	topLevelItemThreshold(0), encodeShift(0), encodeMask(-1), morton(NULL), bytesMorton(0), numGroups(0), numPrimitives(0), numAllocatedPrimitives(0), numAllocatedNodes(0)
      {
        needAllThreads = true;
      }
      
      /*! Destruction */
      ~BVH4BuilderMortonGeneral2 ()
      {
        if (morton) os_free(morton,bytesMorton);
        bvh->alloc.shrink();
      }

       /* build function */
      void build(size_t threadIndex, size_t threadCount) 
      {
        if (g_verbose >= 2)
        std::cout << "building BVH4<" << bvh->primTy.name << "> with " << TOSTRING(isa) << "::BVH4BuilderMortonGeneral ... " << std::flush;
      
      /* calculate size of scene */
      size_t numPrimitivesOld = numPrimitives;
      if (mesh) numPrimitives = mesh->numTriangles;
      else      numPrimitives = scene->numTriangles;
      
      bvh->numPrimitives = numPrimitives;
      numGroups = scene->size();
      size_t maxPrimsPerGroup = 0;
      if (mesh) 
        maxPrimsPerGroup = numPrimitives;
      else {
	for (size_t i=0; i<numGroups; i++) // FIXME: encoding unsafe
        {
          Geometry* geom = scene->get(i);
          if (!geom || geom->type != TRIANGLE_MESH) continue;
          TriangleMesh* mesh = (TriangleMesh*) geom;
	  if (mesh->numTimeSteps != 1) continue;
	  maxPrimsPerGroup = max(maxPrimsPerGroup,mesh->numTriangles);
        }

        /* calculate groupID, primID encoding */
        encodeShift = __bsr(maxPrimsPerGroup) + 1;
        encodeMask = ((size_t)1 << encodeShift)-1;
        size_t maxGroups = ((size_t)1 << (31-encodeShift))-1;
      
        if (maxPrimsPerGroup > encodeMask || numGroups > maxGroups) 
          THROW_RUNTIME_ERROR("encoding error in morton builder");
      }
      
      /* preallocate arrays */
      if (numPrimitivesOld != numPrimitives)
      {
	bvh->init(sizeof(BVH4::Node),numPrimitives,threadCount);
        if (morton) os_free(morton,bytesMorton);
	bytesMorton = ((numPrimitives+7)&(-8)) * sizeof(MortonID32Bit);
        morton = (MortonID32Bit* ) os_malloc(bytesMorton); memset(morton,0,bytesMorton);
      }

       
         
#if defined(PROFILE_MORTON_GENERAL)
      
      double dt_min = pos_inf;
      double dt_avg = 0.0f;
      double dt_max = neg_inf;
      for (size_t k=0; k<20; k++) 
      {
        double t0 = getSeconds();
#endif

        //bvh->alloc.init(numPrimitives*sizeof(BVH4::Node),numPrimitives*sizeof(BVH4::Node));
      size_t bytesAllocated = (numPrimitives+7)/8*sizeof(BVH4::Node) + size_t(1.2f*(numPrimitives+3)/4)*sizeof(Triangle4);
      bvh->alloc2.init(bytesAllocated,2*bytesAllocated);

      /* compute scene bounds */
      Scene::Iterator<TriangleMesh,1> iter1(scene);
      global_bounds = parallel_for_for_reduce( iter1, CentGeomBBox3fa(empty), [&](TriangleMesh* mesh, const range<size_t>& r, size_t k) -> CentGeomBBox3fa
      {
        CentGeomBBox3fa bounds(empty);
        for (size_t i=r.begin(); i<r.end(); i++)
        {
          const BBox3fa b = mesh->bounds(i);
          if (!inFloatRange(b)) continue;
          bounds.extend(b);
        }
        return bounds;
      }, [] (CentGeomBBox3fa a, const CentGeomBBox3fa& b) { a.merge(b); return a; });

      bvh->bounds = global_bounds.geomBounds;

      /* compute morton codes */
      MortonID32Bit* __restrict__ const dest = (MortonID32Bit*) bvh->alloc2.ptr();

      /* compute mapping from world space into 3D grid */
      const ssef base  = (ssef)global_bounds.centBounds.lower;
      const ssef diag  = (ssef)global_bounds.centBounds.upper - (ssef)global_bounds.centBounds.lower;
      const ssef scale = select(diag > ssef(1E-19f), rcp(diag) * ssef(LATTICE_SIZE_PER_DIM * 0.99f),ssef(0.0f));
      
      Scene::Iterator<TriangleMesh,1> iter(scene);
      parallel_for_for( iter, [&](TriangleMesh* mesh, const range<size_t>& r, size_t k)
      {
        size_t currentID = k;
        size_t slots = 0;
        ssei ax = 0, ay = 0, az = 0, ai = 0;
                
        for (size_t i=r.begin(); i<r.end(); i++)	  
        {
          const BBox3fa b = mesh->bounds(i);
          const ssef lower = (ssef)b.lower;
          const ssef upper = (ssef)b.upper;
          const ssef centroid = lower+upper;
          const ssei binID = ssei((centroid-base)*scale);
          unsigned int index = i;
          index |= mesh->id << encodeShift;
          ax[slots] = extract<0>(binID);
          ay[slots] = extract<1>(binID);
          az[slots] = extract<2>(binID);
          ai[slots] = index;
          slots++;
          currentID++;
          
          if (slots == 4)
          {
            const ssei code = bitInterleave(ax,ay,az);
            storeu4i(&dest[currentID-4],unpacklo(code,ai));
            storeu4i(&dest[currentID-2],unpackhi(code,ai));
            slots = 0;
          }
        }
            
        if (slots != 0)
        {
          const ssei code = bitInterleave(ax,ay,az);
          for (size_t i=0; i<slots; i++) {
            dest[currentID-slots+i].index = ai[i];
            dest[currentID-slots+i].code = code[i];
          }
        }
      });

      /* padding */     
      for (size_t i=numPrimitives; i<( (numPrimitives+7)&(-8) ); i++) {
        dest[i].code  = 0xffffffff; 
        dest[i].index = 0;
      }

      BVH4BuilderMortonGeneral builder((BVH4*)bvh,scene,NULL,false,2,false,sizeof(Triangle4),4,inf);
      builder.build(threadIndex,threadCount,morton,numPrimitives,encodeShift,encodeMask);

      #if defined(PROFILE_MORTON_GENERAL)
        double dt = getSeconds()-t0;
        dt_min = min(dt_min,dt);
        if (k != 0) dt_avg = dt_avg + dt;
        dt_max = max(dt_max,dt);
      }
      dt_avg /= double(19);
      
      std::cout << "[DONE]" << std::endl;
      std::cout << "  min = " << 1000.0f*dt_min << "ms (" << numPrimitives/dt_min*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  avg = " << 1000.0f*dt_avg << "ms (" << numPrimitives/dt_avg*1E-6 << " Mtris/s)" << std::endl;
      std::cout << "  max = " << 1000.0f*dt_max << "ms (" << numPrimitives/dt_max*1E-6 << " Mtris/s)" << std::endl;
      std::cout << BVH4Statistics(bvh).str();
#endif

      if (g_verbose >= 2) {
        //std::cout << "[DONE] " << 1000.0f*dt << "ms (" << numPrimitives/dt*1E-6 << " Mtris/s)" << std::endl;
        std::cout << "  bvh4::alloc : "; bvh->alloc.print_statistics();
	std::cout << "  bvh4::alloc2: "; bvh->alloc2.print_statistics();
        std::cout << BVH4Statistics(bvh).str();
      }

      /* benchmark mode */
      if (g_benchmark) {
	BVH4Statistics stat(bvh);
	//std::cout << "BENCHMARK_BUILD " << dt << " " << double(numPrimitives)/dt << " " << stat.sah() << " " << stat.bytesUsed() << std::endl;
      }

      }
    };
    
    // =======================================================================================================
    // =======================================================================================================
    // =======================================================================================================
    
    

      Builder* BVH4Triangle4BuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { 
        return new class BVH4BuilderMortonGeneral2 ((BVH4*)bvh,scene,NULL,mode,2,false,sizeof(Triangle4),4,inf); 
      }
  }
}

#else

#include "common/builder.h"

namespace embree 
{
  namespace isa
  {
    Builder* BVH4Triangle4BuilderMortonGeneral  (void* bvh, Scene* scene, size_t mode) { 
      return NULL;
    }
  }
}
#endif
  
  
