// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "rthwif_embree_builder_stoch.h"

#include "rthwif_embree_builder.h"
#include "builder/qbvh6_builder_sah.h"
#include "rthwif_internal.h"
#include "builder/quadifier.h"

#include "../bvh/bvh.h"
#include "../bvh/bvh_builder.h"
#include "../builders/primrefgen.h"
#include "../builders/primrefgen_presplit.h"
#include "../builders/splitter.h"
//#include "../geometry/triangle1v.h"
//#include "../geometry/quad1v.h"

#include "../common/state.h"
#include "../../common/algorithms/parallel_for_for.h"
#include "../../common/algorithms/parallel_for_for_prefix_sum.h"

//#include "../bvh/gbvh_builder_gpu.h"
//#include "../bvh/gbvh_builder_gpu_morton.h"
#include "builder/gpu/morton.h"

#include "../common/lexers/streamfilters.h"
#include <stack>
#include  <unistd.h>
//#include "../bvh/sobol.h"
#include "builder/gpu/builder.h"
#include "builder/gpu/sort2.h"

#define CHECK2(x) if (!(x)) { throw std::runtime_error("check2 error: " #x); }
#if defined(EMBREE_SYCL_GPU_BVH_BUILDER)

#define NUM_TOPLEVEL_WG  64
#define SIZE_TOPLEVEL_WG 1024
#define CHECKS 0
#define CHECK(x) assert(x)

#define GPU_SUBSET 1
#define MORTON_SORT 1
#define STABLE_ORDER 1
#define COHERENT_SUBSET 1
#define DETERMINISTIC_CLUSTERS 1

#define USE_PHIST 0
#define PHIST_SIZE 256
#define PHIST_MODE 1

namespace embree
{
  using namespace embree::isa;

  typedef gpu::MortonCodePrimitive MCPrim;
    
  const float BIN_BASE = 1.41421356237309504880168872420969807856967187537694807317667973799;
  const int BIN_OFFSET = 64;
  const int BIN_COUNT = 128;
  const uint SUBTREE_THRESHOLD = 512;

  double pHistTimerStochHW = 0.f;

  void resetPhistStochHW(sycl::queue &gpu_queue, uint *phist) {
#if USE_PHIST
    gpu::waitOnQueueAndCatchException(gpu_queue);
#if PHIST_MODE == 0
    for (uint i = 0; i < PHIST_SIZE; i++)
      phist[i] = 0;
#else
    pHistTimerStochHW = getSeconds();
#endif
#endif 
  }

  void printresetPhistStochHW(const char* key, sycl::queue &gpu_queue, uint *phist) {
#if USE_PHIST
    gpu::waitOnQueueAndCatchException(gpu_queue);
#if PHIST_MODE == 0
    std::cout << "phist: " << key << "; ";
    for (uint i = 0; i < PHIST_SIZE; i++) {
      if (i != 0)
        std::cout << ", ";
      std::cout << phist[i];
      phist[i] = 0;
    }
    std::cout << std::endl;
#else
    double newpHistTimerStochHW = getSeconds();
    double diff = newpHistTimerStochHW - pHistTimerStochHW;
    std::cout << "phist: " << key << "; " << diff << std::endl;
    pHistTimerStochHW = newpHistTimerStochHW;
#endif
#endif 
  }

  void writePHistStochHW(uint *phist, gpu::BVH2BuildRecord *bvh_nodes, uint index) {
#if USE_PHIST && PHIST_MODE == 0
    uint numPrims = bvh_nodes[index].end - bvh_nodes[index].start;
    uint level = 0;

    for (; index != 0x7fffffff; level++)
      index = bvh_nodes[index].parent;
    
    level--;

    sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> ap(phist[level]);
    ap.fetch_add(numPrims);
#endif 
  }

  struct WeightClampingContextHW {
    uint bin_counts[BIN_COUNT];
    float clamp;
  };
  

  struct BuildParametersHW {
    // don't do stochastic construction if primitive count is below this
    int minSetSize = 10000;
    // fraction of primitives to use for stochastic construction
    float subsetSizeFrac = 0.2;
    // how much uniform weight to add to sampling of primitives
    float uniformCdfFrac = 0.1;
    // scale bounding box and cost of subset primitives according to cdf
    bool representativePrimitivesBBox = false;
    // this only applies to scaling of bounding box
    float representativePrimitivesBBoxPower = 0.5f;
    // scale bounding box of subset primitives according to cdf
    bool representativePrimitivesCost = true;
    // window of subset primtives and their clusters 
    // around the current primitive to consider for insertion
    int localMortonSearchWindow = 10;
    // separate parameters for subset and cluster BVH construction
    // terminology: base = cluster
    // TODO add indent?

    bool useSobol = false;
    uint randomSeed = 0;

    // deterministic clustering
    bool deterministicClustering = false;
    int clusteringMortonBits = 7;

    uint maxLeafSizeTopLevel = deterministicClustering ? 1 : 8;
    uint maxLeafSizeBottomLevel = 8;
    uint SAHLogBlockShift;

    void printParameters(int indent)
    {
      printf("Stochastic BVH build parameters:\n"
              "  minSetSize %d\n"
              "  subsetSizeFrac %0.2f\n"
              "  uniformCdfFrac %0.2f\n"
              "  samplingMethod %s\n"
              "  randomSeed %u\n"
              "  clustering %s\n"
              "  clusteringMortonBits %d\n"
              "  representativePrimitivesBBox %s\n"
              "  representativePrimitivesBBoxPower %0.2f\n"
              "  representativePrimitivesCost %s\n"
              "  localMortonSearchWindow %d\n"
              "endBuildParametersHW\n",
              minSetSize,
              subsetSizeFrac,
              uniformCdfFrac,
              // TODO use samplingMethod with a string
              useSobol ? "sobol" : "equidistant",
              randomSeed,
              deterministicClustering ? "deterministic" : "stochastic",
              clusteringMortonBits,
              representativePrimitivesBBox ? "true" : "false",
              representativePrimitivesBBoxPower,
              representativePrimitivesCost ? "true" : "false",
              localMortonSearchWindow
              );
    }
  };

  struct BuildContextHW {    
    uint size;
    uint targetSubsetSize;
    uint subsetSize;

    // Data needed across passes
    gpu::Globals *globals;
    gpu::AABB *aabb;
    char *bvh_mem;
    gpu::BVH2BuildRecord *bvh2;
    uint *primref_index; 
    uint *primref_index0;
    uint *primref_index1;
    uint *scratch_mem;
    MCPrim *mc0;
    MCPrim *morton_codes[2];

    // Top SAH build
    gpu::WorkGroupBuildState *wgBuildState;
    gpu::WorkGroupTaskState *wgTaskState;

    // Subset & Clustering
    WeightClampingContextHW *wc;
    
    // the cdf used for sampling. currently stored in full precision using emulated floats.
    // should be optimized to only use a single float (and extended precision at lower chunk granularity)
    // other options: double, two/multi-level or approximate (but correctly-rounded) cdfs instead of emulation
    float* cdf;
    
    // indicates whether a primitive is part of the subset
    uint* subsetSelection;
    
    // stores the selected cluster node for each subset primitive
    uint *subsetNodeSelections;
    
    // gives the index of the closest subset primitive for each primitive
    // used for the local morton search during insertion
    int *closestSubsetPrimitive;
    
    // stores the selected cluster node for each primitive
    uint *nodeSelections;

    // number of primitives per BVH2 node
    gpu::Range *leafPrimitiveRanges;

    uint *sobolMatrix;

    uint *phist;

    gpu::AABB* subsetAABB;
    
    uint*      clusterIDX;
    gpu::AABB* centroidAABB;
    
    void init(DeviceGPU* deviceGPU, size_t numPrims, float frac)
    //:bvhPrimitives(numPrims),
        //closestSubsetPrimitive(numPrims),
        //cdf(numPrims),
        //subsetSelection(numPrims)
        //nodeSelections(numPrims),
        //subsetNodeSelections(maxNumSubsetPrims)
        {
          size = numPrims;         
          targetSubsetSize = (int)std::round(numPrims * frac); // FIXME: adjust allocation due targetSubsetSize
          subsetSize = 0;
          wc                     = (WeightClampingContextHW*)sycl::aligned_alloc(64,sizeof(WeightClampingContextHW),deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
          cdf                    =                 (float*)sycl::aligned_alloc(64,sizeof(float)*numPrims,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
          subsetSelection        =                  (uint*)sycl::aligned_alloc(64,sizeof(uint)*numPrims,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
          closestSubsetPrimitive =                   (int*)sycl::aligned_alloc(64,sizeof(int)*numPrims,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
          subsetNodeSelections   =                  (uint*)sycl::aligned_alloc(64,sizeof(uint)*numPrims,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);          
          nodeSelections         =                  (uint*)sycl::aligned_alloc(64,sizeof(uint)*numPrims,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
          leafPrimitiveRanges    =            (gpu::Range*)sycl::aligned_alloc(64,sizeof(gpu::Range)*2*numPrims,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
          //sobolMatrix            =                  (uint*)sycl::aligned_alloc(64,sizeof(uint)*sobol::Matrices::num_dimensions*sobol::Matrices::size,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
          subsetAABB             =             (gpu::AABB*)sycl::aligned_alloc(64,sizeof(gpu::AABB)*numPrims,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
          clusterIDX             =                  (uint*)sycl::aligned_alloc(64,sizeof(uint)*numPrims,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
          centroidAABB           =             (gpu::AABB*)sycl::aligned_alloc(64,sizeof(gpu::AABB)*numPrims,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
#if USE_PHIST && PHIST_MODE == 0
          phist = (uint*)sycl::aligned_alloc(64,sizeof(uint)*PHIST_SIZE,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
          assert(phist);
#endif
          assert(wc);
          assert(cdf);
          assert(subsetSelection);
          assert(closestSubsetPrimitive);
          assert(subsetNodeSelections);
          assert(nodeSelections);
          assert(leafPrimitiveRanges);
          assert(subsetAABB);
        }
  };

  struct EventContextHW {
    union {double allBegin = 0; double preBegin;};
    sycl::event globals;
    sycl::event compCentroid;
    sycl::event compMortonCodes;
    double sortMortonBegin = 0;
    std::vector<std::pair<sycl::event, sycl::event>> sortMorton;
    double sortMortonEnd = 0;
    union {double preEnd = 0; double subsetBegin;};
    sycl::event initPrimRefIdx;
    sycl::event subsetInitBins;
    sycl::event subsetAccumulateBins;
    sycl::event subsetComputeClampingTerm;
    sycl::event subsetWriteClampedWeights;
    sycl::event subsetPSumLocal;
    sycl::event subsetPSumGlobal;
    sycl::event subsetInitSubsetSelection; // TODO still used in subset2(...) fun
    sycl::event subsetSampleSubset;
    sycl::event subsetWriteClosestSubsetPrimitive;
    sycl::event subsetMortonWindowPSumLocal;
    sycl::event subsetMortonWindowPSumGlobal;
    sycl::event subsetWriteSamples;
    union {double subsetEnd = 0; double topBegin; double topInitFirstBuildRecordBegin;};
    sycl::event topInitFirstBuildRecord;
    union {double topInitFirstBuildRecordEnd = 0; double topBreadthFirstBegin;};
    std::vector<std::tuple<sycl::event, sycl::event, sycl::event, sycl::event>> topBreadthFirst;
    union {double topBreadthFirstEnd = 0; double topMiddleBegin;};
    std::vector<sycl::event> topMiddle;
    union {double topMiddleEnd = 0; double topBottomBegin;};
    sycl::event topBottom;
    union {double topBottomEnd = 0; double topEnd; double clusterBegin;};
    sycl::event clusterInitNPrimitives;
    sycl::event clusterTopRefit;
    sycl::event clusterInitNodeSelection;
    sycl::event clusterWriteSubsetPrimSelection;
    sycl::event clusterWriteClosestSubsetPrimitive;
    sycl::event clusterMortonWindowPSumLocal;
    sycl::event clusterMortonWindowPSumGlobal;
    sycl::event clusterWriteSubsetNodeSelection;
    sycl::event clusterFindBestNode;    
    sycl::event clusterWritePrimref;
    sycl::event clusterReorderPSumLocal;
    sycl::event clusterReorderPSumGlobal;
    sycl::event clusterWriteLeafPrimitiveRanges;
    sycl::event clusterReorder;
    sycl::event clusterUpdateCentroidBounds;
    union {double clusterEnd = 0; double bottomBegin;};
    sycl::event bottom;
    union {double bottomEnd = 0; double refitBegin;};
    sycl::event refit;
    union {double refitEnd = 0; double allEnd;};

    void print(sycl::queue &gpu_queue) {
      struct StackEntry {
        double time = 0;
        std::stringstream out;
      };
      std::stack<StackEntry> stack;

      auto pK = [&](const std::string& key, const sycl::event &queue_event, double hostTime = -1) {
        auto &top = stack.top();
        const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
        const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
        double time = (t1 - t0) * 1E-6;
        top.time += time;
        for (int i = 0; i < stack.size(); i++)
          top.out << "  ";
        top.out << key << ": " << time;
        if (hostTime >= 0)
          top.out << " (" << hostTime * 1000 << ")";
        top.out << std::endl;
      };

      auto push = [&]() {
        stack.emplace();
      };

      auto pop = [&](const std::string& key, double hostTime = -1) {
        auto old = std::move(stack.top());
        stack.pop();
        auto &top = stack.top();
        top.time += old.time;
        for (int i = 0; i < stack.size(); i++)
          top.out << "  ";
        top.out << key << ": " << old.time;
        if (hostTime >= 0)
          top.out << " (" << hostTime * 1000 << ")";
        top.out << std::endl << old.out.str();
      };

      push();

      push();
      push();
      pK("globals", globals);
      pK("compCentroid", compCentroid);
#if MORTON_SORT
      pK("compMortonCodes", compMortonCodes);
      push();
      for (int i = 0; i < sortMorton.size(); i++) {
        pK("bin " + std::to_string(i), sortMorton[i].first);
        pK("scatter " + std::to_string(i), sortMorton[i].second);
      }
      pop("sortMorton", sortMortonEnd - sortMortonBegin);
#endif
      pop("pre", preEnd - preBegin);
      push();
      pK("InitBins", subsetInitBins);
      pK("AccumulateBins", subsetAccumulateBins);
      pK("ComputeClampingTerm", subsetComputeClampingTerm);
      pK("WriteClampedWeights", subsetWriteClampedWeights);
      pK("PSumLocal", subsetPSumLocal);
      pK("PSumGlobal", subsetPSumGlobal);
      pK("SampleSubset", subsetSampleSubset);
#if STABLE_ORDER
      pK("WriteClosestSubsetPrimitive", subsetWriteClosestSubsetPrimitive);
      pK("MortonWindowPSumLocal", subsetMortonWindowPSumLocal);
      pK("MortonWindowPSumGlobal", subsetMortonWindowPSumGlobal);
      pK("WriteSamples", subsetWriteSamples);
#endif
      pop("subset", subsetEnd - subsetBegin);
      push();
      pK("InitFirstBuildRecord", topInitFirstBuildRecord, topInitFirstBuildRecordEnd - topInitFirstBuildRecordBegin);
#if 1
      push();
      for (int i = 0; i < topBreadthFirst.size(); i++) {
        push();
        pK("schedule", std::get<0>(topBreadthFirst[i]));
        pK("init", std::get<1>(topBreadthFirst[i]));
        pK("bin", std::get<2>(topBreadthFirst[i]));
        pK("partition", std::get<3>(topBreadthFirst[i]));
        pop("it " + std::to_string(i));
      }
      pop("topBreadthFirst", topBreadthFirstEnd - topBreadthFirstBegin);
      push();
#endif
      for (int i = 0; i < topMiddle.size(); i++) {
        pK("it " + std::to_string(i), topMiddle[i]);
      }
      pop("topMiddle", topMiddleEnd - topMiddleBegin);
      pK("topBottom", topBottom, topBottomEnd - topBottomBegin);
      pop("top", topEnd - topBegin);
      push();
      pK("InitNPrimitives", clusterInitNPrimitives);
      pK("TopRefit", clusterTopRefit);
      pK("InitNodeSelection", clusterInitNodeSelection);
      pK("WriteSubsetPrimSelection", clusterWriteSubsetPrimSelection);
#if MORTON_SORT
#if !STABLE_ORDER
      pK("WriteClosestSubsetPrimitive", clusterWriteClosestSubsetPrimitive);
      pK("MortonWindowPSumLocal", clusterMortonWindowPSumLocal);
      pK("MortonWindowPSumGlobal", clusterMortonWindowPSumGlobal);
#endif
#if !COHERENT_SUBSET
      pK("WriteSubsetNodeSelection", clusterWriteSubsetNodeSelection);
#endif
#endif
      pK("FindBestNode", clusterFindBestNode);
      pK("WritePrimref", clusterWritePrimref);
      pK("ReorderPSumLocal", clusterReorderPSumLocal);
      pK("ReorderPSumGlobal", clusterReorderPSumGlobal);
      pK("WriteLeafPrimitiveRanges", clusterWriteLeafPrimitiveRanges);
      pK("Reorder", clusterReorder);
      pK("UpdateCentroidBounds", clusterUpdateCentroidBounds);
      pop("cluster", clusterEnd - clusterBegin);
      pK("bottom", bottom, bottomEnd - bottomBegin);
      pK("refit", refit, refitEnd - refitBegin);
      pop("all", allEnd - allBegin);

      std::cout << "Timings:\n" << stack.top().out.str() << "endTimings\n";
    }
  };

  void SubsetHW(sycl::queue &gpu_queue, BuildParametersHW &params, BuildContextHW &ctx, EventContextHW &ectx) {
    auto size = ctx.size;
    auto targetSubsetSize = ctx.targetSubsetSize;
    auto *cdf = ctx.cdf;

    ectx.subsetBegin = getSeconds();


// ******************************************************
//  CLAMPING BINS ZERO-INIT
//  TODO can't we simply set them to zero when gpu-copy?
// ******************************************************
#if GPU_SUBSET == 1
    {
      // BIN_COUNT must be power of 2
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   const sycl::nd_range<1> nd_range1(BIN_COUNT,sycl::range<1>(BIN_COUNT));                      
                                                   cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                       const uint i     = item.get_global_id(0);
                                                       ctx.wc->bin_counts[i] = 0;
                                                     });		  
                                                 });
      ectx.subsetInitBins = queue_event;
    }         
#else    
    gpu::waitOnQueueAndCatchException(gpu_queue);          
    // initialize bins
    // execute in one work group?
    for (int i = 0; i < BIN_COUNT; i++)
      ctx.wc->bin_counts[i] = 0;
#endif    

// ********************************************************************
//  CLAMPING BINS INIT WITH PRIMS WEIGTH AND MORTON INDEX INIT
//   - count histogram over weight distribution given BIN_BASE (sqrt2)
//   - init lower 32 bits for morton ordering with prim id
//     or simple array position if not sorting
// ********************************************************************
#if GPU_SUBSET == 1
    {
      const uint wgSize = 1024;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   sycl::local_accessor< WeightClampingContextHW, 0> _lwc(cgh);
                                                   const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(size,wgSize)),sycl::range<1>(wgSize));                                                        
                                                   cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {

                                                       const uint i     = item.get_global_id(0);
                                                       const uint li    = item.get_local_id(0);

                                                       WeightClampingContextHW &lwc = *_lwc.get_pointer();
                                                       if (li < BIN_COUNT)
                                                        lwc.bin_counts[li] = 0;

                                                       item.barrier(sycl::access::fence_space::local_space);

                                                       if (i < ctx.size)
                                                       {
#if MORTON_SORT
                                                         float w = ctx.aabb[ctx.mc0[i].getIndex()].length();
#else                                                         
                                                         float w = ctx.aabb[i].length();
#endif
                                                         ctx.cdf[i] = w;

                                                         int bin;
                                                         if (w == 0)
                                                           bin = 0;
                                                         else if (w == std::numeric_limits<float>::infinity())
                                                           bin = BIN_COUNT - 1;
                                                         else {
                                                           int exp = std::floor(std::log(w)/std::log(BIN_BASE));
                                                           bin = std::min(std::max(BIN_OFFSET + exp, 0), BIN_COUNT - 1);
                                                         }
                                                        gpu::atomic_add_local<uint>(&lwc.bin_counts[bin],1);
                                                       }

                                                       item.barrier(sycl::access::fence_space::local_space);

                                                       if (li < BIN_COUNT)
                                                       {
                                                         auto bin_count = lwc.bin_counts[li];
                                                         if (bin_count > 0)
                                                          gpu::atomic_add_global<uint>(&ctx.wc->bin_counts[li],bin_count);
                                                       }
                                                     });		  
                                                 });
      ectx.subsetAccumulateBins = queue_event;
    }
#else    
    gpu::waitOnQueueAndCatchException(gpu_queue);
    // compute weight for each primitive and add to corresponding bin
    // parallel for each primitive
    for (int i = 0; i < ctx.size; i++) {
      float w = aabb[i].length();
      ctx.cdf[i] = w;

      int bin;
      if (w == 0)
        bin = 0;
      else if (w == std::numeric_limits<float>::infinity())
        bin = BIN_COUNT - 1;
      else {
        int exp = std::floor(std::log(w)/std::log(BIN_BASE));
        bin = std::min(std::max(BIN_OFFSET + exp, 0), BIN_COUNT - 1);
      }

      ctx.wc->bin_counts[bin] += 1;
    }
#endif

// *******************************************************************
//  SINGLE THREADED BINNED CLAMPING TERM EVALUATION
//  goes over a handful of bins (32 to 64) so it does not matter much
// *******************************************************************
#if GPU_SUBSET == 1
    {
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
                                                    cgh.single_task([=]() {
                                                                      float prefix_sum = 0.f;
                                                                      int postfix_count = ctx.size;
                                                                      float clamp = 0;

                                                                      float weight_factor = 1.f / (1.f - params.uniformCdfFrac);
                                                                      
                                                                      int i = 0;
                                                                      for (; i < BIN_COUNT - 1; i++) {
                                                                        float bin_upper = std::pow(BIN_BASE, i - BIN_OFFSET + 1.f);
                                                                        uint bin_count = ctx.wc->bin_counts[i];
                                                                        prefix_sum += bin_upper * bin_count;
                                                                        postfix_count -= bin_count;
                                                                        clamp = bin_upper;
                                                                        float max_weight = weight_factor * (prefix_sum + clamp * postfix_count) / ctx.targetSubsetSize;;
                                                                        if (clamp >= max_weight) break;
                                                                      }

                                                                      if (i == BIN_COUNT - 1) clamp = std::numeric_limits<float>::infinity();

                                                                      ctx.wc->clamp = clamp;
                                                                    });
                                                  });
      ectx.subsetComputeClampingTerm = queue_event;
    }
#else    
    gpu::waitOnQueueAndCatchException(gpu_queue);
    // compute approximate clamping term
    // execute in one work group or even with just one thread?
    {
      float M = ctx.size * params.subsetSizeFrac;
      float prefix_sum = 0.f;
      int postfix_count = ctx.size;
      float clamp = 0;

      float weight_factor = 1.f / (1.f - params.uniformCdfFrac);
      
      int i = 0;
      for (; i < BIN_COUNT - 1; i++) {
        float bin_upper = std::pow(BIN_BASE, i - BIN_OFFSET + 1.f);
        uint bin_count = ctx.wc->bin_counts[i];
        prefix_sum += bin_upper * bin_count;
        postfix_count -= bin_count;
        clamp = bin_upper;
        float max_weight = weight_factor * (prefix_sum + clamp * postfix_count) / M;
        if (clamp >= max_weight) break;
      }

      if (i == BIN_COUNT - 1) clamp = std::numeric_limits<float>::infinity();

      ctx.wc->clamp = clamp;
    }
#endif

// ************************************************
//  WEIGHT CLAMPING AND SUBSET SELECTION ZERO-INIT
// ************************************************
#if GPU_SUBSET == 1
    {
      // FIXME: this doesn't work yet as prefix_sum algorithms required integral data types like float,int,...
      // TODO: Ideally we could do the prefix sum and select the primitives in the same pass
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(size,wgSize)),sycl::range<1>(wgSize));                                                        
                                                   cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                       const uint i     = item.get_global_id(0);
                                                       if (i < ctx.size)
                                                         // clamp the prim weight
                                                         ctx.cdf[i] = std::min(ctx.cdf[i], ctx.wc->clamp);
                                                         // init the selection of this prim to zero
                                                         ctx.subsetSelection[i] = 0;
                                                     });		  
                                                 });
      ectx.subsetWriteClampedWeights = queue_event;
      
      auto events = inclusive_prefix_sum<float>(gpu_queue,cdf,size,(float*)ctx.scratch_mem, false);
      ectx.subsetPSumLocal = events.first;
      ectx.subsetPSumGlobal = events.second;
    }
#else    
    gpu::waitOnQueueAndCatchException(gpu_queue);      
    // inclusive prefix sum over interpolation between clamped and uniform weights
    // kahan summation for reference
    {
      float pfix = 0.f, c = 0.f;
      for (int i = 0; i < ctx.size; i++) {
        float w = std::min(ctx.cdf[i], ctx.wc->clamp);
        float y = w - c;
        float t = pfix + y;
        c = (t - pfix) - y;
        pfix = t;
        ctx.cdf[i] = pfix;
      }
    }
    for (uint i=0;i<ctx.size;i++)
      ctx.subsetSelection[i] = 0;   
#endif



// #if GPU_SUBSET == 1
//     {
//       const uint wgSize = 256;
//       sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
//                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(ctx.size,wgSize)),sycl::range<1>(wgSize));                                                        
//                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
//                                                        const uint i     = item.get_global_id(0);
//                                                        if (i < ctx.size)
//                                                          ctx.subsetSelection[i] = 0;      
//                                                      });		  
//                                                  });
//       ectx.subsetInitSubsetSelection = queue_event;
//     }
// #else    
//     gpu::waitOnQueueAndCatchException(gpu_queue);      
//     for (uint i=0;i<ctx.size;i++)
//       ctx.subsetSelection[i] = 0;      
// #endif

#if GPU_SUBSET == 1
    {
      const uint wgSize = 32;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                   const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(targetSubsetSize,wgSize)),sycl::range<1>(wgSize));                                                        
                                                   cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                       /*
                                                       auto sampleSobol = [&](unsigned long long index, const unsigned dimension, const unsigned scramble = 0U) {
                                                        assert(dimension < sobol::Matrices::num_dimensions);

                                                        unsigned result = scramble;
                                                        for (unsigned i = dimension * sobol::Matrices::size; index; index >>= 1, ++i)
                                                        {
                                                            if (index & 1)
                                                                result ^= ctx.sobolMatrix[i];
                                                        }

                                                        return result * (1.f / (1ULL << 32));
                                                       };
                                                       */


                                                       const uint i     = item.get_global_id(0);
                                                       const uint subgroupID = get_sub_group_id();
                                                       if (i < ctx.targetSubsetSize)
                                                       {
                                                         float rnd;
                                                         //if (params.useSobol) {
                                                         //  rnd = sampleSobol(i, params.randomSeed);
                                                         //} else {
                                                           float offset = params.randomSeed * (1.f / (1ULL << 32));
                                                           rnd = ((float(i) + offset) / ctx.targetSubsetSize);
                                                         //}
                                                         float lf = (1 - params.uniformCdfFrac) / ctx.cdf[ctx.size-1];
                                                         float rf = params.uniformCdfFrac / ctx.size;

                                                         auto cdf = [&](int i) {
                                                            return lf * ctx.cdf[i] + rf * (i + 1.f);
                                                         };

                                                         int j;
                                                         {
                                                           int first = 0;
                                                           int step, i;
                                                           int count = ctx.size - 1;
                                                          
                                                           while (count > 0) {
                                                             i = first;
                                                             step = count / 2; 
                                                             i += step;
                                                             if (!(rnd < cdf(i))) {
                                                               first = ++i;
                                                               count -= step + 1;
                                                             } 
                                                             else
                                                              count = step;
                                                           }
                                                           j = first;
                                                         }       

                                                         if (gpu::atomic_max_global<uint>(&ctx.subsetSelection[j],1) == 0)
                                                         {
#if 1
                                                           uint subsetSizeID = gpu::atomic_add_global<uint>(&ctx.globals->sched,1);
#else
                                                           uint offset = sub_group_exclusive_scan(1u, std::plus<uint>());
                                                           uint maxID = sub_group_reduce_max(subgroupID);
                                                           uint globalOffset = 0;
                                                           if (subgroupID == maxID)
                                                            globalOffset = gpu::atomic_add_global<uint>(&ctx.globals->sched,offset + 1);
                                                           uint subsetSizeID = sub_group_broadcast(globalOffset, maxID) + offset;
#endif
#if !STABLE_ORDER
#if MORTON_SORT
                                                           ctx.primref_index0[subsetSizeID] = ctx.mc0[j].getIndex();
#else
                                                           ctx.primref_index0[subsetSizeID] = j;
#endif                                                           
#endif
                                                         }
                                                         
                                                       }
                                                     });		  
                                                 });
      ectx.subsetSampleSubset = queue_event;
    }
#else    
    gpu::waitOnQueueAndCatchException(gpu_queue);      
    // sample subset primitives
    // parallel for 0 to subsetSize
    {
      for (int i = 0; i <  ctx.targetSubsetSize; i++) {
        float rnd = (float(i) / ctx.targetSubsetSize);
        float lf = (1 - params.uniformCdfFrac) / ctx.cdf[ctx.size-1];
        float rf = params.uniformCdfFrac / ctx.size;

        auto cdf = [&](int i) {
          return lf * ctx.cdf[i] + rf * (i + 1.f);
        };

        int j;
        {
          int first = 0;
          int step, i;
          int count = ctx.size - 1;
        
          while (count > 0) {
              i = first;
              step = count / 2; 
              i += step;
              if (!(rnd < cdf(i))) {
                  first = ++i;
                  count -= step + 1;
              } 
              else
                  count = step;
          }
          j = first;
        }

        /*
        int upper_index = ctx.size-1;
        for (int j=0;j<ctx.size-1;j++)
        {
          if (ctx.cdf[j] > rnd * ctx.wc->sum) {
            upper_index = j;
            break;
          }
        }        

        if (upper_index_ != upper_index) abort();
        */
        //auto it = std::upper_bound(ctx.cdf.begin(), ctx.cdf.end() - 1, rnd * ctx.wc->sum);

        //auto j = it - ctx.cdf.begin();
        
        if (ctx.subsetSelection[j] == 1) continue;
        ctx.subsetSelection[j] = 1;

        float pdf = (cdf(j) - (0 < j ? cdf(j - 1) : 0));
        CHECK(pdf > 0.f);
        auto scale = std::max(1.f / (float)(pdf *  ctx.targetSubsetSize), 1.f);

        auto prim = ctx.bvhPrimitives[j];
        prim.primitiveNumber = j;
        if (params.representativePrimitivesCost)
          prim.cost = scale;
        ctx.bvhPrimitivesScratch[ctx.subsetSize++] = prim;
      }
    }
#endif
#if STABLE_ORDER
    {
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //11
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(ctx.size,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint i = item.get_global_id(0);
                                                        if (i < ctx.size)
                                                          ctx.closestSubsetPrimitive[i] = ctx.subsetSelection[i];
                                                      });		  
                                                  });
      ectx.subsetWriteClosestSubsetPrimitive = queue_event;
    }        
    
    auto events = exclusive_prefix_sum2<uint>(gpu_queue,(uint*)ctx.closestSubsetPrimitive,ctx.size,(uint*)ctx.scratch_mem);
    ectx.subsetMortonWindowPSumLocal = events.first;
    ectx.subsetMortonWindowPSumGlobal = events.second;

    { // TODO make as bisection
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //11
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(ctx.size,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint i = item.get_global_id(0);
                                                        if (i < ctx.size && ctx.subsetSelection[i] != 0) {
                                                          uint j = ctx.closestSubsetPrimitive[i];
#if COHERENT_SUBSET
#if MORTON_SORT
                                                          uint index = ctx.mc0[i].getIndex();
#else
                                                          uint index = i; 
#endif
                                                          ctx.subsetAABB[j] = ctx.aabb[index];
                                                          ctx.primref_index0[j] = j;
#else
#if MORTON_SORT
                                                          ctx.primref_index0[j] = ctx.mc0[i].getIndex();
#else
                                                          ctx.primref_index0[j] = i;
#endif
#endif
                                                        }
                                                      });		  
                                                  });
      ectx.subsetWriteSamples = queue_event;
    }       
#endif
  }

  gpu::AABB3f checkBVH2Stoch2HW(gpu::BVH2BuildRecord *bvh_nodes, uint index,const uint *const primref_index,gpu::AABB *aabb,uint &nodes,uint &leaves,double &nodeSAH, double &leafSAH, const uint cfg_maxLeafSize);

  void BuildTopSAHHW(sycl::queue &gpu_queue, BuildParametersHW &params, BuildContextHW &ctx, EventContextHW &ectx) {
    uint numPrimitives = ctx.subsetSize;

    // === Build top level BVH over subset ===
    
    // === init first build record ===
    {	  
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) { //5
                                                    cgh.single_task([=]() {
                                                                      ctx.bvh2[0].init(0,ctx.subsetSize,0x7fffffff,convert_AABB3f(ctx.globals->centroidBounds));
                                                                    });
                                                  });
      ectx.topInitFirstBuildRecord = queue_event;
    }

    gpu::waitOnQueueAndCatchException(gpu_queue);   
    ectx.topInitFirstBuildRecordEnd = getSeconds();

    resetPhistStochHW(gpu_queue, ctx.phist);

    const uint cfg_maxLeafSize = params.maxLeafSizeTopLevel;
    PRINT(cfg_maxLeafSize);

    if (numPrimitives > NUM_TOPLEVEL_WG) {

#if 1
    // todo: range <= cfg_maxLeafSize

    //SORT!!!!
    //GLOBAL infinity check with merge
    for (uint j=0;j<7;j++)
    {
      ectx.topBreadthFirst.push_back({});
      auto topBreadthFirst = ectx.topBreadthFirst.rbegin();
      {
        sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) {
          cgh.single_task([=]() {
            ctx.globals->numWGs = 0;        
  
            /* init task states */
          
            //const uint numRecords = ctx.globals->bvh2_breathfirst_records_end - ctx.globals->bvh2_breathfirst_records_start;
            const uint numRecords = ctx.globals->bvh2_index_allocator;
            //PRINT(numRecords);
            if (numRecords == 1)
            {
              writePHistStochHW(ctx.phist, ctx.bvh2, 0);
              for (uint i=0;i<NUM_TOPLEVEL_WG;i++)
              {
                gpu::BVH2BuildRecord &current = ctx.bvh2[0];              
                ctx.wgTaskState[i].recordID = 0;
                ctx.wgTaskState[i].buildStateID = 0;
                ctx.wgTaskState[i].start = current.start + (i + 0)*current.size() / NUM_TOPLEVEL_WG;
                ctx.wgTaskState[i].end   = current.start + (i + 1)*current.size() / NUM_TOPLEVEL_WG;
                //PRINT5(i,ctx.wgTaskState[i].recordID,ctx.wgTaskState[i].buildStateID,ctx.wgTaskState[i].start,ctx.wgTaskState[i].end);                
              }
              ctx.wgBuildState[0].numAssignedWGs = NUM_TOPLEVEL_WG;
              ctx.wgBuildState[0].numDoneWGs = 0;
              //PRINT2(0,ctx.wgBuildState[0].numAssignedWGs);
              ctx.globals->numWGs = NUM_TOPLEVEL_WG;
            }
            else
            {
              //PRINT(average_threshold);
              uint recs = 0;
              uint average_threshold = 0;
              for (uint r=0;r<numRecords;r++)
                if (ctx.bvh2[r].left == -1)
                {
                  average_threshold += ctx.bvh2[r].size();
                  recs++;
                }
              average_threshold = (uint)floorf((float)average_threshold / recs);
              //PRINT(average_threshold);
              //const uint average_threshold = (uint)floorf(numPrimitives / (2*NUM_TOPLEVEL_WG));
              
              uint recordIDs = 0;
              uint recordIDs_total_size = 0;
                
              uint record_table_IDs[NUM_TOPLEVEL_WG];
              for (uint r=0;r<numRecords;r++)
                if (ctx.bvh2[r].size() >= average_threshold && ctx.bvh2[r].left == -1)
                {
                  writePHistStochHW(ctx.phist, ctx.bvh2, r);
                  recordIDs_total_size += ctx.bvh2[r].size();
                  record_table_IDs[recordIDs++] = r;
                }
              
              // for (uint r=0;r<recordIDs;r++)
              //   PRINT2(r,record_table_IDs[r]);
              //
              //const uint average_threshold = (uint)floorf(recordIDs_total_size / NUM_TOPLEVEL_WG);
              
              uint numWGs_per_record_table[NUM_TOPLEVEL_WG];
              for (uint r=0;r<recordIDs;r++)
              {
                const uint recordID = record_table_IDs[r];
                numWGs_per_record_table[r] = max((uint)floorf(NUM_TOPLEVEL_WG * (float)ctx.bvh2[recordID].size() / recordIDs_total_size),(uint)1);
                assert(numWGs_per_record_table[r]);
                ctx.globals->numWGs += numWGs_per_record_table[r];
                //PRINT3(r,numWGs_per_record_table[r],numWGs_launch);
              }
              //PRINT(numWGs_launch);
              assert(ctx.globals->numWGs <= NUM_TOPLEVEL_WG);
              
              uint wgTaskStateID = 0;
              for (uint r=0;r<recordIDs;r++)
              {
                //PRINT(r);
                for (uint i=0;i<numWGs_per_record_table[r];i++)
                {
                  const uint recordID = record_table_IDs[r];
                  gpu::BVH2BuildRecord &current = ctx.bvh2[recordID];
                    
                  ctx.wgTaskState[wgTaskStateID+i].recordID = recordID;
                  ctx.wgTaskState[wgTaskStateID+i].buildStateID = r;
                  ctx.wgTaskState[wgTaskStateID+i].start = current.start + (i + 0)*current.size() / numWGs_per_record_table[r];
                  ctx.wgTaskState[wgTaskStateID+i].end   = current.start + (i + 1)*current.size() / numWGs_per_record_table[r];
                  
                  //PRINT6(i,wgTaskStateID+i,ctx.wgTaskState[wgTaskStateID+i].recordID,ctx.wgTaskState[wgTaskStateID+i].buildStateID,ctx.wgTaskState[wgTaskStateID+i].start,ctx.wgTaskState[wgTaskStateID+i].end);
                }
                wgTaskStateID += numWGs_per_record_table[r];

                ctx.wgBuildState[r].numAssignedWGs = numWGs_per_record_table[r];
                ctx.wgBuildState[r].numDoneWGs = 0;
                //PRINT2(r,ctx.wgBuildState[r].numAssignedWGs);                
              }
              //exit(0);              
            }
          });
        });
        std::get<0>(*topBreadthFirst) = queue_event;
      }

      {
        assert(numPrimitives > cfg_maxLeafSize);
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(NUM_TOPLEVEL_WG*BVH_BINNING_BINS),sycl::range<1>(BVH_BINNING_BINS));
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(BVH_BINNING_BINS) {
                                                        const uint groupID      = item.get_group(0);
                                                        if (!(groupID < ctx.globals->numWGs)) return;
                                                        ctx.wgBuildState[groupID].init();
                                                      });		  
                                                  });
        std::get<1>(*topBreadthFirst) = queue_event;
      }

      {
        assert(numPrimitives > cfg_maxLeafSize);
#define NUM_BININFO 2
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(NUM_TOPLEVEL_WG*SIZE_TOPLEVEL_WG),sycl::range<1>(SIZE_TOPLEVEL_WG));
    
                                                    /* local variables */
                                                    sycl::local_accessor< gpu::BinInfoNew   , 1> _binInfo(sycl::range<1>(NUM_BININFO),cgh);
                                                                                    
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint localID      = item.get_local_id(0);                                                         
                                                        const uint groupID      = item.get_group(0);
                                                        //const uint numGroups    = item.get_group_range(0);
                                                        const uint subgroupID   = get_sub_group_id();
                                                        const uint localSize    = item.get_local_range().size();

                                                        if (!(groupID < ctx.globals->numWGs)) return;

                                                        gpu::BinInfoNew *binInfo    = _binInfo.get_pointer();
#if STABLE_ORDER && COHERENT_SUBSET                                                                                                                  
                                                        const gpu::AABB *const primref  = ctx.subsetAABB;
#else
                                                        const gpu::AABB *const primref  = ctx.aabb;
#endif

                                                        const gpu::WorkGroupTaskState &taskState = ctx.wgTaskState[groupID];
                                                        uint recordID = taskState.recordID;
                                                        gpu::BVH2BuildRecord &current = ctx.bvh2[recordID];
                                                        
                                                        gpu::BinMapping binMapping;    
                                                        binMapping.init(gpu::convert_AABB(current.bounds),BVH_BINNING_BINS);    

                                                        gpu::BinInfoNew *bi = &binInfo[subgroupID % NUM_BININFO];
                                                        
                                                        /* init bininfo */
                                                        if (subgroupID < NUM_BININFO)
                                                          bi->init();
                                                        
                                                        item.barrier(sycl::access::fence_space::local_space);

                                                        const uint startID      = taskState.start;
                                                        const uint endID        = taskState.end;
                                                        const uint buildStateID = taskState.buildStateID;
                                                        
                                                        /* bin primrefs */
                                                        for (uint t=startID + localID;t<endID;t+=localSize)
                                                        {
                                                          const uint index = ctx.primref_index0[t];
                                                          ctx.primref_index1[t] = index;
                                                          bi->atomicUpdateBallot(binMapping,primref[index]);
                                                          //bi->atomicUpdate(binMapping,primref[index]);                                                             
                                                        }
                                                        
                                                        item.barrier(sycl::access::fence_space::local_space);

                                                        if (subgroupID == 0)
                                                        {
                                                          for (uint i=1;i<NUM_BININFO;i++)
                                                            binInfo[0].merge(binInfo[i]);
                                                          binInfo[0].atomic_merge_global(ctx.wgBuildState[buildStateID].binInfo);                                                             
                                                        }
                                                      });		  
                                                  });
        std::get<2>(*topBreadthFirst) = queue_event;
      }      

      /* partitioning */
      {
        assert(numPrimitives > cfg_maxLeafSize);
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                    /* local variables */
                                                    sycl::local_accessor< uint           , 0> _atomicCountLeft(cgh);
                                                    sycl::local_accessor< uint           , 0> _atomicCountRight(cgh);		  		  		  		  
                                                    
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(NUM_TOPLEVEL_WG*SIZE_TOPLEVEL_WG),sycl::range<1>(SIZE_TOPLEVEL_WG));
                                                    
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint localID      = item.get_local_id(0);                                                         
                                                        const uint groupID      = item.get_group(0);
                                                        const uint numGroups    = item.get_group_range(0);
                                                        const uint localSize    = item.get_local_range().size();

                                                        if (!(groupID < ctx.globals->numWGs)) return;
#if STABLE_ORDER && COHERENT_SUBSET                                                                                                                  
                                                        const gpu::AABB *const primref  = ctx.subsetAABB;
#else
                                                        const gpu::AABB *const primref  = ctx.aabb;
#endif
                                                        uint &atomicCountLeft    = *_atomicCountLeft.get_pointer();
                                                        uint &atomicCountRight   = *_atomicCountRight.get_pointer();

                                                        const gpu::WorkGroupTaskState &taskState = ctx.wgTaskState[groupID];
                                                        const uint recordID = taskState.recordID;
                                                        const uint parentID = recordID;
                                                        gpu::BVH2BuildRecord &current = ctx.bvh2[recordID];

                                                        const uint global_startID = current.start;
                                                        const uint global_endID   = current.end;
                                                        
                                                        const uint local_startID  = taskState.start;
                                                        const uint local_endID    = taskState.end;
                                                        const uint buildStateID   = taskState.buildStateID;
                                                        
                                                        gpu::BinMapping binMapping;    
                                                        binMapping.init(gpu::convert_AABB(current.bounds),BVH_BINNING_BINS);    
                                                                                                                    
                                                        atomicCountLeft  = 0;
                                                        atomicCountRight = 0;

                                                        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counterLeft(atomicCountLeft);
                                                        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> counterRight(atomicCountRight);

                                                        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> global_counterLeft (ctx.wgBuildState[buildStateID].atomicCountLeft);
                                                        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> global_counterRight(ctx.wgBuildState[buildStateID].atomicCountRight);
                                                        
                                                        gpu::Split split = ctx.wgBuildState[buildStateID].binInfo.reduceBinsAndComputeBestSplit(binMapping.scale,current.start,current.end,params.SAHLogBlockShift);

                                                        //const uint split_pos = ctx.wgBuildState[buildStateID].binInfo.computeSplitPosition(current.start,current.end,split);

                                                        
                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                                                                    
                                                        const uint size  = local_endID-local_startID;
                                                        
                                                        gpu::AABB3f lbounds, rbounds;
                                                        lbounds.init();
                                                        rbounds.init();
                                                                                                                    
                                                        if (split.sah != (float)(INFINITY))
                                                        {
                                                          /* sah split */
                                                          const uint aligned_size = gpu::alignTo(size,localSize);
                                                          for (uint t=0;t<aligned_size;t+=localSize)
                                                          {
                                                            uint offset, index;
                                                            bool l;

                                                            const uint ID = local_startID + localID + t;

                                                            
                                                            if (ID<local_endID)
                                                            {
                                                              //if ((ID < local_startID || ID >= local_endID) && groupID == 0) PRINT3(ID,local_startID,t);
                                                              
                                                              index = ctx.primref_index1[ID];
                                                              const gpu::AABB pref = primref[index];
                                                              l = is_left(binMapping, split,pref);

                                                              
                                                              if (l) 
                                                              {
                                                                const uint left_index = counterLeft.fetch_add(1);
                                                                lbounds.extend(pref.centroid2());
                                                                offset =left_index;
                                                              }
                                                              else
                                                              {
                                                                const uint right_index = counterRight.fetch_add(1);                                                                   
                                                                rbounds.extend(pref.centroid2());
                                                                offset = right_index;
                                                              }
                                                            }

                                                            item.barrier(sycl::access::fence_space::local_space);

                                                            uint global_offset_left = 0, global_offset_right = 0;
                                                              
                                                            if (localID == 0)
                                                            {
                                                              global_offset_left  = global_counterLeft.fetch_add(atomicCountLeft);
                                                              global_offset_right = global_counterRight.fetch_add(atomicCountRight);                                                                   
                                                              atomicCountLeft  = 0;
                                                              atomicCountRight = 0;
                                                            }
                                                            global_offset_left  = group_broadcast(item.get_group(),global_offset_left,0);                                                                
                                                            global_offset_right = group_broadcast(item.get_group(),global_offset_right,0);

                                                            if (ID<local_endID)
                                                            {
                                                              if (l)
                                                              {
                                                                offset = current.start + offset + global_offset_left;
                                                                ctx.primref_index0[offset] = index;

                                                                // if (offset >= split_pos && groupID == 0)
                                                                //   PRINT2("left",offset);

                                                              }
                                                              else
                                                              {
                                                                offset = global_endID - 1 - offset - global_offset_right;
                                                                // if (offset < split_pos  && groupID == 0)
                                                                //   PRINT2("right",offset);                                                                   
                                                                ctx.primref_index0[offset] = index;                                                                                                                                      
                                                              }

                                                            }
                                                            
                                                            item.barrier(sycl::access::fence_space::local_space);
                                                                                                                            
                                                          }
                                                          
                                                        }
                                                        else
                                                        {
                                                          /* fallback split */                                                             
                                                          const uint mid = (global_startID+global_endID)/2;                                                             
                                                          for (uint t=global_startID + localID;t<mid;t+=localSize)
                                                          {
                                                            const uint index = ctx.primref_index1[t];
                                                            lbounds.extend(primref[index].centroid2());
                                                          }
                                                          for (uint t=mid + localID;t<global_endID;t+=localSize)
                                                          {
                                                            const uint index = ctx.primref_index1[t];
                                                            rbounds.extend(primref[index].centroid2());
                                                          }
                                                        }
                                                        
                                                        lbounds.atomic_merge_global(ctx.wgBuildState[buildStateID].leftBounds);
                                                        rbounds.atomic_merge_global(ctx.wgBuildState[buildStateID].rightBounds);

                                                        item.barrier(sycl::access::fence_space::global_space);
                                                        
                                                        
                                                        /* last active HW thread ? */
                                                        if (localID == 0)
                                                        {
                                                          /* last WG done for record ?*/
                                                          sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> wg_counter(ctx.wgBuildState[buildStateID].numDoneWGs);
                                                          const uint wg = wg_counter.fetch_add(1);
                                                          if (wg + 1 == ctx.wgBuildState[buildStateID].numAssignedWGs)                                                             {
                                                            
                                                            /* split position */
                                                            const uint split_position = global_startID + ctx.wgBuildState[buildStateID].atomicCountLeft;


                                                            //UPRINT2(split_position,split_pos);
                                                          
                                                            /* write out two new ctx.bvh2 build records */
                                                            sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> bvh2_counter(ctx.globals->bvh2_index_allocator);
                                                            const uint bvh2_index = bvh2_counter.fetch_add(2);
                                                            current.left  = bvh2_index+0;
                                                            current.right = bvh2_index+1;                                                                                                                          
                                                            ctx.bvh2[bvh2_index+0].init(global_startID,split_position,parentID,ctx.wgBuildState[buildStateID].leftBounds);
                                                            ctx.bvh2[bvh2_index+1].init(split_position,global_endID  ,parentID,ctx.wgBuildState[buildStateID].rightBounds);                                                               
                                                          }

                                                          const uint sync = gpu::atomic_add_global<uint>(&ctx.globals->sync, 1);
                                                          if (sync + 1 == ctx.globals->numWGs)
                                                          {
                                                            ctx.globals->sync = 0;
                                                            
                                                            /* set final number of buildrecords */                                                             
                                                            ctx.globals->bvh2_breathfirst_records_start = 0;
                                                            ctx.globals->bvh2_breathfirst_records_end = ctx.globals->bvh2_index_allocator;
                                                          }
                                                        }
                                                        
                                                      });		  
                                                  });
        std::get<3>(*topBreadthFirst) = queue_event;
      }
    
    

#if CHECKS == 1
      //PRINT2(ctx.wgBuildState[0].atomicCountLeft,ctx.wgBuildState[0].atomicCountRight);
      //PRINT2(ctx.wgBuildState[0].leftBounds,ctx.wgBuildState[0].rightBounds);

      //PRINT(ctx.globals->bvh2_breathfirst_records_start);
      //PRINT(ctx.globals->bvh2_breathfirst_records_end);
      
      for (uint i=ctx.globals->bvh2_breathfirst_records_start;i<ctx.globals->bvh2_breathfirst_records_end;i++)
      {
        gpu::BVH2BuildRecord &current = ctx.bvh2[i];
        //PRINT4(current.start,current.end,current.size(),current.bounds);
        gpu::AABB bounds = convert_AABB(current.bounds);          
        for (uint j=current.start;j<current.end;j++)
        {
          if (!bounds.encloses(ctx.aabb[ctx.primref_index0[j]].centroid2()))
            PRINT3(j,gpu::AABB(ctx.aabb[ctx.primref_index0[j]].centroid2()),bounds);
          assert(bounds.encloses(ctx.aabb[ctx.primref_index0[j]].centroid2()));
        }
      }
    uint count_middle = 0;
    for (uint i=0;i<ctx.globals->bvh2_breathfirst_records_end;i++)
      {
        gpu::BVH2BuildRecord &current = ctx.bvh2[i];
        if (current.size() > cfg_maxLeafSize && current.left == -1)
        {
          //PRINT4(current.start,current.end,current.size(),current.bounds);
          count_middle++;
        }
      }
    PRINT(count_middle);
      
#endif
      printresetPhistStochHW("u", gpu_queue, ctx.phist);
    }
#endif

    gpu::waitOnQueueAndCatchException(gpu_queue);
    ectx.topBreadthFirstEnd = getSeconds();

    if (false) {
      uint nodes = 0;
      uint leaves = 0;
      double nodeSAH = 0;
      double leafSAH = 0;      
#if STABLE_ORDER && COHERENT_SUBSET
      auto* aabb = ctx.subsetAABB;
#else
      auto* aabb = ctx.aabb;
#endif
      float area = checkBVH2Stoch2HW(ctx.bvh2,ctx.globals->rootIndex,ctx.primref_index,aabb,nodes,leaves,nodeSAH,leafSAH,cfg_maxLeafSize).area();
      nodeSAH /= area;
      leafSAH /= area;                
      PRINT4(nodes,leaves,(float)nodeSAH,(float)leafSAH);
    }

    // === build top-level tree breadth-first ===
    uint numWGs = 64;
    uint sizeWG = 1024;
    for (uint iteration = 0; iteration < 12 /*ctx.globals->bvh2_breathfirst_records_start < ctx.globals->bvh2_breathfirst_records_end*/;iteration++)
    {
      if (sizeWG > 256 && iteration > 0) {
        sizeWG /= 2;
        numWGs *= 2;
      }
      assert(numPrimitives > cfg_maxLeafSize);
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //6
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(numWGs*sizeWG),sycl::range<1>(sizeWG));
  
                                                    /* local variables */
                                                    sycl::local_accessor< gpu::BinInfoNew, 0> _binInfo(cgh);
                                                    sycl::local_accessor< gpu::Split     , 0> _bestSplit(cgh);		  
                                                    sycl::local_accessor< uint           , 0> _atomicCountLeft(cgh);
                                                    sycl::local_accessor< uint           , 0> _atomicCountRight(cgh);		  		  		  		  
                                                    sycl::local_accessor< gpu::AABB3f    , 0> _leftBounds(cgh);
                                                    sycl::local_accessor< gpu::AABB3f    , 0> _rightBounds(cgh);
                                                                                  
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint localID     = item.get_local_id(0);                                                         
                                                        const uint groupID     = item.get_group(0);
                                                        const uint numGroups   = item.get_group_range(0);

                                                        gpu::BinInfoNew &binInfo = *_binInfo.get_pointer();
                                                        gpu::Split &bestSplit    = *_bestSplit.get_pointer();
                                                        uint &atomicCountLeft    = *_atomicCountLeft.get_pointer();
                                                        uint &atomicCountRight   = *_atomicCountRight.get_pointer();
                                                        gpu::AABB3f &leftBounds  = *_leftBounds.get_pointer();
                                                        gpu::AABB3f &rightBounds = *_rightBounds.get_pointer();
                                                                                                                

#if STABLE_ORDER && COHERENT_SUBSET                                                                                                                  
                                                        const gpu::AABB *const primref  = ctx.subsetAABB;
#else
                                                        const gpu::AABB *const primref  = ctx.aabb;
#endif

                                                        const uint record_start = ctx.globals->bvh2_breathfirst_records_start;
                                                        const uint num_records = ctx.globals->bvh2_breathfirst_records_end - record_start;
                                                        
#if 1
                                                        for (int i = groupID; i < num_records; i += numWGs) {
                                                        const uint recordID = record_start + i;
#else                                                           
                                                        while (true) {
                                                        
                                                        item.barrier(sycl::access::fence_space::local_space);
                                                        if (localID == 0) atomicCountLeft = gpu::atomic_add_global<uint>(&ctx.globals->sched, 1);
                                                        item.barrier(sycl::access::fence_space::local_space);
                                                        if (!(atomicCountLeft < num_records)) break;
                                                        const uint recordID = record_start + atomicCountLeft;
#endif                                                           

                                                        if (ctx.bvh2[recordID].size() > cfg_maxLeafSize && ctx.bvh2[recordID].left == -1)
                                                        {
                                                          gpu::BVH2BuildRecord &current = ctx.bvh2[recordID];
                                                          if (current.size() > SUBTREE_THRESHOLD)
                                                          {
                                                            if (localID == 0)
                                                              writePHistStochHW(ctx.phist, ctx.bvh2, recordID);
                                                            const uint parentID = recordID;
                                                            const uint startID = current.start;
                                                            const uint endID   = current.end;
                                                        
                                                            bin_partition_local(item,current,binInfo,bestSplit,atomicCountLeft,atomicCountRight,leftBounds,rightBounds,primref,ctx.primref_index0,ctx.primref_index1,params.SAHLogBlockShift,cfg_maxLeafSize);

                                                            if (localID == 0)
                                                            {                                                                                                                          
                                                              /* split position */
                                                              const uint split_position = startID + atomicCountLeft;
                                                          
                                                              /* write out two new ctx.bvh2 build records */
                                                              sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> bvh2_counter(ctx.globals->bvh2_index_allocator);
                                                              const uint bvh2_index = bvh2_counter.fetch_add(2);
                                                              current.left  = bvh2_index+0;
                                                              current.right = bvh2_index+1;                                                                                                                          
                                                              ctx.bvh2[bvh2_index+0].init(startID,split_position,parentID,leftBounds);
                                                              ctx.bvh2[bvh2_index+1].init(split_position,endID  ,parentID,rightBounds);

                                                            }
                                                          }
                                                        }

                                                        }
                                                                                                              
                                                        /* last active HW thread ? */
                                                        if (localID == 0)
                                                        {
                                                          const uint sync = gpu::atomic_add_global<uint>(&ctx.globals->sync, 1);
                                                          if (sync + 1 == numGroups)
                                                          {
                                                            ctx.globals->sync = 0;
                                                            /* set final number of buildrecords */                                                             
                                                            ctx.globals->bvh2_breathfirst_records_start = ctx.globals->bvh2_breathfirst_records_end;
                                                            ctx.globals->bvh2_breathfirst_records_end = ctx.globals->bvh2_index_allocator;
                                                            ctx.globals->sched = 0;
                                                          }
                                                        }
                                                      

                                                      });		  
                                                  });
      ectx.topMiddle.push_back(queue_event);

    
#if CHECKS == 1          
      PRINT(ctx.globals->bvh2_breathfirst_records_start);
      PRINT(ctx.globals->bvh2_breathfirst_records_end);
    
      for (uint i=ctx.globals->bvh2_breathfirst_records_start;i<ctx.globals->bvh2_breathfirst_records_end;i++)
      {
        gpu::BVH2BuildRecord &current = ctx.bvh2[i];
        gpu::AABB bounds = convert_AABB(current.bounds);          
        for (uint j=current.start;j<current.end;j++)
        {
          if (!bounds.encloses(ctx.aabb[ctx.primref_index0[j]].centroid2()))
            PRINT3(j,gpu::AABB(ctx.aabb[ctx.primref_index0[j]].centroid2()),bounds);
          assert(bounds.encloses(ctx.aabb[ctx.primref_index0[j]].centroid2()));
        }
      }
#endif
      printresetPhistStochHW("m", gpu_queue, ctx.phist);
    }

    gpu::waitOnQueueAndCatchException(gpu_queue);
    ectx.topMiddleEnd = getSeconds();

    if (false) {
      uint nodes = 0;
      uint leaves = 0;
      double nodeSAH = 0;
      double leafSAH = 0;      
#if STABLE_ORDER && COHERENT_SUBSET
      auto* aabb = ctx.subsetAABB;
#else
      auto* aabb = ctx.aabb;
#endif
      float area = checkBVH2Stoch2HW(ctx.bvh2,ctx.globals->rootIndex,ctx.primref_index,aabb,nodes,leaves,nodeSAH,leafSAH,cfg_maxLeafSize).area();
      nodeSAH /= area;
      leafSAH /= area;                
      PRINT4(nodes,leaves,(float)nodeSAH,(float)leafSAH);
    }

    }

    // ============================================        
    // === build BVH2 over all top-level ranges ===
    // ============================================

    {
      const uint numWGs = 4096;
      const uint sizeWG = 16;        
      assert(numPrimitives > cfg_maxLeafSize);
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //18
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(numWGs*sizeWG),sycl::range<1>(sizeWG));		  
                                                    /* local variables */
                                                    sycl::local_accessor< uint           , 0> _atomicCountLeft(cgh);
                                                    sycl::local_accessor< uint           , 0> _atomicCountRight(cgh);		  		  		  		  
                                                    sycl::local_accessor< uint           , 1> _stack(sycl::range<1>(128),cgh);
                                                                                  
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint localID      = item.get_local_id(0);                                                         
                                                        const uint groupID      = item.get_group(0);
                                                        const uint numGroups   = item.get_group_range(0);
                                                        uint &atomicCountLeft   = *_atomicCountLeft.get_pointer();                                                         
                                                        uint &atomicCountRight  = *_atomicCountRight.get_pointer();
                                                        uint *stack             =  _stack.get_pointer();
                                                                                                                

#if STABLE_ORDER && COHERENT_SUBSET                                                                                                                  
                                                        const gpu::AABB *const primref  = ctx.subsetAABB;
#else
                                                        const gpu::AABB *const primref  = ctx.aabb;
#endif
                                                        
                                                        for (int i = groupID; i < ctx.globals->bvh2_breathfirst_records_end; i += numGroups) {

                                                        const uint recordID = i;

                                                        if (ctx.bvh2[recordID].isLeaf() && ctx.bvh2[recordID].size() > cfg_maxLeafSize)
                                                        {
                                                          uint sindex = 1;
                                                          stack[0] = recordID;
                                                          
                                                          while(sindex)
                                                          {
                                                            sindex--;                                                               
                                                            const uint ID = stack[sindex];
                                                            gpu::BVH2BuildRecord &current = ctx.bvh2[ID];
                                                            if (localID == 0)
                                                              writePHistStochHW(ctx.phist, ctx.bvh2, ID);
                                                            
                                                            const uint parentID = ID;
                                                            const uint startID = current.start;
                                                            const uint endID   = current.end;
                                                            
                                                            gpu::AABB3f leftBounds, rightBounds;
                                                            bin_partition_register(item,current,atomicCountLeft,atomicCountRight,leftBounds,rightBounds,primref,ctx.primref_index0,ctx.primref_index1,params.SAHLogBlockShift,cfg_maxLeafSize);

                                                            /* split position */
                                                            const uint split_position = startID + atomicCountLeft;

                                                            /* allocate new ctx.bvh2 node */
                                                            uint bvh2_index = 0; 
                                                            if (localID == 0)
                                                            {
                                                              sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> bvh2_counter(ctx.globals->bvh2_index_allocator);
                                                              bvh2_index = bvh2_counter.fetch_add(2);
                                                            }
                                                            bvh2_index = sub_group_broadcast(bvh2_index,0);
                                                              
                                                            /* write out two new ctx.bvh2 build records */
                                                            current.left  = bvh2_index+0;
                                                            current.right = bvh2_index+1;                                                                                                                          
                                                            ctx.bvh2[bvh2_index+0].init(startID,split_position,parentID,leftBounds);
                                                            ctx.bvh2[bvh2_index+1].init(split_position,endID  ,parentID,rightBounds);
                                                            
                                                            /* left child non leaf? */
                                                            if (split_position - startID > cfg_maxLeafSize)
                                                              stack[sindex++] = bvh2_index+0;
                                                            
                                                            /* right child non leaf? */                                                               
                                                            if (endID - split_position  > cfg_maxLeafSize)
                                                              stack[sindex++] = bvh2_index+1;                                                               
                                                          }                                                             
                                                        
                                                        }
                                                                                                                                                                        
                                                        }

                                                      });		  
                                                  });
      ectx.topBottom = queue_event;
    }

    gpu::waitOnQueueAndCatchException(gpu_queue);      

    printresetPhistStochHW("b", gpu_queue, ctx.phist);

    ectx.topBottomEnd = getSeconds();
    
    const uint numTopLevelNodes = ctx.globals->bvh2_index_allocator;

    if (false) {
      uint nodes = 0;
      uint leaves = 0;
      double nodeSAH = 0;
      double leafSAH = 0;      
#if STABLE_ORDER && COHERENT_SUBSET
      auto* aabb = ctx.subsetAABB;
#else
      auto* aabb = ctx.aabb;
#endif
      float area = checkBVH2Stoch2HW(ctx.bvh2,ctx.globals->rootIndex,ctx.primref_index,aabb,nodes,leaves,nodeSAH,leafSAH,cfg_maxLeafSize).area();
      nodeSAH /= area;
      leafSAH /= area;                
      PRINT4(nodes,leaves,(float)nodeSAH,(float)leafSAH);
    }
    
    PRINT(numTopLevelNodes);        

  }

 __forceinline uint FindBestNode(const BuildParametersHW &params, const BuildContextHW &ctx, const int idx, const uint primitiveID, const gpu::AABB *const aabb, const gpu::BVH2BuildRecord *const bvh2)
 {
   const gpu::AABB3f primitive_bounds = convert_AABB3f(aabb[primitiveID]);

   float mincost = FLT_MAX; 
   uint bestnodeID = -1;

   int subsetPrimitiveIdx = ctx.closestSubsetPrimitive[idx];
   int sbegin = max(subsetPrimitiveIdx - params.localMortonSearchWindow, (int)0);
   int send = min(subsetPrimitiveIdx + params.localMortonSearchWindow, (int)ctx.subsetSize);

   int csbegin = sub_group_reduce_min(sbegin);
   int csend = sub_group_reduce_max(send);

   {
      int i = subsetPrimitiveIdx;
      const uint org_nodeID = ctx.subsetNodeSelections[i];
      uint nodeID = org_nodeID;

      float nPrim = bvh2[nodeID].end - bvh2[nodeID].start;
      float SA = merge(bvh2[nodeID].bounds, primitive_bounds).area();
      float cost = SA * (nPrim + 1) - bvh2[nodeID].bounds.area() * nPrim;
      
      while(bvh2[nodeID].parent != 0x7fffffff) {
          nodeID = bvh2[nodeID].parent;
          float diff = merge(bvh2[nodeID].bounds, primitive_bounds).area() - bvh2[nodeID].bounds.area();
          cost += diff;
          if (diff < FLT_EPSILON) break;
      };

      mincost = cost;
      bestnodeID = org_nodeID;
   }
                                                                 
   for (int i = csbegin; i < csend; i++) {
     const uint org_nodeID = ctx.subsetNodeSelections[i];
     uint nodeID = org_nodeID;

     if (nodeID == bestnodeID) continue;

     float nPrim = bvh2[nodeID].end - bvh2[nodeID].start;
     float SA = merge(bvh2[nodeID].bounds, primitive_bounds).area();
     float cost = SA * (nPrim + 1) - bvh2[nodeID].bounds.area() * nPrim;
     
     while(bvh2[nodeID].parent != 0x7fffffff && cost < mincost) {
        nodeID = bvh2[nodeID].parent;
        float diff = merge(bvh2[nodeID].bounds, primitive_bounds).area() - bvh2[nodeID].bounds.area();
        cost += diff;
        if (diff < FLT_EPSILON) break;
     };
     
     if(cost < mincost)
     {
       mincost = cost;
       bestnodeID = org_nodeID;
     }
   }

   return bestnodeID;   
  }

// TODO experiment with short stacks
// This is fastest, but breaks with larger scenes or subset fractions (stack size gets larger)
 __forceinline uint FindBestNode2(const BuildParametersHW &params, const BuildContextHW &ctx, const int idx, const gpu::AABB *const aabb, const gpu::BVH2BuildRecord *const bvh2)
 {
   const uint primitiveID = idx;
   const gpu::AABB3f primBounds = convert_AABB3f(aabb[primitiveID]);
   const float leafCostBound = primBounds.area();


   float bestCost = FLT_MAX;
   int bestNodeID = -1;

   int nodeID = 0;
   float cost = 0;
   int stackSize = 0;
   int stack[26];
   float costs[26];
   
   auto nodeCost = [&](const uint nodeID) {
      const auto &node = bvh2[nodeID];
      float oldSA = node.bounds.area();
      float newSA = merge(node.bounds, primBounds).area();
      if (node.isLeaf()) {
        float nPrim = node.end - node.start;
        float nodeCost = cost + newSA * (nPrim + 1) - oldSA * nPrim;
        if (nodeCost < bestCost) {
          bestCost = nodeCost;
          bestNodeID = nodeID;
        }
        return FLT_MAX;
      } else {
        return cost + newSA - oldSA;
      }
   };

   while (true) {
     const auto &node = bvh2[nodeID];

      float leftCost = nodeCost(node.left);
      float rightCost = nodeCost(node.right);

      const float minCost = min(leftCost, rightCost);
      const float maxCost = max(leftCost, rightCost);
      const int   minNodeIdx  = leftCost == minCost ? node.left : node.right;
      const int   maxNodeIdx  = leftCost == minCost ? node.right : node.left;

      float minCostBound = leafCostBound;// max(leafCostBound, ctx.minLeafSA[minNodeIdx]);
      if (minCost + minCostBound < bestCost) {
        float maxCostBound = leafCostBound;// max(leafCostBound, ctx.minLeafSA[maxNodeIdx]);
        if (maxCost + maxCostBound < bestCost) {
          stack[stackSize] = maxNodeIdx;
          costs[stackSize] = maxCost;
          stackSize++;
        }

        nodeID = minNodeIdx;
        cost = minCost;
      } else if (stackSize > 0) {
        while (stackSize > 0) {
          stackSize--;
          nodeID = stack[stackSize];
          cost = costs[stackSize];
          if (cost + leafCostBound < bestCost) break;
          if (stackSize == 0) return bestNodeID;
        }
      } else return bestNodeID; 
   }
  }

 __forceinline uint FindBestNode3(const BuildParametersHW &params, const BuildContextHW &ctx, const int idx, const gpu::AABB *const aabb, const gpu::BVH2BuildRecord *const bvh2)
 {
   const gpu::AABB3f primBounds = convert_AABB3f(aabb[idx]);
   const float leafCostBound = primBounds.area();

   float bCost = FLT_MAX, cCost = 0;
   int bNodeID = -1, cNodeID = 0;

   auto nodeCost = [&](const uint nodeID) {
      const auto &node = bvh2[nodeID];
      float oldSA = node.bounds.area();
      float newSA = merge(node.bounds, primBounds).area();
      float diff = newSA - oldSA;
      if (!node.isLeaf()) return diff;
      float cost = cCost + diff * (node.end - node.start) + newSA;
      if (cost < bCost) {
        bCost = cost;
        bNodeID = nodeID;
      }
      return FLT_MAX;
   };

   auto nodeCost2 = [&](const uint nodeID) {
      const auto &node = bvh2[nodeID];
      float oldSA = node.bounds.area();
      float newSA = merge(node.bounds, primBounds).area();
      return node.isLeaf() ? FLT_MAX : newSA - oldSA;
   };
   
   bool down = true;
   while (true) {
     const auto &cNode = bvh2[cNodeID];
     if (down) {
       const float leftCost = nodeCost(cNode.left);
       const float rightCost = nodeCost(cNode.right);

       const float minCost = min(leftCost, rightCost);
       const int   minNodeIdx  = leftCost == minCost ? cNode.left : cNode.right;

       if (cCost + minCost + leafCostBound < bCost) {
         cNodeID = minNodeIdx;
         cCost += minCost;
       } else
         down = false;
     } else {
       const auto pNodeID = cNode.parent;
       if (pNodeID == 0x7fffffff) break;
       const auto &pNode = bvh2[pNodeID];
       const auto sNodeID = pNode.left == cNodeID ? pNode.right : pNode.left;
       const auto &sNode = bvh2[sNodeID];
       const auto dCost = nodeCost2(cNodeID);
       const auto sCost = nodeCost2(sNodeID);

       cCost -= dCost;

       bool second = sCost > dCost || (sCost == dCost && pNode.left == cNodeID);
       if (second && cCost + sCost + leafCostBound < bCost) {
         cNodeID = sNodeID;
         cCost += sCost;
         down = true;
       } else {
         cNodeID = pNodeID;
       }
     }
   }

   return bNodeID;
  }

 __forceinline uint FindBestNode4(const BuildParametersHW &params, const BuildContextHW &ctx, const int idx, const gpu::AABB *const aabb, const gpu::BVH2BuildRecord *const bvh2)
 {
   const gpu::AABB3f primBounds = convert_AABB3f(aabb[idx]);
   const float leafCostBound = primBounds.area();

   float bCost = FLT_MAX, cCost = 0;
   int bNodeID = -1, cNodeID = 0;
   bool down = true;

   auto nodeCost = [&](const uint nodeID) {
      const auto &node = bvh2[nodeID];
      float oldSA = node.bounds.area();
      float newSA = merge(node.bounds, primBounds).area();
      float diff = newSA - oldSA;
      if (!node.isLeaf()) return diff;
      if (down) {
        float cost = cCost + diff * (node.end - node.start) + newSA;
        if (cost < bCost) {
          bCost = cost;
          bNodeID = nodeID;
        }
      }
      return FLT_MAX;
   };

   while (true) {
     const auto pNodeID = cNodeID;
     const auto *pNode = &bvh2[pNodeID];

     if (!down) {
       cNodeID = pNode->parent;
       if (cNodeID == 0x7fffffff) break;
     }

     const auto *cNode = &bvh2[cNodeID];

     const float leftCost = nodeCost(cNode->left);
     const float rightCost = nodeCost(cNode->right);

     if (down) {
       const float minCost = min(leftCost, rightCost);
       const int   minNodeIdx  = leftCost == minCost ? cNode->left : cNode->right;

       if (cCost + minCost + leafCostBound < bCost) {
         cNodeID = minNodeIdx;
         cCost += minCost;
       } else
         down = false;
     } else {
       const auto sNodeID = cNode->left == pNodeID ? cNode->right : cNode->left;
       const auto dCost = cNode->left == pNodeID ? leftCost : rightCost;
       const auto sCost = cNode->left == pNodeID ? rightCost : leftCost;

       cCost -= dCost;

       bool second = sCost > dCost || (sCost == dCost && cNode->left == pNodeID);
       if (second && cCost + sCost + leafCostBound < bCost) {
         cNodeID = sNodeID;
         cCost += sCost;
         down = true;
       }
     }
   }

   return bNodeID;
  }

  // =================================================================================================================================================================================================
  // ================================================================================= CHECK/CONVERT BVH2 ============================================================================================
  // =================================================================================================================================================================================================

  gpu::AABB3f checkBVH2Stoch2HW(gpu::BVH2BuildRecord *bvh_nodes, uint index,const uint *const primref_index,gpu::AABB *aabb,uint &nodes,uint &leaves,double &nodeSAH, double &leafSAH, const uint cfg_maxLeafSize)
  {
    gpu::AABB3f bounds;

    if (bvh_nodes[index].isLeaf())
    {      
      leaves++;
      bounds.init();
      for (uint i=bvh_nodes[index].start;i<bvh_nodes[index].end;i++)
      {
        const gpu::AABB3f primBounds = convert_AABB3f( aabb[ primref_index[ i ]] );
        bounds = merge(bounds, primBounds);
        leafSAH += primBounds.area();
      }
    }
    else
    {
      nodes++;
      gpu::AABB3f lB = checkBVH2Stoch2HW(bvh_nodes,bvh_nodes[index].left,primref_index,aabb,nodes,leaves,nodeSAH,leafSAH,cfg_maxLeafSize);
      gpu::AABB3f rB = checkBVH2Stoch2HW(bvh_nodes,bvh_nodes[index].right,primref_index,aabb,nodes,leaves,nodeSAH,leafSAH,cfg_maxLeafSize);
      bounds = merge(lB, rB);
      nodeSAH += bounds.area();
    }
    return bounds;
  }


    void checkBVH2StochHW(gpu::BVH2BuildRecord *bvh_nodes, uint index,const uint *const primref_index,gpu::AABB *aabb,uint &nodes,uint &leaves,double &nodeSAH, double &leafSAH, const uint cfg_maxLeafSize)
  {
    //PRINT3(index,bvh_nodes[index].isLeaf(),bvh_nodes[index]);
    if (!bvh_nodes[index].bounds.isValid()) {
      FATAL("invalid bounds in BVH2");
    }

    if (bvh_nodes[index].isLeaf())
    {      
      for (uint i=bvh_nodes[index].start;i<bvh_nodes[index].end;i++)
      {
        assert(bvh_nodes[index].bounds.encloses( convert_AABB3f( aabb[ primref_index[ i ]  ])));
      }

      
      leaves++;
      for (uint i=bvh_nodes[index].start;i<bvh_nodes[index].end;i++)      
        leafSAH +=  aabb[ primref_index[i] ].area();                    
    }
    else
    {
      nodes++;
      nodeSAH += bvh_nodes[index].bounds.area();
      if (bvh_nodes[index].left != -1)
      {
        assert(bvh_nodes[index].bounds.encloses( bvh_nodes[ bvh_nodes[index].left ].bounds ));        
        checkBVH2StochHW(bvh_nodes,bvh_nodes[index].left,primref_index,aabb,nodes,leaves,nodeSAH,leafSAH,cfg_maxLeafSize);
      }
      
      if (bvh_nodes[index].right != -1)
      {
        assert(bvh_nodes[index].bounds.encloses( bvh_nodes[ bvh_nodes[index].right ].bounds ));        
        checkBVH2StochHW(bvh_nodes,bvh_nodes[index].right,primref_index,aabb,nodes,leaves,nodeSAH,leafSAH,cfg_maxLeafSize);
      }
    }
  }

  // ===================================================================================================================================================================================
  // ================================================================================= BVH2 ============================================================================================
  // ===================================================================================================================================================================================

  void refitGeometryBoundsCalculateSAHHW(gpu::BVH2BuildRecord *const bvh_nodes, const uint index, uint *const primref_index, const gpu::AABB *const aabb)
  {
    auto &current = bvh_nodes[index];
    //PRINT5(index,bvh_nodes[index].start,bvh_nodes[index].end,bvh_nodes[index].left,bvh_nodes[index].right);

    if (bvh_nodes[index].isLeaf())
    {
      gpu::AABB3f bounds;
      bounds.init();
      auto start = current.start, end = current.end;
      for (uint i=start;i<end;i++)
      {
        uint primID = primref_index[i];
        bounds.extend(convert_AABB3f(aabb[primID]));
      }                     
      current.bounds = bounds;
    }
    else
    {
      const auto &l = bvh_nodes[current.left];
      const auto &r = bvh_nodes[current.right];

      refitGeometryBoundsCalculateSAHHW(bvh_nodes,current.left,primref_index,aabb);
      refitGeometryBoundsCalculateSAHHW(bvh_nodes,current.right,primref_index,aabb);

      current.bounds = merge(l.bounds,r.bounds);

      assert( current.bounds.encloses( merge(l.bounds, r.bounds ) ) );
    }
  }

  void getLeafIndicesStoch(const uint index, const gpu::BVH2BuildRecord *const bvh_nodes, const uint* primref_index, uint *const dest, uint &indexID)
  {
    const auto & node = bvh_nodes[index];
    if (node.isLeaf())
    {
      for (uint i = node.start; i < node.end; i++)
        dest[indexID++] = primref_index[i];
    }
    else
    {
      getLeafIndicesStoch(node.left, bvh_nodes, primref_index, dest, indexID);
      getLeafIndicesStoch(node.right,bvh_nodes, primref_index, dest, indexID);
    }    
  }

  // ====================================================================================================================================================================================
  // ====================================================================================================================================================================================
  // ====================================================================================================================================================================================

  // *************************************************
  // TEMPORARY SOLUTION TO PASS PARAMS TO THE BUILDER
  // TODO: MOVE TO CMD
  // *************************************************
  void parseBuildParams(BuildParametersHW &params)
  {
    /*
    // for now we have a fixed file in the experiment folder
    // TODO make it command-line
    char cwd[PATH_MAX];
    getcwd(cwd, sizeof(cwd)); // avoids memory leaks from NULL args
    std::string cpath(cwd), token;
    int fpos = cpath.find("embree");
    cpath = cpath.substr(0,fpos) + "embree/experiments/params.txt";
    FILE* f = fopen(cpath.c_str(),"r");
    if (fpos == -1 || !f) 
    {
      printf("Cannot find %s, using default BuildParams\n",cpath.c_str());
      return;
    }
    Ref<Stream<int> > file = new FileStream(f,cpath);
    
    std::vector<std::string> syms;
    const char* symbols[3] = { "=", ",", "|" };
    for (size_t i=0; i<sizeof(symbols)/sizeof(void*); i++) 
      syms.push_back(symbols[i]);
    
    Ref<TokenStream> cin = new TokenStream(new LineCommentFilter(file,"#"),
                                          TokenStream::alpha+TokenStream::ALPHA+TokenStream::numbers+"_.",
                                          TokenStream::separators,syms);
    
    while (cin->peek() != Token::Eof())
    {
      const Token tok = cin->get();
      // Token::Id("string") compares false due to TY type mismatch
      std::string stok = tok.String();

      if (stok == "minsetsize")
        params.minSetSize = cin->get().Int();
      else if (stok == "subsetsizefrac") 
        params.subsetSizeFrac = cin->get().Float();
      else if (stok == "uniformcdffrac") 
        params.uniformCdfFrac = cin->get().Float();
      // TODO move code to use "samplingMethod" string to enum  
      else if (stok == "samplingMethod") 
        params.useSobol = cin->get().String() == "sobol";
      else if (stok == "randomseed"){
        // no unsigned function, hacky conversion
        int rand = cin->get().Int();
        params.randomSeed = *((unsigned*)(&rand));
      }
       else if (stok == "clustering") 
        params.deterministicClustering = cin->get().String() == "deterministic";
      else if (stok == "clusteringMortonBits")
        params.clusteringMortonBits = cin->get().Int();  
      else if (stok == "representativeprimitivesbbox")
        params.representativePrimitivesBBox = cin->get().String() == "true";
      else if (stok == "representativeprimitivesbboxpower") 
        params.representativePrimitivesBBoxPower = cin->get().Float();
      else if (stok == "representativeprimtivescost") 
        params.representativePrimitivesCost = cin->get().String() == "true";
      else if (stok == "localmortonsearchwindow")
        params.localMortonSearchWindow = cin->get().Int();
    }
    params.printParameters(0);
    */
  }

  BBox3fa rthwifBuildStoch(DeviceGPU* deviceGPU, sycl::queue &gpu_queue, const uint numPrimitives, gpu::AABB *aabb)
  {
    PING;
    /*
    DeviceGPU* deviceGPU = dynamic_cast<DeviceGPU*>(scene->device);
    assert(deviceGPU);
    
    size_t org_numPrimitives = 0;
    for (size_t geomID = 0; geomID < scene->size(); geomID++)
    {
      if (GridMesh* mesh = scene->getSafe<GridMesh>(geomID))
        org_numPrimitives += mesh->getNumTotalQuads(); // FIXME: slow
      else
        org_numPrimitives += scene->get(geomID)->size();
    }

    const size_t bytes = 2*4096 + org_numPrimitives*(3*64);
    if (accel.size() < bytes) accel = std::move(Device::avector<char,64>(scene->device,bytes));
    memset(accel.data(),0,accel.size());

    FastAllocator alloc(scene->device, false, true, false);
    alloc.addBlock(accel.data(),accel.size());

    // FastAllocator::CachedAllocator thread_alloc = alloc.getCachedAllocator();
    // thread_alloc.malloc0(128-FastAllocator::blockHeaderSize);

    mvector<PrimRef> prims(scene->device,0);
    prims.getAlloc().enableUSM(&deviceGPU->getGPUDevice(),&deviceGPU->getGPUContext(),&deviceGPU->getGPUQueue());	    
    prims.resize(org_numPrimitives); 

    //mvector<PrimRef> prims(scene->device,org_numPrimitives);
    
    PrimInfo pinfo = rthwifCreatePrimRefArrayStoch(scene,Geometry::MTY_ALL,false,org_numPrimitives,prims,scene->progressInterface);

    const uint numPrimitives = pinfo.size();
    PRINT2(org_numPrimitives,numPrimitives);
    */
    
    const int maxWorkGroupSize = deviceGPU->getGPUMaxWorkGroupSize();

    const uint gpu_maxWorkGroupSize = deviceGPU->getGPUDevice().get_info<sycl::info::device::max_work_group_size>();
    const uint gpu_maxComputeUnits  = deviceGPU->getGPUDevice().get_info<sycl::info::device::max_compute_units>();    
    const uint gpu_maxLocalMemory   = deviceGPU->getGPUDevice().get_info<sycl::info::device::local_mem_size>();
    const uint gpu_maxSubgroups     = gpu_maxComputeUnits * 8;

    PRINT( deviceGPU->getGPUDevice().get_info<sycl::info::device::global_mem_size>() );
    PRINT(gpu_maxWorkGroupSize);
    PRINT(gpu_maxComputeUnits);
    PRINT(gpu_maxLocalMemory);
    PRINT(gpu_maxSubgroups);
    
    GeneralBVHBuilder::Settings settings;
    const uint cfg_SAHLogBlockShift = settings.logBlockSize;
    /* build down to 1 primitive per leaf */
    const uint cfg_maxLeafSize_TopLevel = 8;
    const uint cfg_maxLeafSize_BottomLevel = 6;
    
    /* --- estimate size of the BVH --- */
    const uint leaf_primitive_size = 64;
    const uint node_size       = 0;//max( (uint)(sizeof(uint)*8*numPrimitives),(uint)( (numPrimitives+15)/8 * sizeof(BVH::AlignedNode))); // need at least 32 bytes * numPrimitives node memory for storing the clusters
    const uint leaf_size       = numPrimitives * 64; // REMOVE !!!
    const uint totalSize       = /*sizeof(gpu::BVHBase) +*/ node_size + leaf_size; 
    const uint node_data_start = /*sizeof(gpu::BVHBase) +*/ 0;
    const uint leaf_data_start = /*sizeof(gpu::BVHBase) +*/ node_size;

    //if (unlikely(deviceGPU->verbosity(2)))
    {
      PRINT( maxWorkGroupSize );	
      PRINT( leaf_primitive_size );
      PRINT( node_size );
      PRINT( leaf_size );	
      PRINT( totalSize );
      PRINT( numPrimitives );
    }

    assert( (leaf_data_start % 64) == 0 );
    /* --- allocate and set buffers --- */

    double alloc_time0 = getSeconds();

    //gpu::AABB *aabb = (gpu::AABB*)prims.data();
    assert(aabb);

    char *bvh_mem = (char*)sycl::aligned_alloc(64,totalSize,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
    assert(bvh_mem);
    uint *primref_index = (uint*)sycl::aligned_alloc(64,sizeof(uint)*numPrimitives * 2,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared); 
    uint *primref_index0 = primref_index;
    uint *primref_index1 = primref_index + numPrimitives;
    
    assert(primref_index);	        
    gpu::Globals *globals = (gpu::Globals*)sycl::aligned_alloc(64,sizeof(gpu::Globals),deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared);
    assert(globals);

    /* -- up to 262k for radix sort histograms (256 WGs max) --*/
    uint *scratch_mem = (uint*)sycl::aligned_alloc(64,sizeof(uint)*RADIX_SORT_BINS*(RADIX_SORT_MAX_NUM_DSS+1),deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared); // FIXME
    assert(scratch_mem);

    char *const leaf_mem = bvh_mem + leaf_data_start;
    
    gpu::BVH2BuildRecord *const bvh2          = (gpu::BVH2BuildRecord*)(leaf_mem);

    gpu::WorkGroupBuildState *wgBuildState = (gpu::WorkGroupBuildState *)sycl::aligned_alloc(64,sizeof(gpu::WorkGroupBuildState)*NUM_TOPLEVEL_WG,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared); 
    PRINT(sizeof(gpu::WorkGroupBuildState));

    gpu::WorkGroupTaskState   *wgTaskState = (gpu::WorkGroupTaskState *)sycl::aligned_alloc(64,sizeof(gpu::WorkGroupTaskState)*NUM_TOPLEVEL_WG,deviceGPU->getGPUDevice(),deviceGPU->getGPUContext(),sycl::usm::alloc::shared); 
            
    MCPrim *const mc0 = (MCPrim*)(bvh2 + numPrimitives);
    
    MCPrim *const morton_codes[2] = { mc0, mc0 + gpu::alignTo(numPrimitives,8)}; 
    
    const size_t totalUSMAllocations = totalSize + sizeof(scratch_mem) + sizeof(gpu::Globals);
                    
    double alloc_time1 = getSeconds();

    //if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "USM allocation time " << 1000 * (alloc_time1 - alloc_time0) << " ms for " << (double)totalUSMAllocations / (1024*1024) << " MBs " << std::endl;     	

    void* scenePtr = (void*) nullptr; // FIXME: hack to pass polymorphic class

    assert(sizeof( gpu::BVH2BuildRecord ) <= 48 );

    EventContextHW ectx;

    // ===========================          
    // ==== stoch allocations ====
    // ===========================
    
    BuildParametersHW params;
    parseBuildParams(params);
    params.SAHLogBlockShift = cfg_SAHLogBlockShift;
    params.maxLeafSizeTopLevel = params.deterministicClustering ? 1 : 8;
    params.maxLeafSizeBottomLevel = 8;

    BuildContextHW ctx;
    ctx.init(deviceGPU, numPrimitives, params.subsetSizeFrac);
    ctx.globals = globals;
    ctx.aabb = aabb;
    ctx.bvh_mem = bvh_mem;
    ctx.bvh2 = bvh2;
    ctx.primref_index = primref_index;
    ctx.primref_index0 = primref_index0;
    ctx.primref_index1 = primref_index1;
    ctx.scratch_mem = scratch_mem;
    ctx.mc0 = mc0;
    ctx.morton_codes[0] = morton_codes[0];
    ctx.morton_codes[1] = morton_codes[1];
    ctx.wgBuildState = wgBuildState;
    ctx.wgTaskState = wgTaskState;        

    //std::memcpy(ctx.sobolMatrix, sobol::Matrices::matrices, sizeof(uint)*sobol::Matrices::num_dimensions*sobol::Matrices::size);

    // === DUMMY KERNEL TO TRIGGER USM TRANSFER ===
    {	  
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) { //1
                                                    cgh.single_task([=]() {
                                                                      globals->init(bvh_mem,numPrimitives,node_data_start,leaf_data_start,totalSize,0,leaf_primitive_size,scenePtr);
                                                                    });
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);
    }	    

    double total0 = getSeconds();
    ectx.allBegin = total0;	
    
    // ======================          
    // ==== init globals ====
    // ======================

    double t1,t2,preprocess_time = 0.0f;
      
    {	  
      sycl::event queue_event =  gpu_queue.submit([&](sycl::handler &cgh) { //2
                                                    cgh.single_task([=]() {
                                                                      globals->init(bvh_mem,numPrimitives,node_data_start,leaf_data_start,totalSize,0,leaf_primitive_size,scenePtr);
                                                                      globals->leaf_mem_allocator_cur += leaf_primitive_size*numPrimitives;
                                                                      globals->bvh2_index_allocator = 1; // 0 == root
                                                                      globals->numBuildRecords = numPrimitives;

                                                                      globals->range_allocator = 1;
                                                                      globals->numBuildRecords_extended = 0;
                                                                      globals->bvh2_breathfirst_records_start = 0;
                                                                      globals->bvh2_breathfirst_records_end   = 1;                                                                       

                                                                    });
                                                  });
      ectx.globals = queue_event;
      gpu::waitOnQueueAndCatchException(gpu_queue);
    }	    

    
    // ==========================================          
    // ==== get centroid and geometry bounds ====
    // ==========================================
    
    ectx.compCentroid = computeCentroidGeometryBounds(gpu_queue, globals, aabb, primref_index, numPrimitives);

    preprocess_time += 1000 * (t2 - t1);

#if MORTON_SORT
    gpu::waitOnQueueAndCatchException(gpu_queue);
    ectx.sortMortonBegin = getSeconds();
    // ==============================          
    // ==== compute morton codes ====
    // ==============================

    ectx.compMortonCodes = computeMortonCodes3D(gpu_queue,globals,mc0,aabb,numPrimitives,preprocess_time);
    //ectx.compMortonCodes = computeMortonCodes64Bit(gpu_queue,globals,mc0,aabb,numPrimitives,preprocess_time);
      
    // ===========================          
    // ==== sort morton codes ====
    // ===========================
    
    double sort_time = 0.0;          
    const uint nextPowerOf2 =  1 << (32 - sycl::clz(numPrimitives) - 1);
    const uint sortWGs = max(min((int)nextPowerOf2/8192,(int)RADIX_SORT_MAX_NUM_DSS),1);        
    const uint start = 4, end = 8;
    for (uint i=start;i<end;i++) {
      auto events = gpu::sort_iteration_type<false,MCPrim>(gpu_queue, morton_codes[i%2], morton_codes[(i+1)%2], numPrimitives, scratch_mem, i, sort_time, sortWGs);
      ectx.sortMorton.push_back(events);
    }

    gpu::waitOnQueueAndCatchException(gpu_queue);
    ectx.sortMortonEnd = getSeconds();
#endif

    preprocess_time += 1000 * (t2 - t1);

    t1 = getSeconds();
    ectx.preEnd = getSeconds();

    if (!params.deterministicClustering) {

    // =================================================================================
    // =========================== STOCHASTIC BVH BUILD ================================
    // =================================================================================

    SubsetHW(gpu_queue, params, ctx, ectx);

    /*
    // === init primref_index ===
    {
      // for (uint i=0;i<ctx.subsetSize;i++)
      //   primref_index0[i] = ctx.bvhPrimitives[ctx.bvhPrimitivesScratch[i].primitiveNumber].primitiveNumber;
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //4
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(ctx.subsetSize,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint i     = item.get_global_id(0);
                                                        if (i < ctx.subsetSize)
                                                          primref_index0[i] = ctx.bvhPrimitives[ctx.bvhPrimitivesScratch[i].primitiveNumber].primitiveNumber;
                                                      });		  
                                                  });
      ectx.initPrimRefIdx = queue_event;
    }         
    */

    gpu::waitOnQueueAndCatchException(gpu_queue);
    ctx.subsetSize = globals->sched;
    globals->sched = 0;
    ectx.subsetEnd = getSeconds();

    }
    else {

    gpu::waitOnQueueAndCatchException(gpu_queue); 
    PRINT(numPrimitives);
    PRINT(globals->sched);

    const uint bits = 3*params.clusteringMortonBits;

    //for (int i = 0; i < numPrimitives; i++)
    //  std::cout << (ctx.mc0[i].getMCode() >> (64 - bits)) << ", ";
    //std::cout << std::endl;

    {
      const uint wgSize = 32;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //7
                                              const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(numPrimitives,wgSize)),sycl::range<1>(wgSize));                                                        
                                              cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                  const auto getChunkID = [&](MCPrim &mcprim) -> uint {
                                                    return mcprim.getMCode() >> (64 - bits);
                                                  };

                                                  uint i     = item.get_global_id(0);
                                                  const uint subgroupID = get_sub_group_local_id();
                                                  const uint subgroupSize = get_sub_group_size();

                                                  if (!(i < numPrimitives)) return;

                                                  bool newChunk = i != 0 && getChunkID(ctx.mc0[i - 1]) != getChunkID(ctx.mc0[i]);

                                                  if (i == 0 || newChunk)
                                                    gpu::atomic_add_global<uint>(&globals->sched, 1);

                                                  ctx.clusterIDX[i] = newChunk ? 1 : 0;
                                                });
                                            });
    }
    gpu::waitOnQueueAndCatchException(gpu_queue);
    PRINT(globals->sched);
    ctx.subsetSize = globals->sched;
    globals->sched = 0;
    PRINT(ctx.subsetSize);

    inclusive_prefix_sum<uint>(gpu_queue,(uint*)ctx.clusterIDX,numPrimitives,(uint*)scratch_mem);

    gpu::waitOnQueueAndCatchException(gpu_queue);

    {
      const uint wgSize = 32;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //7
                                              const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(ctx.subsetSize,wgSize)),sycl::range<1>(wgSize));                                                        
                                              cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                  uint i     = item.get_global_id(0);

                                                  if (!(i < ctx.subsetSize)) return;

                                                  ctx.primref_index0[i] = i;
                                                  ctx.leafPrimitiveRanges[i] = gpu::Range(~0, 0); 
                                                  ctx.subsetAABB[i].init();
                                                  ctx.centroidAABB[i].init();
                                                });
                                            });
    }
    gpu::waitOnQueueAndCatchException(gpu_queue);
    {
      const uint wgSize = 32;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //7
                                              const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(numPrimitives,wgSize)),sycl::range<1>(wgSize));                                                        
                                              cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                  const auto getChunkID = [&](const MCPrim &mcprim) -> uint {
                                                    return mcprim.getMCode() >> (64 - bits);
                                                  };

                                                  uint i     = item.get_global_id(0);

                                                  if (!(i < numPrimitives)) return;
                                                
                                                  const MCPrim mcprim = ctx.mc0[i];

                                                  bool first = i == 0 || getChunkID(ctx.mc0[i - 1]) != getChunkID(mcprim);
                                                  bool last = i == (numPrimitives - 1) || getChunkID(mcprim) != getChunkID(ctx.mc0[i + 1]);

                                                  auto idx = mcprim.getIndex();
                                                  auto aabb = ctx.aabb[idx];

                                                  gpu::AABB centroid(aabb.centroid());

                                                  uint c = ctx.clusterIDX[i];
                                                  if (first) ctx.leafPrimitiveRanges[c].start = i;
                                                  if (last) ctx.leafPrimitiveRanges[c].end = i + 1;
                                                  aabb.atomic_merge_global(ctx.subsetAABB[c]);
                                                  centroid.atomic_merge_global(ctx.centroidAABB[c]);
                                                });
                                            });
    }

    gpu::waitOnQueueAndCatchException(gpu_queue);

    }

    
    PRINT(ctx.subsetSize);
    if (false && params.deterministicClustering) {
      for (int i = 0; i < ctx.subsetSize; i++) {
        auto range = ctx.leafPrimitiveRanges[i];
        std::cout << ctx.primref_index0[i] << ", [" << range.start << ", " << range.end << ")" << std::endl;
      }

      size_t min = ~0;
      size_t max = 0;

      const uint bits = 3*params.clusteringMortonBits;
      uint prev = 0;
      for (int i = 0; i < ctx.subsetSize; i++) {
        const auto &range = ctx.leafPrimitiveRanges[i];
        CHECK2(prev == range.start);
        CHECK2(range.start < range.end)
        const uint chunk = ctx.mc0[range.start].getMCode() >> (64 - bits);
        auto size = range.end - range.start;
        if (size > max) max = size;
        if (size < min) min = size;
        for (int j = range.start + 1; j < range.end; j++) {
          const uint ochunk = ctx.mc0[j].getMCode() >> (64 - bits);
          CHECK2(chunk == ochunk);

          CHECK2(ctx.subsetAABB[i].encloses(ctx.aabb[ctx.mc0[j].getIndex()]));
          CHECK2(ctx.centroidAABB[i].encloses(gpu::AABB(ctx.aabb[ctx.mc0[j].getIndex()].centroid())));
        }
        prev = range.end;
      }
      PRINT2(max, min);
      CHECK2(prev == ctx.size);
    }

    PRINT(globals->bvh2_index_allocator);
    
    // === Build top level BVH over subset ===
    
    BuildTopSAHHW(gpu_queue, params, ctx, ectx);

    const uint numTopLevelNodes = ctx.globals->bvh2_index_allocator;
    PRINT(numTopLevelNodes);        

    if (!params.deterministicClustering)
    {

    // ==============================        
    // === initialize nPrimitives ===
    // ==============================
    
#if 1        
    {
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //7
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(numTopLevelNodes,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint i     = item.get_global_id(0);
                                                        if (i < numTopLevelNodes)
                                                          ctx.leafPrimitiveRanges[i] = gpu::Range(0,0);   
                                                      });		  
                                                  });
      ectx.clusterInitNPrimitives = queue_event;
    }
#else        
    gpu::waitOnQueueAndCatchException(gpu_queue);
    for (int i = 0; i < numTopLevelNodes; i++)
      ctx.leafPrimitiveRanges[i] = gpu::Range(0,0);
#endif
    
#if 1
    {
      const uint wgSize = 32;
      const sycl::nd_range<1> nd_range1(sycl::range<1>(gpu::alignTo(numTopLevelNodes,wgSize)),sycl::range<1>(wgSize)); 
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //8
                                                    cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32)
                                                                    {
#if STABLE_ORDER && COHERENT_SUBSET                                                                                                                  
                                                                      const gpu::AABB *const aabb  = ctx.subsetAABB;
#else
                                                                      const gpu::AABB *const aabb  = ctx.aabb;
#endif

                                                                      const uint nodeID     = item.get_global_id(0);
                                                                      if (nodeID < numTopLevelNodes && bvh2[nodeID].isLeaf())
                                                                      {
                                                                        gpu::BVH2BuildRecord &leaf = bvh2[nodeID];                                                                        
                                                                        
                                                                        gpu::AABB bounds;
                                                                        bounds.init();
                                                                        for (uint i=leaf.start;i<leaf.end;i++)
                                                                          bounds.extend(aabb[primref_index0[i]]);

                                                                        leaf.bounds = convert_AABB3f(bounds);
                                                                      
                                                                        sycl::atomic_fence(sycl::memory_order::release,sycl::memory_scope::device);                                                                                
                                                                      
                                                                        uint ID = leaf.parent & 0x7fffffff; 
        
                                                                        while(ID != 0x7fffffff)
                                                                        {
                                                                          gpu::BVH2BuildRecord &current = bvh2[ID];              
                                                                          sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> ap(current.parent);
                                                                          const uint previous = ap.fetch_xor(0x80000000);
                                                                          if ((previous & 0x80000000) == 0) break;

                                                                          const gpu::BVH2BuildRecord &l = bvh2[current.left];
                                                                          const gpu::BVH2BuildRecord &r = bvh2[current.right];

                                                                          gpu::AABB3f node_bounds;                                                                              
                                                                          node_bounds.init();
                                                                          node_bounds.extend(l.bounds);
                                                                          node_bounds.extend(r.bounds);
                                                                          
                                                                          current.bounds = node_bounds;

                                                                          sycl::atomic_fence(sycl::memory_order::release,sycl::memory_scope::device);                                                                                
                                                                          
                                                                          ID = current.parent  & 0x7fffffff;
                                                                        }
                                                                      }
                                                                    });
                                                    
                                                  });
      ectx.clusterTopRefit = queue_event;
    }
#else
    gpu::waitOnQueueAndCatchException(gpu_queue);        
    refitGeometryBoundsCalculateSAHHW(bvh2,0,primref_index,aabb);
#endif

    // =================================================================================                
    // write node selection of subset primitives into original index
    // prim.primitiveNumber is where the subset primitive resides in ctx.bvhPrimitives
    // parallel for each leaf node?
    // ===============================================================================        

#if 1
    {
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //9
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(ctx.size,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                        const uint i     = item.get_global_id(0);
                                                        if (i < ctx.size)
                                                          ctx.nodeSelections[i] = -1;   
                                                      });		  
                                                  });
      ectx.clusterInitNodeSelection = queue_event;
    }
#else        
    gpu::waitOnQueueAndCatchException(gpu_queue);      
    for (int i = 0; i < ctx.size; i++) ctx.nodeSelections[i] = -1;
#endif        


#if 1
    {
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //10
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(numTopLevelNodes,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint i     = item.get_global_id(0);
                                                        if (i < numTopLevelNodes)
                                                        {
                                                          if (bvh2[i].isLeaf())
                                                          {
                                                            for (uint j=bvh2[i].start;j<bvh2[i].end;j++)
                                                            {
                                                              const uint index = primref_index0[j];
#if STABLE_ORDER && COHERENT_SUBSET
                                                              ctx.subsetNodeSelections[index] = i;
#else
                                                              ctx.nodeSelections[index] = i;
#endif
                                                            }
                                                            ctx.leafPrimitiveRanges[i].start+=bvh2[i].size();
                                                          }
                                                        }
                                                      });		  
                                                  });
      ectx.clusterWriteSubsetPrimSelection = queue_event;
    }        
#else        
    gpu::waitOnQueueAndCatchException(gpu_queue);      
    for (uint i=0;i<numTopLevelNodes;i++)
      if (bvh2[i].isLeaf())
      {
        for (uint j=bvh2[i].start;j<bvh2[i].end;j++)
        {
          const uint index = primref_index0[j];
          ctx.nodeSelections[index] = i;
        }
        ctx.leafPrimitiveRanges[i].start+=bvh2[i].size();
      }
#endif        

#if MORTON_SORT
#if !STABLE_ORDER
    // ====================================================================================================
    // exclusive prefix sum over indicator function of primitives (whether primitive is a subset primitive)
    // needed by the morton search
    // ====================================================================================================

#if 1
    {
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //11
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(ctx.size,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                        const uint i = item.get_global_id(0);
                                                        if (i < ctx.size)
                                                          ctx.closestSubsetPrimitive[i] = ctx.subsetSelection[i];
                                                      });		  
                                                  });
      ectx.clusterWriteClosestSubsetPrimitive = queue_event;
    }        
    
    auto events = exclusive_prefix_sum2<uint>(gpu_queue,(uint*)ctx.closestSubsetPrimitive,ctx.size,(uint*)scratch_mem);
    ectx.clusterMortonWindowPSumLocal = events.first;
    ectx.clusterMortonWindowPSumGlobal = events.second;
#else        
    gpu::waitOnQueueAndCatchException(gpu_queue);      
    {
      int j = 0;
      for (int i = 0; i < ctx.size; i++) {
        ctx.closestSubsetPrimitive[i] = j;
        if (ctx.subsetSelection[i] != 0) j++;
      }
    }
#endif
#endif


    // ==================================================        
    // write subset node selections into compacted array
    // parallel for each primitive
    // ==================================================

#if !(STABLE_ORDER && COHERENT_SUBSET)
#if 1
    {
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //12
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(ctx.size,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                        const uint i = item.get_global_id(0);
                                                        if (i < ctx.size)
                                                        {
                                                          if (ctx.subsetSelection[i] != 0) {
                                                            int j = ctx.closestSubsetPrimitive[i];
                                                            ctx.subsetNodeSelections[j] = ctx.nodeSelections[ctx.mc0[i].getIndex()];
                                                          }                                                             
                                                        }
                                                      });		  
                                                  });
      ectx.clusterWriteSubsetNodeSelection = queue_event;
    }        
#else        
    gpu::waitOnQueueAndCatchException(gpu_queue);      
    for (int i = 0; i < ctx.size; i++) {
      if (ctx.subsetSelection[i] != 0) {
        int j = ctx.closestSubsetPrimitive[i] - 1;
        ctx.subsetNodeSelections[j] = ctx.nodeSelections[ctx.bvhPrimitives[i].primitiveNumber];
      }
    }
#endif
#endif
#endif

    // ===========================
    // insert primitives into BVH
    // ===========================
    
#if 1
    {
      const uint wgSize = 16;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //13
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(ctx.size,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint i = item.get_global_id(0);
                                                        if (i < ctx.size)
                                                        {
                                                          int subsetPrimitiveIdx = ctx.closestSubsetPrimitive[i];
#if MORTON_SORT
                                                          const uint primID = ctx.mc0[i].getIndex();
#else
                                                          const uint primID = i;
#endif
                                                          if (ctx.subsetSelection[i] == 0)
                                                          {
#if MORTON_SORT
                                                          uint nodeID = FindBestNode(params, ctx, i, primID, aabb, bvh2);
#else
                                                          uint nodeID = FindBestNode3(params, ctx, i, aabb, bvh2);
#endif
                                                          ctx.nodeSelections[primID] = nodeID;              
                                                          //ctx.leafPrimitiveRanges[nodeID].start++;
                                                          gpu::atomic_add_global<uint>(&ctx.leafPrimitiveRanges[nodeID].start,1);
                                                          }
#if STABLE_ORDER && COHERENT_SUBSET
                                                          else {
                                                            auto selection = ctx.subsetNodeSelections[subsetPrimitiveIdx];
                                                            ctx.nodeSelections[primID] = selection;
                                                          }
#endif
                                                        }
                                                      });		  
                                                  });
      ectx.clusterFindBestNode = queue_event;
    }        
#else        
    gpu::waitOnQueueAndCatchException(gpu_queue);
    for (int i = 0; i < ctx.size; i++) {
      {
        if (ctx.subsetSelection[i] != 0) continue;              
        uint nodeID = FindBestNode(params, &ctx, i, aabb, bvh2);
        //PRINT5(i,nodeID,bvh2[nodeID].isLeaf(),ctx.nodeSelections[i],ctx.subsetSelection[i]);
        ctx.nodeSelections[ctx.bvhPrimitives[i].primitiveNumber] = nodeID;              
        ctx.leafPrimitiveRanges[nodeID].start++;
      }
    }
#endif

    // ================================================================================================        
    // exclusive prefix sum over primitive counts of leaf nodes (they are offsets after the prefix sum)
    // ================================================================================================

#if 1 // TODO
    {
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //14
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(numTopLevelNodes,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                        const uint i = item.get_global_id(0);
                                                        if (i < numTopLevelNodes)
                                                          primref_index1[i] = ctx.leafPrimitiveRanges[i].start;                                                           
                                                      });		  
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);      
      ectx.clusterWritePrimref = queue_event;
    }
    {
      auto events = exclusive_prefix_sum2<uint>(gpu_queue,primref_index1,numTopLevelNodes,(uint*)scratch_mem);
      ectx.clusterReorderPSumLocal = events.first;
      ectx.clusterReorderPSumGlobal = events.second;
    {
    }
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //15
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(numTopLevelNodes,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                        const uint i = item.get_global_id(0);
                                                        if (i < numTopLevelNodes)
                                                        {
                                                          ctx.leafPrimitiveRanges[i].start = primref_index1[i];
                                                          ctx.leafPrimitiveRanges[i].end = primref_index1[i];                                                             
                                                        }
                                                      });		  
                                                  });
      gpu::waitOnQueueAndCatchException(gpu_queue);      
      ectx.clusterWriteLeafPrimitiveRanges = queue_event;
    }
#else        
    uint acc = 0;        
    for (uint i=0;i<numTopLevelNodes;i++)
    {
      //PRINT2(i,acc);
      const uint tmp = ctx.leafPrimitiveRanges[i].start;
      ctx.leafPrimitiveRanges[i].start = acc;
      ctx.leafPrimitiveRanges[i].end   = acc; // + tmp;          
      acc += tmp;
    }
#endif
    // =====================================        
    // reorder primitives into scratch space
    // =====================================

#if 1
    {
      const uint wgSize = 256;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //16
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(ctx.size,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32) {
                                                        const uint i = item.get_global_id(0);
                                                        if (i < ctx.size)
                                                        {
#if MORTON_SORT
                                                          uint primID = ctx.mc0[i].getIndex();
#else
                                                          uint primID = i;
#endif
                                                          uint nodeID = ctx.nodeSelections[primID];
                                                          uint endID = gpu::atomic_add_global<uint>(&ctx.leafPrimitiveRanges[nodeID].end,1);
                                                          primref_index0[endID] = primID;
                                                        }
                                                      });		  
                                                  });
      ectx.clusterReorder = queue_event;
    }               
#else        
    gpu::waitOnQueueAndCatchException(gpu_queue);      
    for (int i = 0; i < ctx.size; i++) {
      uint primID = ctx.bvhPrimitives[i].primitiveNumber;
      uint nodeID = ctx.nodeSelections[primID];
      primref_index0[ctx.leafPrimitiveRanges[nodeID].end++] = primID;
    }
#endif

    // for (uint i=0;i<numTopLevelNodes;i++)
    //   if (ctx.leafPrimitiveRanges[i].size() > 0)
    //   {
    //     PRINT5(i,ctx.leafPrimitiveRanges[i].start,ctx.leafPrimitiveRanges[i].end,ctx.leafPrimitiveRanges[i].size(),bvh2[i].isLeaf());
        // for (uint j=ctx.leafPrimitiveRanges[i].start;j<ctx.leafPrimitiveRanges[i].end;j++)
        //   PRINT2(j,primref_index[j]);
      // }

    // ===========================================================================        
    // === recompute centroid bounds per range and reset buildrecord/bvh2 node ===
    // ===========================================================================

#if 1
    {
      const uint wgSize = 16;
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //17
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(numTopLevelNodes,wgSize)),sycl::range<1>(wgSize));                                                        
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint i = item.get_global_id(0);
                                                        if (i < numTopLevelNodes)
                                                        {
                                                          if (ctx.leafPrimitiveRanges[i].size() > 0)
                                                          {
                                                            gpu::BVH2BuildRecord &current = bvh2[i];
                                                            assert(current.isLeaf());
        
                                                            current.start  = ctx.leafPrimitiveRanges[i].start;
                                                            current.end    = ctx.leafPrimitiveRanges[i].end;
                                                            current.left   = -1;
                                                            current.right  = -1;

                                                            gpu::AABB3f bounds;
                                                            bounds.init();
                                                            for (uint j=current.start;j<current.end;j++)
                                                              bounds.extend(convert_AABB3f(aabb[primref_index0[j]]).centroid2());
                                                            current.bounds = bounds;
                                                          }          
                                                        }
                                                      });		  
                                                  });
      ectx.clusterUpdateCentroidBounds = queue_event;
    }               
#else        
    gpu::waitOnQueueAndCatchException(gpu_queue);      
    for (uint i=0;i<numTopLevelNodes;i++)
    {
      if (ctx.leafPrimitiveRanges[i].size() > 0)
      {
        gpu::BVH2BuildRecord &current = bvh2[i];
        assert(current.isLeaf());
        
        current.bounds.init();
        current.start  = ctx.leafPrimitiveRanges[i].start;
        current.end    = ctx.leafPrimitiveRanges[i].end;
        current.left   = -1;
        current.right  = -1;
        
        for (uint j=current.start;j<current.end;j++)
          current.bounds.extend(convert_AABB3f(aabb[primref_index0[j]]).centroid2());
      }          
    }        
#endif

    gpu::waitOnQueueAndCatchException(gpu_queue);    
    ectx.clusterEnd = getSeconds();

    if (false) {
      std::unordered_map<size_t, size_t> hist;
      for (int i = 0; i < numTopLevelNodes; i++) {
        const auto &node = bvh2[i];
        if (!node.isLeaf()) continue;
        hist.insert({node.end - node.start, 0}).first->second++;
      }
      size_t max = 0;
      for (const auto &kv : hist)
        max = std::max(max, kv.first);
      for (size_t i = 1; i < max + 1; i++) {
        auto it = hist.find(i);
        std::cout << i << ": " << ((it == hist.end()) ? 0 : it->second) << std::endl;
      }
    }

    }
    else
    {
      if (false)
      for (uint i = 0; i < numTopLevelNodes; i++) {
        gpu::BVH2BuildRecord &current = ctx.bvh2[i];
        if (!current.isLeaf()) continue;
        CHECK2(current.size() == 1);
      }

      {
        const uint wgSize = 16;
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //17
                                                      const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(numTopLevelNodes,wgSize)),sycl::range<1>(wgSize));                                                        
                                                      cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                          const uint i = item.get_global_id(0);
                                                          if (!(i < numTopLevelNodes)) return;
                                                          gpu::BVH2BuildRecord &current = ctx.bvh2[i];
                                                          if (!current.isLeaf()) return;
                                                          auto index = ctx.primref_index0[current.start];
                                                          auto range = ctx.leafPrimitiveRanges[index];

                                                          current.bounds = convert_AABB3f(ctx.centroidAABB[index]);
                                                          current.start = range.start;
                                                          current.end = range.end;
                                                          current.left = -1;
                                                          current.right = -1;
                                                        });		  
                                                    });
        ectx.clusterReorder = queue_event;
      }    

      gpu::waitOnQueueAndCatchException(gpu_queue);              
      {
        const uint wgSize = 16;
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //17
                                                      const sycl::nd_range<1> nd_range(sycl::range<1>(gpu::alignTo(numPrimitives,wgSize)),sycl::range<1>(wgSize));                                                        
                                                      cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                          const uint i = item.get_global_id(0);
                                                          if (!(i < numPrimitives)) return;

                                                          uint idx = ctx.mc0[i].getIndex();
                                                          ctx.primref_index0[i] = idx;
                                                        });		  
                                                    });
        ectx.clusterReorder = queue_event;
      }  

      gpu::waitOnQueueAndCatchException(gpu_queue);              
    }

    // ============================================        
    // === build BVH2 over all top-level ranges ===
    // ============================================

    //globals->sched = 0;
    //globals->numWGs = 0;
    //globals->sync = 0;

    resetPhistStochHW(gpu_queue, ctx.phist);

    {
      const uint sizeWG = 16;        
      assert(numPrimitives > cfg_maxLeafSize_BottomLevel);
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //18
                                                    const sycl::nd_range<1> nd_range(sycl::range<1>(numTopLevelNodes*sizeWG),sycl::range<1>(sizeWG));		  
                                                    /* local variables */
                                                    sycl::local_accessor< uint           , 0> _atomicCountLeft(cgh);
                                                    sycl::local_accessor< uint           , 0> _atomicCountRight(cgh);		  		  		  		  
                                                    sycl::local_accessor< uint           , 1> _stack(sycl::range<1>(128),cgh);
                                                                                  
                                                    cgh.parallel_for(nd_range,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16) {
                                                        const uint localID      = item.get_local_id(0);                                                         
                                                        const uint groupID      = item.get_group(0);
                                                        uint &atomicCountLeft   = *_atomicCountLeft.get_pointer();                                                         
                                                        uint &atomicCountRight  = *_atomicCountRight.get_pointer();
                                                        uint *stack             =  _stack.get_pointer();
                                                                                                                
                                                        const gpu::AABB *const primref  = aabb;                                                         
                                                        const uint recordID = groupID;                                                         
                                                        
                                                        if (bvh2[recordID].isLeaf() && bvh2[recordID].size() > cfg_maxLeafSize_BottomLevel)
                                                        {
                                                          //if (localID == 0) gpu::atomic_add_global<uint>(&globals->sched,1);
                                                          uint sindex = 1;
                                                          stack[0] = recordID;

                                                          //uint prims = 0;
                                                          //uint its = 0;
                                                          while(sindex)
                                                          {
                                                            //prims += bvh2[recordID].size();
                                                            //its++;
                                                            sindex--;                                                               
                                                            const uint ID = stack[sindex];
                                                            gpu::BVH2BuildRecord &current = bvh2[ID];
                                                            if (localID == 0)
                                                            writePHistStochHW(ctx.phist, ctx.bvh2, ID);
                                                            
                                                            const uint parentID = ID;
                                                            const uint startID = current.start;
                                                            const uint endID   = current.end;
                                                            
                                                            gpu::AABB3f leftBounds, rightBounds;
                                                            bin_partition_register(item,current,atomicCountLeft,atomicCountRight,leftBounds,rightBounds,primref,primref_index0,primref_index1,cfg_SAHLogBlockShift,cfg_maxLeafSize_BottomLevel);

                                                            /* split position */
                                                            const uint split_position = startID + atomicCountLeft;

                                                            /* allocate new bvh2 node */
                                                            uint bvh2_index = 0; 
                                                            if (localID == 0)
                                                            {
                                                              sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> bvh2_counter(globals->bvh2_index_allocator);
                                                              bvh2_index = bvh2_counter.fetch_add(2);
                                                            }
                                                            bvh2_index = sub_group_broadcast(bvh2_index,0);
                                                              
                                                            /* write out two new bvh2 build records */
                                                            current.left  = bvh2_index+0;
                                                            current.right = bvh2_index+1;                                                                                                                          
                                                            bvh2[bvh2_index+0].init(startID,split_position,parentID,leftBounds);
                                                            bvh2[bvh2_index+1].init(split_position,endID  ,parentID,rightBounds);
                                                            
                                                            /* left child non leaf? */
                                                            if (split_position - startID > cfg_maxLeafSize_BottomLevel)
                                                              stack[sindex++] = bvh2_index+0;
                                                            
                                                            /* right child non leaf? */                                                               
                                                            if (endID - split_position  > cfg_maxLeafSize_BottomLevel)
                                                              stack[sindex++] = bvh2_index+1;                                                               
                                                          }                                                             
                                                        
                                                          //if (localID == 0) gpu::atomic_add_global<uint>(&globals->numWGs,its);
                                                          //if (localID == 0) gpu::atomic_add_global<uint>(&globals->sync,prims);
                                                        }
                                                                                                                                                                        

                                                      });		  
                                                  });
      ectx.bottom = queue_event;
    }

    printresetPhistStochHW("c", gpu_queue, ctx.phist);

    gpu::waitOnQueueAndCatchException(gpu_queue);        
    ectx.bottomEnd = getSeconds();
    const uint numBVH2Nodes = globals->bvh2_index_allocator;

    /*{
      uint initial = globals->sched;
      uint processed = globals->numWGs;
      float ratio = (float)processed/(float)initial;
      uint prims = globals->sync;
      PRINT4(initial, processed, ratio, prims);
      globals->sched = 0;
      globals->numWGs = 0;
    }*/
    PRINT(numBVH2Nodes);



    /* --- bottom-up BVH2 refit --- */
#if 0

    refitGeometryBoundsCalculateSAHHW(bvh2,0,primref_index,aabb); // FIXME needs GPU refit variant

#else        
    {
      const uint wgSize = 32;
      const sycl::nd_range<1> nd_range1(sycl::range<1>(gpu::alignTo(numBVH2Nodes,wgSize)),sycl::range<1>(wgSize)); 
      sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) { //19
                                                    cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(32)
                                                                    {
                                                                      const uint nodeID     = item.get_global_id(0);
                                                                      if (nodeID < numBVH2Nodes && bvh2[nodeID].isLeaf())
                                                                      {
                                                                        gpu::BVH2BuildRecord &leaf = bvh2[nodeID];                                                                        
                                                                        
                                                                        gpu::AABB bounds;
                                                                        bounds.init();
                                                                        for (uint i=leaf.start;i<leaf.end;i++)
                                                                          bounds.extend(aabb[primref_index0[i]]);

                                                                        leaf.bounds = convert_AABB3f(bounds);
                                                                      
                                                                        ctx.leafPrimitiveRanges[nodeID].start = leaf.size();

                                                                        sycl::atomic_fence(sycl::memory_order::release,sycl::memory_scope::device);                                                                                
                                                                      
                                                                        uint ID = leaf.parent & 0x7fffffff; 
        
                                                                        while(ID != 0x7fffffff)
                                                                        {
                                                                          gpu::BVH2BuildRecord &current = bvh2[ID];              
                                                                          sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::device,sycl::access::address_space::global_space> ap(current.parent);
                                                                          const uint previous = ap.fetch_xor(0x80000000);
                                                                          if ((previous & 0x80000000) == 0) break;

                                                                          gpu::AABB3f node_bounds;                                                                              
                                                                          node_bounds.init();
                                                                          node_bounds.extend(bvh2[current.left].bounds);
                                                                          node_bounds.extend(bvh2[current.right].bounds);
                                                                            
                                                                          current.bounds = node_bounds;
                                                                          
                                                                          ctx.leafPrimitiveRanges[ID].start = 
                                                                            ctx.leafPrimitiveRanges[current.left].start +
                                                                            ctx.leafPrimitiveRanges[current.right].start;

                                                                          sycl::atomic_fence(sycl::memory_order::release,sycl::memory_scope::device);                                                                                
                                                                          
                                                                          ID = current.parent  & 0x7fffffff;
                                                                        }
                                                                        
                                                                      }
                                                                    });
                                                    
                                                  });
      ectx.refit = queue_event;
    }
#endif
    gpu::waitOnQueueAndCatchException(gpu_queue);        
    double total1 = getSeconds();
    ectx.refitEnd = total1;
    double total_diff = total1 - total0;
    PRINT( (float)(total1-total0)*1000.0f);

    // =================================================================================
    // ================================= CHECK BVH2 ====================================
    // =================================================================================
    
    uint nodes = 0;
    uint leaves = 0;
    double nodeSAH = 0;
    double leafSAH = 0;      
    checkBVH2StochHW(bvh2,globals->rootIndex,primref_index,aabb,nodes,leaves,nodeSAH,leafSAH,cfg_maxLeafSize_BottomLevel);
    nodeSAH /= globals->geometryBounds.area();
    leafSAH /= globals->geometryBounds.area();                
    PRINT4(nodes,leaves,(float)nodeSAH,(float)leafSAH);
    
    //if (unlikely(deviceGPU->verbosity(2)))
      std::cout << "BVH GPU Stochastic Builder DONE in " << 1000.*total_diff << " ms : " << numPrimitives*0.000001f/total_diff << " MPrims/s " << std::endl << std::flush;
        
    PRINT2(globals->bvh2_index_allocator,numPrimitives);
        
    if( globals->bvh2_index_allocator >= 2*numPrimitives)
      FATAL("ALLOCATOR");

    /*
    // ==========================================================
    // ==========================================================

    alloc.clear();
    QBVH6::SizeEstimate sizeEstimate(node_size,leaf_size,0);
    PRINT(pinfo.geomBounds);
    PRINT(sizeEstimate);
    QBVH6* qbvh = new (accel.data()) QBVH6(sizeEstimate, 6);
    qbvh->numPrims = org_numPrimitives;
    qbvh->bounds = pinfo.geomBounds;

    PRINT(numPrimitives);
    PRINT(qbvh->sizeNodeMem());
    PRINT(qbvh->sizeNodeMem()/64);    
    PRINT(qbvh->sizeLeafMem());
    PRINT(qbvh->sizeLeafMem()/64);
    
    
    char* root_node = qbvh->allocNode(sizeof(QBVH6::InternalNode6));    
    qbvh->rootNodeOffset = QBVH6::Node((char*)(root_node - (char*)qbvh), NODE_TYPE_INTERNAL, 0);
    PRINT(qbvh->rootNodeOffset);
    
    //convertBVH2toQBVH6_Stoch(scene,qbvh,root_node,bvh2,ctx.leafPrimitiveRanges,globals->rootIndex,primref_index,prims.data(),nullptr,settings2);
    //convertBVH2toQBVH6_new(scene,qbvh,root_node,bvh2,globals->rootIndex,bvh2_subtree_size,prims.data(),settings,numPrimitives);

    qbvh->computeStatistics();
    BVHStatistics::instance.print(std::cout);
    qbvh->print();

    
    if (deviceGPU) {
      HWAccel* hwaccel = (HWAccel*) accel.data();
      hwaccel->dispatchGlobalsPtr = (uint64_t) deviceGPU->dispatchGlobalsPtr;
    }
    
    return pinfo.geomBounds;
    */
  }

  /************************************************************************************/
  /************************************************************************************/
  /************************************************************************************/
  /************************************************************************************/



  /************************************************************************************/
  /************************************************************************************/
  /************************************************************************************/
  /************************************************************************************/    
}  
#endif