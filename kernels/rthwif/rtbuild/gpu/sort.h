// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(EMBREE_SYCL_SUPPORT)
#include "common.h"

#define RADIX_SORT_BINS        256    
#define RADIX_SORT_MAX_NUM_DSS 128
#define RADIX_SORT_WG_SIZE     512
#define RADIX_ITERATIONS_64BIT 8

namespace embree
{
  namespace gpu
  {

    __forceinline unsigned int getNumWGsScratchSize(const ssize_t scratch_size)
    {
      return min(max( (int)floorf(scratch_size / (sizeof(unsigned int)*RADIX_SORT_BINS))-1,(int)1),RADIX_SORT_MAX_NUM_DSS);
    }
    
    __forceinline void localAtomicBallot(unsigned int *const histogram, const unsigned int ID, const unsigned int add)
    {
      unsigned int mask = sub_group_ballot(true);
      while(mask)
      {
        const unsigned int first = sycl::ctz(mask);
        const unsigned int index = sub_group_broadcast(ID,first);
        const bool cmp = ID == index;
        const unsigned int cmp_mask = sub_group_ballot(cmp);
        const unsigned int reduced_count = sycl::popcount(cmp_mask) * add;
        mask &= ~cmp_mask;
        
        sycl::atomic_ref<unsigned int, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> hist(histogram[ID]);                                                                                    
        if (get_sub_group_local_id() == first)
          hist += reduced_count;
      }        
    }
    

    __forceinline void radix_sort_single_workgroup(sycl::queue &gpu_queue, uint64_t *_input, uint64_t *_output, const unsigned int numPrimitives,  const unsigned int start_iteration, const unsigned int stop_iteration)
    {
      static const unsigned int RADIX_SORT_SINGLE_WG_SIZE = 256; 

      struct __aligned(RADIX_SORT_SINGLE_WG_SIZE/32 * sizeof(unsigned int)) BinFlags
      {
        unsigned int flags[RADIX_SORT_SINGLE_WG_SIZE/32];
      };
    
      const sycl::nd_range<1> nd_range1(RADIX_SORT_SINGLE_WG_SIZE,sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE));          
      sycl::event event = gpu_queue.submit([&](sycl::handler &cgh) {
                                             sycl::local_accessor< unsigned int, 1> histogram(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                             sycl::local_accessor< unsigned int, 1> sums(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                             sycl::local_accessor< unsigned int, 1> prefix_sums(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                             sycl::local_accessor< unsigned int, 1> local_offset(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                             sycl::local_accessor< BinFlags, 1> bin_flags(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                   
                                             cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)                                                                    
                                                              {                                                                                                                                            
                                                                const unsigned int localID     = item.get_local_id(0);
                                                                const unsigned int step_local  = item.get_local_range().size();
                                                                const unsigned int subgroupID      = get_sub_group_id();
                                                                const unsigned int subgroupSize    = get_sub_group_size();
                                                                const unsigned int subgroupLocalID = get_sub_group_local_id();

                                                                for (unsigned int iter = start_iteration;iter<stop_iteration;iter++)
                                                                {
                                                                  item.barrier(sycl::access::fence_space::local_space);
                                                                          
                                                                  const unsigned int shift = iter*8;

                                                                  const uint64_t *const input  = (iter % 2) == 0 ? _input : _output;
                                                                  uint64_t *const output = (iter % 2) == 1 ? _input : _output;
                                                                        
                                                                  // ==== bin key into global histogram =====
                                                                      
                                                                  if (localID < RADIX_SORT_BINS)
                                                                    histogram[localID] = 0;

                                                                  item.barrier(sycl::access::fence_space::local_space);
                                                                    
                                                                  for (unsigned int ID = localID; ID < numPrimitives; ID += step_local)
                                                                  {
                                                                    const unsigned int bin = ((unsigned int)(input[ID] >> shift)) & (RADIX_SORT_BINS - 1);
                                                                    //localAtomicBallot(histogram.get_pointer(),bin,1);
                                                                    gpu::atomic_add_local((unsigned int*)histogram.get_pointer() + bin,(unsigned int)1);
                                                                  }

                                                                  item.barrier(sycl::access::fence_space::local_space);

                                                                  // ==== reduce global histogram =====    
                                                                                                                                              
                                                                  SYCL_EXT_ONEAPI::sub_group sub_group = this_sub_group();
                                                                  sub_group.barrier();

                                                                  if (localID < RADIX_SORT_BINS)
                                                                  {
                                                                    const unsigned int count = histogram[localID];
                                                                    const unsigned int sum = sub_group_reduce(count, std::plus<unsigned int>());
                                                                    const unsigned int prefix_sum = sub_group_exclusive_scan(count, std::plus<unsigned int>());

                                                                    sums[localID] = sum;
                                                                    prefix_sums[localID] = prefix_sum;
                                                                  }
                                                                        
                                                                  item.barrier(sycl::access::fence_space::local_space);


                                                                  if (subgroupID == 0)
                                                                  {
                                                                    unsigned int off = 0;
                                                                    for (int i = subgroupLocalID; i < RADIX_SORT_BINS; i += subgroupSize)
                                                                    {
                                                                      local_offset[i] = off + prefix_sums[i];
                                                                      off += sums[i];
                                                                    }
                                                                  }

                                                                  item.barrier(sycl::access::fence_space::local_space);                                                                          
                                                                      
                                                                  // ==== scatter key/value pairs according to global historgram =====      
                                                                                                                         
                                                                  const unsigned int flags_bin = localID / 32;
                                                                  const unsigned int flags_bit = 1 << (localID % 32);                                                                      
                                                                      
                                                                  for (unsigned int blockID = 0; blockID < numPrimitives; blockID += step_local)
                                                                  {
                                                                    item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                    const unsigned int ID = blockID + localID;                                                                            
                                                                        
                                                                    unsigned int binID = 0;
                                                                    unsigned int binOffset = 0;

                                                                    if (localID < RADIX_SORT_BINS)                                                                            
                                                                      for (int i=0;i<RADIX_SORT_SINGLE_WG_SIZE/32;i++)
                                                                        bin_flags[localID].flags[i] = 0;

                                                                    item.barrier(sycl::access::fence_space::local_space);
                                                                                 
                                                                            
                                                                    uint64_t in;
                                                                    if (ID < numPrimitives)
                                                                    {
                                                                      in = input[ID];                                                                              
                                                                      binID = (in >> shift) & (RADIX_SORT_BINS - 1);                                                                            
                                                                      binOffset = local_offset[binID];
                                                                              
                                                                      sycl::atomic_ref<unsigned int, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> bflags(bin_flags[binID].flags[flags_bin]);                                                                                    
                                                                      bflags.fetch_add(flags_bit);

                                                                    }

                                                                          
                                                                    item.barrier(sycl::access::fence_space::local_space);
                                                                            
                                                                    if (ID < numPrimitives)
                                                                    {
                                                                      unsigned int prefix = 0;
                                                                      unsigned int count = 0;
                                                                      for (unsigned int i = 0; i < RADIX_SORT_SINGLE_WG_SIZE / 32; i++)
                                                                      {
                                                                        const unsigned int bits = bin_flags[binID].flags[i];
                                                                        const unsigned int full_count    = sycl::popcount(bits);
                                                                        const unsigned int partial_count = sycl::popcount(bits & (flags_bit - 1));
                                                                        prefix += (i  < flags_bin) ? full_count : 0;
                                                                        prefix += (i == flags_bin) ? partial_count : 0;
                                                                        count += full_count;
                                                                      }

                                                                      output[binOffset + prefix] = in;
                                                                      if (prefix == count - 1)
                                                                        local_offset[binID] += count;                                                                          
                                                                    }
                                                                          
                                                                  }                                                                       
                                                                }
                                                              });
                                                   
                                           });

                
      try {
        event.wait_and_throw();
      } catch (sycl::exception const& e) {
        std::cout << "Caught synchronous SYCL exception:\n"
                  << e.what() << std::endl;
        FATAL("SYCL Exception");     		
      }
    }

    
    template<typename sort_type>
    sycl::event radix_sort_iteration_type(sycl::queue &gpu_queue, sycl::event &input_event, sort_type *input, sort_type *output, const unsigned int primitives,  unsigned int *global_histogram, const unsigned int iter, const unsigned int RADIX_SORT_NUM_DSS=256)
    {
      const unsigned int shift = iter*8;

      
      // ==== bin key into global histogram =====
      
      const sycl::nd_range<1> nd_range_binning(sycl::range<1>(RADIX_SORT_WG_SIZE*RADIX_SORT_NUM_DSS),sycl::range<1>(RADIX_SORT_WG_SIZE));          
      sycl::event bin_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                 cgh.depends_on(input_event);
                                                 sycl::local_accessor< unsigned int, 1> histogram(sycl::range<1>(RADIX_SORT_BINS),cgh);                                                 
                                                 cgh.parallel_for(nd_range_binning,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                  {
                                                                    const unsigned int localID     = item.get_local_id(0);
                                                                    const unsigned int step_local  = item.get_local_range().size();
                                                                    const unsigned int groupID     = item.get_group(0);

                                                                    const unsigned int startID = ((size_t)groupID + 0)*(size_t)primitives / RADIX_SORT_NUM_DSS;
                                                                    const unsigned int endID   = ((size_t)groupID + 1)*(size_t)primitives / RADIX_SORT_NUM_DSS;
                                                                                                                                                                                                            
                                                                    if (localID < RADIX_SORT_BINS)
                                                                      histogram[localID] = 0;

                                                                    item.barrier(sycl::access::fence_space::local_space);
                                                                    
                                                                    for (unsigned int ID = startID + localID; ID < endID; ID += step_local)
                                                                    {
                                                                      const uint64_t key = input[ID];
                                                                      const unsigned int bin = ((unsigned int)(key >> shift)) & (RADIX_SORT_BINS - 1);
                                                                      //gpu::localAtomicBallot(histogram.get_pointer(),bin,1);
                                                                      gpu::atomic_add_local((unsigned int*)histogram.get_pointer() + bin,(unsigned int)1);
                                                                          
                                                                    }

                                                                    item.barrier(sycl::access::fence_space::local_space);
    
                                                                    if (localID < RADIX_SORT_BINS)
                                                                      global_histogram[RADIX_SORT_BINS*groupID + localID] = histogram[localID];
                                                                    
                                                                  });
                                                 
                                               });
            
      // ==== scatter key/value pairs according to global historgram =====
        
      struct __aligned(64) BinFlags
      {
        unsigned int flags[RADIX_SORT_WG_SIZE/32];
      };
      
      const sycl::nd_range<1> nd_range_scatter(sycl::range<1>(RADIX_SORT_WG_SIZE*RADIX_SORT_NUM_DSS),sycl::range<1>(RADIX_SORT_WG_SIZE));          
      sycl::event scatter_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     cgh.depends_on(bin_event);
                                                     sycl::local_accessor< unsigned int, 1> local_offset(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     sycl::local_accessor< unsigned int, 1> sums(sycl::range<1>(RADIX_SORT_BINS),cgh);                                                     
                                                     sycl::local_accessor< BinFlags, 1> bin_flags(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     cgh.parallel_for(nd_range_scatter,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const unsigned int groupID     = item.get_group(0);
                                                                        const unsigned int localID     = item.get_local_id(0);
                                                                        const unsigned int step_local  = item.get_local_range().size();
                                                                        const unsigned int subgroupID      = get_sub_group_id();
                                                                        const unsigned int subgroupLocalID = get_sub_group_local_id();
                                                                        

                                                                        const unsigned int startID = ((size_t)groupID + 0)*(size_t)primitives / RADIX_SORT_NUM_DSS;
                                                                        const unsigned int endID   = ((size_t)groupID + 1)*(size_t)primitives / RADIX_SORT_NUM_DSS;

                                                                        
                                                                        /* --- reduce global histogram --- */
                                                                        unsigned int local_hist = 0;
                                                                        unsigned int prefix_sum = 0;
                                                                        
                                                                        if (localID < RADIX_SORT_BINS)
                                                                        {
                                                                          unsigned int t = 0;
                                                                          for (unsigned int j = 0; j < RADIX_SORT_NUM_DSS; j++)
                                                                          {
                                                                            const unsigned int count = global_histogram[RADIX_SORT_BINS*j + localID];
                                                                            local_hist = (j == groupID) ? t : local_hist;
                                                                            t += count;
                                                                          }
                                                                        
                                                                          const unsigned int count = t;
                                                                          const unsigned int sum = sub_group_reduce(count, std::plus<unsigned int>());
                                                                          prefix_sum = sub_group_exclusive_scan(count, std::plus<unsigned int>());

                                                                          sums[subgroupID] = sum;
                                                                        }
                                                                      
                                                                        item.barrier(sycl::access::fence_space::local_space);

                                                                        if (localID < RADIX_SORT_BINS)
                                                                        {                                                                        
                                                                          const unsigned int sums_prefix_sum = sub_group_broadcast(sub_group_exclusive_scan(sums[subgroupLocalID], std::plus<unsigned int>()),subgroupID);
                                                                          const unsigned int global_hist = sums_prefix_sum + prefix_sum;
                                                                          local_offset[localID] = global_hist + local_hist;
                                                                        }

                                                                        // === barrier comes later ===
                                                                        
                                                                        const unsigned int flags_bin = localID / 32;
                                                                        const unsigned int flags_bit = 1 << (localID % 32);                                                                      
                                                                      
                                                                        for (unsigned int blockID = startID; blockID < endID; blockID += step_local)
                                                                        {
                                                                        
                                                                          const unsigned int ID = blockID + localID;
                                                                        
                                                                          unsigned int binID = 0;
                                                                          unsigned int binOffset = 0;

#if 0                                                                          
                                                                          if (localID < RADIX_SORT_BINS)
                                                                            for (int i=0;i<RADIX_SORT_WG_SIZE/32;i++)
                                                                              bin_flags[localID].flags[i] = 0;
#else
                                                                          unsigned int *const bflags = (unsigned int*)&bin_flags.get_pointer()[0];                                                                          
                                                                          for (unsigned int i=localID;i<RADIX_SORT_BINS*RADIX_SORT_WG_SIZE/32;i+=step_local)
                                                                            bflags[i] = 0;
#endif                                                                          

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                          sort_type key_value;
                                                                          uint64_t key;
                                                                          if (ID < endID)
                                                                          {
                                                                            key_value = input[ID];
                                                                            key = key_value;
                                                                            binID = (key >> shift) & (RADIX_SORT_BINS - 1);
                                                                            binOffset = local_offset[binID];
                                                                            sycl::atomic_ref<unsigned int, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> bflags(bin_flags[binID].flags[flags_bin]);                                                                            
                                                                            bflags += flags_bit;
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                          if (ID < endID)
                                                                          {
                                                                            unsigned int prefix = 0;
                                                                            unsigned int count = 0;
                                                                            for (unsigned int i = 0; i < RADIX_SORT_WG_SIZE / 32; i++)
                                                                            {
                                                                              const unsigned int bits = bin_flags[binID].flags[i];
                                                                              const unsigned int full_count    = sycl::popcount(bits);
                                                                              const unsigned int partial_count = sycl::popcount(bits & (flags_bit - 1));
                                                                              prefix += (i  < flags_bin) ? full_count : 0;
                                                                              prefix += (i == flags_bin) ? partial_count : 0;
                                                                              count += full_count;
                                                                            }
                                                                            output[binOffset + prefix] = key_value;
                                                                            if (prefix == count - 1)
                                                                              local_offset[binID] += count;                                                                          
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                        }
                                                                    
                                                                      });
                                                 
                                                   });
      return scatter_event;
    }


    template<typename sort_type>
    sycl::event radix_sort_Nx8Bit(sycl::queue &gpu_queue, sort_type *input, sort_type *output, const unsigned int items,  unsigned int *global_histogram, const unsigned int start_iteration, const unsigned int end_iteration, sycl::event &initial, const unsigned int RADIX_SORT_NUM_DSS=256)
    {
      sycl::event events[8]; // 8x8=64bit maximum
      for (unsigned int i=start_iteration;i<end_iteration;i++)
      {
        events[i] = radix_sort_iteration_type<sort_type>(gpu_queue,i == start_iteration ? initial : events[i-1],input,output,items,global_histogram,i,RADIX_SORT_NUM_DSS);
        std::swap(input,output);
      }
      
      return events[end_iteration-1];
    }    
    
  };
};

#endif
