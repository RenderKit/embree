// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#if defined(EMBREE_SYCL_SUPPORT)
#include "common.h"

#define RADIX_SORT_BINS        256    

#if 1
#define RADIX_SORT_MAX_NUM_DSS 256
#define RADIX_SORT_WG_SIZE     256
#else
#define RADIX_SORT_MAX_NUM_DSS 128
#define RADIX_SORT_WG_SIZE     512
#endif
//todo access byte directly
namespace embree
{
  namespace gpu
  {

    __forceinline void localAtomicBallot(uint *const histogram, const uint ID, const uint add)
    {
      uint mask = sub_group_ballot(true);
      while(mask)
      {
        const uint first = sycl::ctz(mask);
        const uint index = sub_group_broadcast(ID,first);
        const bool cmp = ID == index;
#if 0     
        const uint count = cmp ? add : 0;
        const uint reduced_count = sub_group_reduce(count, SYCL_ONEAPI::plus<uint>());
        mask &= ~sub_group_ballot(cmp);
#else
        const uint cmp_mask = sub_group_ballot(cmp);
        const uint reduced_count = sycl::popcount(cmp_mask) * add;
        mask &= ~cmp_mask;        
#endif

        sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> hist(histogram[ID]);                                                                                    
        if (get_sub_group_local_id() == first)
          hist += reduced_count;
      }        
    }
    

    template<bool sync>
    void sort_iteration(sycl::queue &gpu_queue, uint64_t *input, uint64_t *output, const uint primitives,  uint *global_histogram, const uint iter, double &time, const uint RADIX_SORT_NUM_DSS=256)
    {
      const uint shift = iter*8;

      
      // ==== bin key into global histogram =====
      {
        const sycl::nd_range<1> nd_range1(sycl::range<1>(RADIX_SORT_WG_SIZE*RADIX_SORT_NUM_DSS),sycl::range<1>(RADIX_SORT_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::local_accessor< uint, 1> histogram(sycl::range<1>(RADIX_SORT_BINS),cgh);                                                 
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint step_local  = item.get_local_range().size();
                                                                        const uint groupID     = item.get_group(0);

                                                                        const uint startID = (groupID + 0)*primitives / RADIX_SORT_NUM_DSS;
                                                                        const uint endID   = (groupID + 1)*primitives / RADIX_SORT_NUM_DSS;
                                                                                                                                                                                                            
                                                                        if (localID < RADIX_SORT_BINS)
                                                                          histogram[localID] = 0;

                                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                    
                                                                        for (uint ID = startID + localID; ID < endID; ID += step_local)
                                                                        {
                                                                          const uint bin = ((uint)(input[ID] >> shift)) & (RADIX_SORT_BINS - 1);
                                                                          localAtomicBallot(histogram.get_pointer(),bin,1);
                                                                        }

                                                                        item.barrier(sycl::access::fence_space::local_space);
    
                                                                        if (localID < RADIX_SORT_BINS)
                                                                          global_histogram[RADIX_SORT_BINS*groupID + localID] = histogram[localID];
                                                                    
                                                                      });
                                                 
                                                   });
        if (sync)
        {
          try {
            gpu_queue.wait_and_throw();
          } catch (sycl::exception const& e) {
            std::cout << "Caught synchronous SYCL exception:\n"
                      << e.what() << std::endl;
            FATAL("OpenCL Exception");     		
          }        
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          PRINT((float)dt);
          time += dt;
        }
      }

#if 0
      // ==== reduce global histogram =====    
      {
        const sycl::nd_range<1> nd_range1(sycl::range<1>(RADIX_SORT_BINS),sycl::range<1>(RADIX_SORT_BINS));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::local_accessor< uint, 1> sums(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint globalID     = item.get_global_id(0);
                                                                        const uint subgroupID      = get_sub_group_id();
                                                                        const uint subgroupSize    = get_sub_group_size();
                                                                        const uint subgroupLocalID = get_sub_group_local_id();
                                                                      
                                                                        uint t = 0;
                                                                        for (uint j = 0; j < RADIX_SORT_NUM_DSS; j++)
                                                                        {
                                                                          const uint count = global_histogram[RADIX_SORT_BINS*j + globalID];
                                                                          global_histogram[RADIX_SORT_BINS*j + globalID] = t;
                                                                          //t += (j <  RADIX_SORT_NUM_DSS) ? count : 0;
                                                                          t += count;
                                                                        }

                                                                        global_histogram[RADIX_SORT_BINS*RADIX_SORT_NUM_DSS + globalID] = t;
                                                                          
                                                                        SYCL_EXT_ONEAPI::sub_group sub_group = this_sub_group();
                                                                        sub_group.barrier();

                                                                        const uint count = t;
                                                                        const uint sum = sub_group_reduce(count, std::plus<uint>());
                                                                        const uint prefix_sum = sub_group_exclusive_scan(count, std::plus<uint>());

                                                                        sums[subgroupID] = sum;
                                                                      
                                                                        item.barrier(sycl::access::fence_space::local_space);

                                                                        const uint sums_prefix_sum = sub_group_broadcast(sub_group_exclusive_scan(sums[subgroupLocalID], std::plus<uint>()),subgroupID);
                                                                        
                                                                        global_histogram[RADIX_SORT_BINS*RADIX_SORT_NUM_DSS + globalID] = sums_prefix_sum + prefix_sum;
                                                                      });
                                                 
                                                   });
        if (sync)
        {      
          try {
            gpu_queue.wait_and_throw();
          } catch (sycl::exception const& e) {
            std::cout << "Caught synchronous SYCL exception:\n"
                      << e.what() << std::endl;
            FATAL("OpenCL Exception");     		
          }
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          time += dt;          
          PRINT((float)dt);
        }
      }
#endif
      
      // ==== scatter key/value pairs according to global historgram =====
      {
        struct __aligned(RADIX_SORT_WG_SIZE/32 * sizeof(uint)) BinFlags
        {
          uint flags[RADIX_SORT_WG_SIZE/32];
        };
      
        const sycl::nd_range<1> nd_range1(sycl::range<1>(RADIX_SORT_WG_SIZE*RADIX_SORT_NUM_DSS),sycl::range<1>(RADIX_SORT_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::local_accessor< uint, 1> local_offset(sycl::range<1>(RADIX_SORT_WG_SIZE),cgh);
                                                     sycl::local_accessor< BinFlags, 1> bin_flags(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     sycl::local_accessor< uint, 1> sums(sycl::range<1>(RADIX_SORT_WG_SIZE),cgh);
                                                     
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint groupID     = item.get_group(0);
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint step_local  = item.get_local_range().size();
                                                                        const uint subgroupID      = get_sub_group_id();
                                                                        const uint subgroupLocalID = get_sub_group_local_id();
                                                                        

                                                                        const uint startID = (groupID + 0)*primitives / RADIX_SORT_NUM_DSS;
                                                                        const uint endID   = (groupID + 1)*primitives / RADIX_SORT_NUM_DSS;


                                                                        /* --- reduce global histogram --- */
#if 1
                                                                        uint local_hist = 0;
                                                                        uint t = 0;
                                                                        for (uint j = 0; j < RADIX_SORT_NUM_DSS; j++)
                                                                        {
                                                                          const uint count = global_histogram[RADIX_SORT_BINS*j + localID];
                                                                          local_hist = (j == groupID) ? t : local_hist;
                                                                          t += count;
                                                                        }
                                                                        
                                                                        SYCL_EXT_ONEAPI::sub_group sub_group = this_sub_group();
                                                                        sub_group.barrier();

                                                                        const uint count = t;
                                                                        const uint sum = sub_group_reduce(count, std::plus<uint>());
                                                                        const uint prefix_sum = sub_group_exclusive_scan(count, std::plus<uint>());

                                                                        sums[subgroupID] = sum;
                                                                      
                                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                        const uint sums_prefix_sum = sub_group_broadcast(sub_group_exclusive_scan(sums[subgroupLocalID], std::plus<uint>()),subgroupID);
                                                                        
                                                                        const uint global_hist = sums_prefix_sum + prefix_sum;
                                                                                                                                                
                                                                        local_offset[localID] = global_hist + local_hist;
#else                                                                        
                                                                        
                                                                        local_offset[localID] = global_histogram[RADIX_SORT_BINS*RADIX_SORT_NUM_DSS + localID] + global_histogram[RADIX_SORT_BINS*groupID + localID];
#endif
                                                                        
                                                                        const uint flags_bin = localID / 32;
                                                                        const uint flags_bit = 1 << (localID % 32);                                                                      
                                                                      
                                                                        for (uint blockID = startID; blockID < endID; blockID += step_local)
                                                                        {
                                                                        
                                                                          const uint ID = blockID + localID;
                                                                        
                                                                          uint binID = 0;
                                                                          uint binOffset = 0;

                                                                          if (localID < RADIX_SORT_BINS)
                                                                          for (int i=0;i<RADIX_SORT_WG_SIZE/32;i++)
                                                                            bin_flags[localID].flags[i] = 0;

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                          uint64_t in;
                                                                          if (ID < endID)
                                                                          {
                                                                            in = input[ID];
                                                                            binID = ((uint)(in >> shift)) & (RADIX_SORT_BINS - 1);
                                                                            binOffset = local_offset[binID];
                                                                            sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> bflags(bin_flags[binID].flags[flags_bin]);                                                                            
                                                                            bflags += flags_bit;
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                          if (ID < endID)
                                                                          {
                                                                            uint prefix = 0;
                                                                            uint count = 0;
                                                                            for (uint i = 0; i < RADIX_SORT_WG_SIZE / 32; i++)
                                                                            {
                                                                              const uint bits = bin_flags[binID].flags[i];
                                                                              const uint full_count    = sycl::popcount(bits);
                                                                              const uint partial_count = sycl::popcount(bits & (flags_bit - 1));
                                                                              prefix += (i  < flags_bin) ? full_count : 0;
                                                                              prefix += (i == flags_bin) ? partial_count : 0;
                                                                              count += full_count;
                                                                            }
                                                                            output[binOffset + prefix] = in;
                                                                            if (prefix == count - 1)
                                                                              local_offset[binID] += count;                                                                          
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                        }
                                                                    
                                                                      });
                                                 
                                                   });

        if (sync)
        {            
          try {
            gpu_queue.wait_and_throw();
          } catch (sycl::exception const& e) {
            std::cout << "Caught synchronous SYCL exception:\n"
                      << e.what() << std::endl;
            FATAL("OpenCL Exception");     		
          }
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          time += dt;          
          PRINT((float)dt);
        }
        //exit(0);
      }    
    }


    __forceinline void radix_sort_single_workgroup(sycl::queue &gpu_queue, uint64_t *_input, uint64_t *_output, const uint numPrimitives,  double &time)
    {
      static const uint RADIX_SORT_SINGLE_WG_SIZE = 256;
      static const uint RADIX_SORT_SINGLE_WG_SUB_GROUP_SIZE = 16;

      struct __aligned(RADIX_SORT_SINGLE_WG_SIZE/32 * sizeof(uint)) BinFlags
      {
        uint flags[RADIX_SORT_SINGLE_WG_SIZE/32];
      };
    
      {
        const sycl::nd_range<1> nd_range1(RADIX_SORT_SINGLE_WG_SIZE,sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::local_accessor< uint, 1> histogram(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                                     sycl::local_accessor< uint, 1> sums(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                                     sycl::local_accessor< uint, 1> prefix_sums(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                                     sycl::local_accessor< uint, 1> local_offset(sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE),cgh);
                                                     sycl::local_accessor< BinFlags, 1> bin_flags(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                   
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(RADIX_SORT_SINGLE_WG_SUB_GROUP_SIZE)                                                                    
                                                                      {                                                                                                                                            
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint step_local  = item.get_local_range().size();
                                                                        const uint subgroupID      = get_sub_group_id();
                                                                        const uint subgroupSize    = get_sub_group_size();
                                                                        const uint subgroupLocalID = get_sub_group_local_id();

                                                                        for (uint iter = 0;iter<8;iter++)
                                                                        {
                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                          
                                                                          const uint shift = iter*8;

                                                                          const uint64_t *const input  = (iter % 2) == 0 ? _input : _output;
                                                                          uint64_t *const output = (iter % 2) == 1 ? _input : _output;
                                                                        
                                                                          // ==== bin key into global histogram =====
                                                                      
                                                                          if (localID < RADIX_SORT_BINS)
                                                                            histogram[localID] = 0;

                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                    
                                                                          for (uint ID = localID; ID < numPrimitives; ID += step_local)
                                                                          {
                                                                            const uint bin = ((uint)(input[ID] >> shift)) & (RADIX_SORT_BINS - 1);
                                                                            localAtomicBallot(histogram.get_pointer(),bin,1);
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                          // ==== reduce global histogram =====    
                                                                                                                                              
                                                                          SYCL_EXT_ONEAPI::sub_group sub_group = this_sub_group();
                                                                          sub_group.barrier();

                                                                          if (localID < RADIX_SORT_BINS)
                                                                          {
                                                                            const uint count = histogram[localID];
                                                                            const uint sum = sub_group_reduce(count, std::plus<uint>());
                                                                            const uint prefix_sum = sub_group_exclusive_scan(count, std::plus<uint>());

                                                                            sums[localID] = sum;
                                                                            prefix_sums[localID] = prefix_sum;
                                                                          }
                                                                        
                                                                          item.barrier(sycl::access::fence_space::local_space);


                                                                          if (subgroupID == 0)
                                                                          {
                                                                            uint off = 0;
                                                                            for (int i = subgroupLocalID; i < RADIX_SORT_BINS; i += subgroupSize)
                                                                            {
                                                                              local_offset[i] = off + prefix_sums[i];
                                                                              off += sums[i];
                                                                            }
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);                                                                          
                                                                      
                                                                          // ==== scatter key/value pairs according to global historgram =====      
                                                                                                                         
                                                                          const uint flags_bin = localID / 32;
                                                                          const uint flags_bit = 1 << (localID % 32);                                                                      
                                                                      
                                                                          for (uint blockID = 0; blockID < numPrimitives; blockID += step_local)
                                                                          {
                                                                            item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                            const uint ID = blockID + localID;                                                                            
                                                                        
                                                                            uint binID = 0;
                                                                            uint binOffset = 0;

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
                                                                              
                                                                              sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> bflags(bin_flags[binID].flags[flags_bin]);                                                                                    
                                                                              bflags.fetch_add(flags_bit);

                                                                            }

                                                                          
                                                                            item.barrier(sycl::access::fence_space::local_space);
                                                                            
                                                                            if (ID < numPrimitives)
                                                                            {
                                                                              uint prefix = 0;
                                                                              uint count = 0;
                                                                              for (uint i = 0; i < RADIX_SORT_SINGLE_WG_SIZE / 32; i++)
                                                                              {
                                                                                const uint bits = bin_flags[binID].flags[i];
                                                                                const uint full_count    = sycl::popcount(bits);
                                                                                const uint partial_count = sycl::popcount(bits & (flags_bit - 1));
                                                                                prefix += (i  < flags_bin) ? full_count : 0;
                                                                                prefix += (i == flags_bin) ? partial_count : 0;
                                                                                count += full_count;
                                                                              }

                                                                              output[binOffset + prefix] = in;
                                                                              if (prefix == count - 1)
                                                                                local_offset[binID] += count;                                                                          
                                                                            }
                                                                            item.barrier(sycl::access::fence_space::local_space);
                                                                          
                                                                          }                                                                       
                                                                        }
                                                                      });
                                                   
                                                   });

        {            
          try {
            gpu_queue.wait_and_throw();
          } catch (sycl::exception const& e) {
            std::cout << "Caught synchronous SYCL exception:\n"
                      << e.what() << std::endl;
            FATAL("OpenCL Exception");     		
          }
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          PRINT((float)dt);
          time += dt;
        }
      }    
    }



    __forceinline void radix_sort_ranges(sycl::queue &gpu_queue, uint64_t *global_input, uint64_t *global_output, gpu::Range *const ranges, const uint numRanges, const uint start_iteration, const uint end_iteration, double &time)
    {
      static const uint RADIX_SORT_SINGLE_WG_SIZE = 512;
      static const uint RADIX_SORT_SINGLE_WG_SUB_GROUP_SIZE = 16;

      struct __aligned(RADIX_SORT_SINGLE_WG_SIZE/32 * sizeof(uint)) BinFlags
      {
        uint flags[RADIX_SORT_SINGLE_WG_SIZE/32];
      };
    
      {
        const sycl::nd_range<1> nd_range1(numRanges * RADIX_SORT_SINGLE_WG_SIZE,sycl::range<1>(RADIX_SORT_SINGLE_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::local_accessor< uint, 1> histogram(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     sycl::local_accessor< uint, 1> sums(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     sycl::local_accessor< uint, 1> prefix_sums(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     sycl::local_accessor< uint, 1> local_offset(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     sycl::local_accessor< BinFlags, 1> bin_flags(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                   
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(RADIX_SORT_SINGLE_WG_SUB_GROUP_SIZE)                                                                    
                                                                      {
                                                                        const uint groupID = item.get_group(0);                                                                        
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint step_local  = item.get_local_range().size();
                                                                        const uint subgroupID      = get_sub_group_id();
                                                                        const uint subgroupSize    = get_sub_group_size();
                                                                        const uint subgroupLocalID = get_sub_group_local_id();

                                                                        const uint startID = ranges[groupID].start;
                                                                        const uint endID   = ranges[groupID].end;
                                                                        const uint numPrimitives = endID-startID;
                                                                        
                                                                        uint64_t *_input  = &global_input[startID];
                                                                        uint64_t *_output = &global_output[startID];
                                                                        
                                                                        for (uint iter = start_iteration;iter<end_iteration;iter++)
                                                                        {
                                                                          const uint shift = iter*8;

                                                                          const uint64_t *const input  = (iter % 2) == 0 ? _input : _output;
                                                                          uint64_t *const output = (iter % 2) == 1 ? _input : _output;
                                                                        
                                                                          // ==== bin key into global histogram =====
                                                                      
                                                                          if (localID < RADIX_SORT_BINS)
                                                                            histogram[localID] = 0;

                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                    
                                                                          for (uint ID = localID; ID < numPrimitives; ID += step_local)
                                                                          {
                                                                            const uint bin = ((uint)(input[ID] >> shift)) & (RADIX_SORT_BINS - 1);
                                                                            localAtomicBallot(histogram.get_pointer(),bin,1);
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                          // ==== reduce global histogram =====    
                                                                                                                                              
                                                                          SYCL_EXT_ONEAPI::sub_group sub_group = this_sub_group();
                                                                          sub_group.barrier();

                                                                          if (localID < RADIX_SORT_BINS)
                                                                          {
                                                                            const uint count = histogram[localID];
                                                                            const uint sum = sub_group_reduce(count, std::plus<uint>());
                                                                            const uint prefix_sum = sub_group_exclusive_scan(count, std::plus<uint>());

                                                                            sums[localID] = sum;
                                                                            prefix_sums[localID] = prefix_sum;
                                                                          }
                                                                        
                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                          if (subgroupID == 0)
                                                                          {
                                                                            uint off = 0;
                                                                            for (int i = subgroupLocalID; i < RADIX_SORT_BINS; i += subgroupSize)
                                                                            {
                                                                              local_offset[i] = off + prefix_sums[i];
                                                                              off += sums[i];
                                                                            }
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                      
                                                                          // ==== scatter key/value pairs according to global historgram =====      
                                                                                                                         
                                                                          const uint flags_bin = localID / 32;
                                                                          const uint flags_bit = 1 << (localID % 32);                                                                      
                                                                      
                                                                          for (uint blockID = 0; blockID < numPrimitives; blockID += step_local)
                                                                          {
                                                                        
                                                                            const uint ID = blockID + localID;
                                                                        
                                                                            uint binID = 0;
                                                                            uint binOffset = 0;

                                                                            if (localID < RADIX_SORT_BINS)
                                                                              for (int i=0;i<RADIX_SORT_SINGLE_WG_SIZE/32;i++)
                                                                                bin_flags[localID].flags[i] = 0;

                                                                            item.barrier(sycl::access::fence_space::local_space);

                                                                        
                                                                            if (ID < numPrimitives)
                                                                            {
                                                                              binID = (input[ID] >> shift) & (RADIX_SORT_BINS - 1);                                                                            
                                                                              binOffset = local_offset[binID];                                                                            
                                                                              sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> bflags(bin_flags[binID].flags[flags_bin]);                                                                                    
                                                                              bflags += flags_bit;

                                                                            }

                                                                          
                                                                            item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                            if (ID < numPrimitives)
                                                                            {
                                                                              uint prefix = 0;
                                                                              uint count = 0;
                                                                              for (uint i = 0; i < RADIX_SORT_SINGLE_WG_SIZE / 32; i++)
                                                                              {
                                                                                const uint bits = bin_flags[binID].flags[i];
                                                                                const uint full_count    = sycl::popcount(bits);
                                                                                const uint partial_count = sycl::popcount(bits & (flags_bit - 1));
                                                                                prefix += (i  < flags_bin) ? full_count : 0;
                                                                                prefix += (i == flags_bin) ? partial_count : 0;
                                                                                count += full_count;
                                                                              }
                                                                              output[binOffset + prefix] = input[ID];
                                                                              if (prefix == count - 1)
                                                                                local_offset[binID] += count;                                                                          
                                                                            }
                                                                            item.barrier(sycl::access::fence_space::local_space);
                                                                          
                                                                          }                                                                        
                                                                        }
                                                                      });
                                                   
                                                   });

          try {
            gpu_queue.wait_and_throw();
          } catch (sycl::exception const& e) {
            std::cout << "Caught synchronous SYCL exception:\n"
                      << e.what() << std::endl;
            FATAL("OpenCL Exception");     		
          }
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          PRINT((float)dt);
          time += dt;
        }
    }
    
  template<bool sync, typename sort_type>
    std::pair<sycl::event, sycl::event> sort_iteration_type(sycl::queue &gpu_queue, sort_type *input, sort_type *output, const uint primitives,  uint *global_histogram, const uint iter, double &time, const uint RADIX_SORT_NUM_DSS=256)
    {
      std::pair<sycl::event, sycl::event> out;
      const uint shift = iter*8;

      
      // ==== bin key into global histogram =====
      {
        const sycl::nd_range<1> nd_range1(sycl::range<1>(RADIX_SORT_WG_SIZE*RADIX_SORT_NUM_DSS),sycl::range<1>(RADIX_SORT_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::local_accessor< uint, 1> histogram(sycl::range<1>(RADIX_SORT_BINS),cgh);                                                 
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint step_local  = item.get_local_range().size();
                                                                        const uint groupID     = item.get_group(0);

                                                                        const uint startID = (groupID + 0)*primitives / RADIX_SORT_NUM_DSS;
                                                                        const uint endID   = (groupID + 1)*primitives / RADIX_SORT_NUM_DSS;
                                                                                                                                                                                                            
                                                                        if (localID < RADIX_SORT_BINS)
                                                                          histogram[localID] = 0;

                                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                    
                                                                        for (uint ID = startID + localID; ID < endID; ID += step_local)
                                                                        {
                                                                          const uint64_t key = input[ID];
                                                                          const uint bin = ((uint)(key >> shift)) & (RADIX_SORT_BINS - 1);
                                                                          gpu::localAtomicBallot(histogram.get_pointer(),bin,1);
                                                                        }

                                                                        item.barrier(sycl::access::fence_space::local_space);
    
                                                                        if (localID < RADIX_SORT_BINS)
                                                                          global_histogram[RADIX_SORT_BINS*groupID + localID] = histogram[localID];
                                                                    
                                                                      });
                                                 
                                                   });
        if (sync)
        {
          try {
            gpu_queue.wait_and_throw();
          } catch (sycl::exception const& e) {
            std::cout << "Caught synchronous SYCL exception:\n"
                      << e.what() << std::endl;
            FATAL("OpenCL Exception");     		
          }        
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          PRINT((float)dt);
          time += dt;
        }

        out.first = queue_event;
      }

      
      // ==== scatter key/value pairs according to global historgram =====
      {
        
        struct __aligned(RADIX_SORT_WG_SIZE/32 * sizeof(uint)) BinFlags
        {
          uint flags[RADIX_SORT_WG_SIZE/32];
        };
      
        const sycl::nd_range<1> nd_range1(sycl::range<1>(RADIX_SORT_WG_SIZE*RADIX_SORT_NUM_DSS),sycl::range<1>(RADIX_SORT_WG_SIZE));          
        sycl::event queue_event = gpu_queue.submit([&](sycl::handler &cgh) {
                                                     sycl::local_accessor< uint, 1> local_offset(sycl::range<1>(RADIX_SORT_WG_SIZE),cgh);
                                                     sycl::local_accessor< uint, 1> sums(sycl::range<1>(RADIX_SORT_WG_SIZE),cgh);                                                     
                                                     sycl::local_accessor< BinFlags, 1> bin_flags(sycl::range<1>(RADIX_SORT_BINS),cgh);
                                                     
                                                     cgh.parallel_for(nd_range1,[=](sycl::nd_item<1> item) EMBREE_SYCL_SIMD(16)
                                                                      {
                                                                        const uint groupID     = item.get_group(0);
                                                                        const uint localID     = item.get_local_id(0);
                                                                        const uint step_local  = item.get_local_range().size();
                                                                        const uint subgroupID      = get_sub_group_id();
                                                                        const uint subgroupLocalID = get_sub_group_local_id();
                                                                        

                                                                        const uint startID = (groupID + 0)*primitives / RADIX_SORT_NUM_DSS;
                                                                        const uint endID   = (groupID + 1)*primitives / RADIX_SORT_NUM_DSS;


                                                                        /* --- reduce global histogram --- */
                                                                        uint local_hist = 0;
                                                                        uint t = 0;
                                                                        for (uint j = 0; j < RADIX_SORT_NUM_DSS; j++)
                                                                        {
                                                                          const uint count = global_histogram[RADIX_SORT_BINS*j + localID];
                                                                          local_hist = (j == groupID) ? t : local_hist;
                                                                          t += count;
                                                                        }
                                                                        
                                                                        SYCL_EXT_ONEAPI::sub_group sub_group = this_sub_group();
                                                                        sub_group.barrier();

                                                                        const uint count = t;
                                                                        const uint sum = sub_group_reduce(count, std::plus<uint>());
                                                                        const uint prefix_sum = sub_group_exclusive_scan(count, std::plus<uint>());

                                                                        sums[subgroupID] = sum;
                                                                      
                                                                        item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                        const uint sums_prefix_sum = sub_group_broadcast(sub_group_exclusive_scan(sums[subgroupLocalID], std::plus<uint>()),subgroupID);
                                                                        
                                                                        const uint global_hist = sums_prefix_sum + prefix_sum;
                                                                                                                                                
                                                                        local_offset[localID] = global_hist + local_hist;
                                                                        
                                                                        const uint flags_bin = localID / 32;
                                                                        const uint flags_bit = 1 << (localID % 32);                                                                      
                                                                      
                                                                        for (uint blockID = startID; blockID < endID; blockID += step_local)
                                                                        {
                                                                        
                                                                          const uint ID = blockID + localID;
                                                                        
                                                                          uint binID = 0;
                                                                          uint binOffset = 0;

                                                                          if (localID < RADIX_SORT_BINS)
                                                                            for (int i=0;i<RADIX_SORT_WG_SIZE/32;i++)
                                                                              bin_flags[localID].flags[i] = 0;

                                                                          item.barrier(sycl::access::fence_space::local_space);

                                                                          sort_type key_value;
                                                                          uint64_t key;
                                                                          if (ID < endID)
                                                                          {
                                                                            key_value = input[ID];
                                                                            key = key_value;
                                                                            binID = (key >> shift) & (RADIX_SORT_BINS - 1);
                                                                            binOffset = local_offset[binID];
                                                                            sycl::atomic_ref<uint, sycl::memory_order::relaxed, sycl::memory_scope::work_group,sycl::access::address_space::local_space> bflags(bin_flags[binID].flags[flags_bin]);                                                                            
                                                                            bflags += flags_bit;
                                                                          }

                                                                          item.barrier(sycl::access::fence_space::local_space);
                                                                        
                                                                          if (ID < endID)
                                                                          {
                                                                            uint prefix = 0;
                                                                            uint count = 0;
                                                                            for (uint i = 0; i < RADIX_SORT_WG_SIZE / 32; i++)
                                                                            {
                                                                              const uint bits = bin_flags[binID].flags[i];
                                                                              const uint full_count    = sycl::popcount(bits);
                                                                              const uint partial_count = sycl::popcount(bits & (flags_bit - 1));
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

        if (sync)
        {            
          try {
            gpu_queue.wait_and_throw();
          } catch (sycl::exception const& e) {
            std::cout << "Caught synchronous SYCL exception:\n"
                      << e.what() << std::endl;
            FATAL("OpenCL Exception");     		
          }
          const auto t0 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_start>();
          const auto t1 = queue_event.template get_profiling_info<sycl::info::event_profiling::command_end>();
          const double dt = (t1-t0)*1E-6;
          time += dt;          
          PRINT((float)dt);
        }
        //exit(0);
        out.second = queue_event;
      }
      return out;
    }


    
  };
};

#endif
