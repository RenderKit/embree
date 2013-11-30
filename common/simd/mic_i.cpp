// ======================================================================== //
//                 INTEL CORPORATION PROPRIETARY INFORMATION                //
//   This software is supplied under the terms of a license agreement or    //
//   nondisclosure agreement with Intel Corporation and may not be copied   //
//    or disclosed except in accordance with the terms of that agreement.   //
//        Copyright (C) 2012 Intel Corporation. All Rights Reserved.        //
// ======================================================================== //

#include "mic.h"

namespace embree
{
#if 0
  __align(64) int mic_i::int_identity[16] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15 };
  __align(64) int mic_i::int_idMod4[16] = {0,1,2,3,0,1,2,3,0,1,2,3,0,1,2,3};
  __align(64) int mic_i::int_idDiv4[16] = {0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3};
  __align(64) int mic_i::int_pow4[16] = {1,4,16,64,256,1024,4096,16384,65536,262144,1048576,4194304,16777216,67108864,268435456,1073741824};
  __align(64) int mic_i::int_zlc4[4] = {0xffffffff,0xffffffff,0xffffffff,0};
  __align(64) int mic_i::int_addtriID4[16] = {0,0,0,0,1,1,1,1,2,2,2,2,3,3,3,3};
  
  __align(64) int mic_i::int_aos2soa[16] = {
    0,4,8,12,1,5,9,13,2,6,10,14,3,7,11,15
  };
  
  __align(64) int mic_i::int_reverse_identity[16] = { 15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0 };
#endif


}
