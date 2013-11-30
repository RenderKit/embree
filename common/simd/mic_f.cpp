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
  __align(64) float mic_f::float_idMod4[16] = {0.f,1.f,2.f,3.f,0.f,1.f,2.f,3.f,0.f,1.f,2.f,3.f,0.f,1.f,2.f,3.f};
  __align(64) float mic_f::float_idDiv4[16] = {0.f,0.f,0.f,0.f,1.f,1.f,1.f,1.f,2.f,2.f,2.f,2.f,3.f,3.f,3.f,3.f};
  __align(64) float mic_f::float_identity[16] = {0.f,1.f,2.f,3.f,4.f,5.f,6.f,7.f,8.f,9.f,10.f,11.f,12.f,13.f,14.f,15.f};
  __align(64) float mic_f::float_cancelLE[4] = { 1.0f,1.0f,1.0f,0.0f };
  
  __align(64) float mic_f::float_one_over[32] = {
    0.0f,
    1.0f /  1.0f,
    1.0f /  2.0f,
    1.0f /  3.0f,
    1.0f /  4.0f,
    1.0f /  5.0f,
    1.0f /  6.0f,
    1.0f /  8.0f,
    1.0f /  9.0f,
    1.0f / 10.0f,
    1.0f / 11.0f,
    1.0f / 12.0f,
    1.0f / 13.0f,
    1.0f / 14.0f,
    1.0f / 15.0f,
    1.0f / 16.0f,
    1.0f / 17.0f,
    1.0f / 18.0f,
    1.0f / 19.0f,
    1.0f / 21.0f,
    1.0f / 22.0f,
    1.0f / 23.0f,
    1.0f / 24.0f,
    1.0f / 25.0f,
    1.0f / 26.0f,
    1.0f / 27.0f,
    1.0f / 28.0f,
    1.0f / 29.0f,
    1.0f / 30.0f,
    1.0f / 31.0f
  };
#endif
}


