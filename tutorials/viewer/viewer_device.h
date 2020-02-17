// ======================================================================== //
// Copyright 2009-2020 Intel Corporation                                    //
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

#include "../common/tutorial/tutorial_device.h"
#include "../common/math/random_sampler.h"
#include "../common/core/differential_geometry.h"
#include "../common/tutorial/scene_device.h"

namespace embree {

extern "C" ISPCScene* g_ispc_scene;
extern "C" int g_instancing_mode;

struct TutorialData
{
  ISPCScene* ispc_scene;
  int instancing_mode;
  RTCIntersectContextFlags iflags_coherent;
  
  /* scene data */
  RTCScene scene;
  bool subdiv_mode;
};

void TutorialData_Constructor(TutorialData* This)
{
  This->ispc_scene = g_ispc_scene;
  This->instancing_mode = g_instancing_mode;
  This->iflags_coherent = g_iflags_coherent;
  This->scene = nullptr;
  This->subdiv_mode = false;
}

void TutorialData_Destructor(TutorialData* This)
{
  rtcReleaseScene (This->scene); This->scene = nullptr;
}

} // namespace embree
