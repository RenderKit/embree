// ======================================================================== //
// Copyright 2009-2016 Intel Corporation                                    //
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

#include "../include/embree2/rtcore.h"

namespace embree
{
  struct RTCSceneRef
  {
  public:
    RTCScene scene;
    
    RTCSceneRef (std::nullptr_t) 
      : scene(nullptr) {}
    
    RTCSceneRef (RTCScene scene) 
    : scene(scene) {}
    
    ~RTCSceneRef () { 
      rtcDeleteScene(scene); 
    }
    
    __forceinline operator RTCScene () const { return scene; }
    
    __forceinline RTCSceneRef& operator= (RTCSceneRef& in) 
    {
      RTCScene tmp = in.scene;
      in.scene = nullptr;
      if (scene) rtcDeleteScene(scene);
      scene = tmp;
      return *this;
    }
      
    __forceinline RTCSceneRef& operator= (RTCScene in) 
    {
      if (scene) rtcDeleteScene(scene);
      scene = in;
      return *this;
    }
        
    __forceinline RTCSceneRef& operator= (std::nullptr_t) 
    {
      if (scene) rtcDeleteScene(scene);
      scene = nullptr;
      return *this;
    }
  };

  std::string string_of(RTCError code)
  {
    switch (code) {
    case RTC_UNKNOWN_ERROR    : return "RTC_UNKNOWN_ERROR";
    case RTC_INVALID_ARGUMENT : return "RTC_INVALID_ARGUMENT";
    case RTC_INVALID_OPERATION: return "RTC_INVALID_OPERATION";
    case RTC_OUT_OF_MEMORY    : return "RTC_OUT_OF_MEMORY";
    case RTC_UNSUPPORTED_CPU  : return "RTC_UNSUPPORTED_CPU";
    case RTC_CANCELLED        : return "RTC_CANCELLED";
    default                   : return "invalid error code";
    }
  }

  /* error reporting function */
  void error_handler(const RTCError code, const char* str = nullptr)
  {
    if (code == RTC_NO_ERROR) 
      return;
    
    std::string errorStr;
    errorStr += "Embree: ";
    errorStr += string_of(code);
    if (str) errorStr += " (" + std::string(str) + ")";
    throw std::runtime_error(errorStr);
  }
}

