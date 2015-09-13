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

#include "../../include/embree2/rtcore.h"

namespace embree
{
#if TBB_INTERFACE_VERSION_MAJOR < 8    
#  define USE_TASK_ARENA 0
#else
#  define USE_TASK_ARENA 1
#endif

/*! Makros used in the rtcore API implementation */
#define RTCORE_CATCH_BEGIN try {
#define RTCORE_CATCH_END(device)                                        \
  } catch (std::bad_alloc&) {                                            \
    if (device) (device)->process_error(RTC_OUT_OF_MEMORY,"out of memory"); \
  } catch (rtcore_error& e) {                                           \
    if (device) (device)->process_error(e.error,e.what());              \
  } catch (std::exception& e) {                                         \
    if (device) (device)->process_error(RTC_UNKNOWN_ERROR,e.what());    \
  } catch (...) {                                                       \
    if (device) (device)->process_error(RTC_UNKNOWN_ERROR,"unknown exception caught"); \
  }

#define RTCORE_CATCH_END_NOREPORT                                     \
  } catch (...) {                                                     \
  }

#define RTCORE_VERIFY_HANDLE(handle)                                    \
  if (handle == nullptr) {                                              \
    throw_RTCError(RTC_INVALID_ARGUMENT,"invalid argument");            \
  }

#define RTCORE_VERIFY_GEOMID(id)                                  \
  if (id == -1) {                                                  \
    throw_RTCError(RTC_INVALID_ARGUMENT,"invalid argument");       \
  }

#if 0 // enable to debug print all API calls
#define RTCORE_TRACE(x) std::cout << #x << std::endl;
#else
#define RTCORE_TRACE(x) 
#endif

  /*! used to throw embree API errors */
  struct rtcore_error : public std::exception
  {
    __forceinline rtcore_error(RTCError error, const std::string& str)
      : error(error), str(str) {}
    
    ~rtcore_error() throw() {}
    
    const char* what () const throw () {
      return str.c_str();
    }
    
    RTCError error;
    std::string str;
  };
  
#define throw_RTCError(error,str) \
  throw rtcore_error(error,std::string(__FILE__) + " (" + std::to_string((long long)__LINE__) + "): " + std::string(str));
}
