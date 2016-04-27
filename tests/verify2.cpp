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

#include "../kernels/common/default.h"
#include "../include/embree2/rtcore.h"
#include "../include/embree2/rtcore_ray.h"
#include "rtcore_helpers.h"
#include "../tutorials/common/tutorial/application.h"

#define DEFAULT_STACK_SIZE 4*1024*1024

#if defined(__INTEL_COMPILER)
#pragma warning (disable: 1478) // warning: function was declared deprecated
#elif defined(_MSC_VER)
#pragma warning (disable: 4996) // warning: function was declared deprecated
#elif defined(__clang__)
#pragma clang diagnostic ignored "-Wdeprecated-declarations" // warning: xxx is deprecated
#elif defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wdeprecated-declarations" // warning: xxx is deprecated
#endif

namespace embree
{
  class VerifyApplication : public Application
  {
  public:

    VerifyApplication ()
    {
      registerOption("rtcore", [this] (Ref<ParseStream> cin, const FileName& path) {
          rtcore += "," + cin->getString();
        }, "--rtcore <string>: uses <string> to configure Embree device");
      
      registerOption("threads", [this] (Ref<ParseStream> cin, const FileName& path) {
          rtcore += ",threads=" + toString(cin->getInt());
        }, "--threads <int>: number of threads to use");
      
      registerOption("affinity", [this] (Ref<ParseStream> cin, const FileName& path) {
          rtcore += ",set_affinity=1";
        }, "--affinity: affinitize threads");
      
      registerOption("verbose", [this] (Ref<ParseStream> cin, const FileName& path) {
          rtcore += ",verbose=" + toString(cin->getInt());
        }, "--verbose <int>: sets verbosity level");

      registerOption("regressions", [this] (Ref<ParseStream> cin, const FileName& path) {
          regressionN = cin->getInt();
        }, "--regressions <int>: number of regressions to perform");
    }

    int main(int argc, char** argv) try
    {
      /* parse command line options */
      parseCommandLine(argc,argv);
      
      return 0;
    }  
    catch (const std::exception& e) {
      std::cout << "Error: " << e.what() << std::endl;
      return 1;
    }
    catch (...) {
      std::cout << "Error: unknown exception caught." << std::endl;
      return 1;
    }

  public:
    std::string rtcore;
    int regressionN;
  };
}

int main(int argc, char** argv)
{
  embree::VerifyApplication app;
  app.main(argc,argv);
}
