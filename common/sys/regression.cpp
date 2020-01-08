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

#include "regression.h"

namespace embree
{
  /* registerRegressionTest is invoked from static initializers, thus
   * we cannot have the regression_tests variable as global static
   * variable due to issues with static variable initialization
   * order. */
  std::vector<RegressionTest*>& get_regression_tests()
  {
    static std::vector<RegressionTest*> regression_tests;
    return regression_tests;
  } 

  void registerRegressionTest(RegressionTest* test) 
  {
    get_regression_tests().push_back(test);
  }

  RegressionTest* getRegressionTest(size_t index)
  {
    if (index >= get_regression_tests().size())
      return nullptr;
    
    return get_regression_tests()[index];
  }
}
