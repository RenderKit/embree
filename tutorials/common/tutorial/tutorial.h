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

#pragma once

#include "../../../common/sys/platform.h"
#include "../../../common/sys/sysinfo.h"
#include "../../../common/sys/ref.h"
#include "../../../common/lexers/streamfilters.h"
#include "../../../common/lexers/parsestream.h"
#include "glutdisplay.h"
#include "../transport/transport_host.h"

#if defined __WIN32__
inline double drand48() {
  return (double)rand()/(double)RAND_MAX;
}
#endif
