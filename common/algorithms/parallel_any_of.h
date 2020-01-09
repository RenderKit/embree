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

#pragma once

#include <functional>
#include "parallel_reduce.h"

namespace embree
{

    template<typename Index, class UnaryPredicate>
    __forceinline bool parallel_any_of (Index first, Index last, UnaryPredicate pred)
    {
        bool ret = false;

#if defined(TASKING_TBB)
        tbb::parallel_for(tbb::blocked_range<size_t>{first, last}, [&ret,pred](const tbb::blocked_range<size_t>& r) {
            if (tbb::task::self().is_cancelled()) return;
            for (size_t i = r.begin(); i != r.end(); ++i) {
                if (pred(i)) {
                    ret = true;
                    tbb::task::self().cancel_group_execution();
                }
            }
        });
#else
        ret = parallel_reduce (first, last, false, 
            [pred](const range<size_t>& r)->bool {
                bool localret = false;
                for (auto i=r.begin(); i<r.end(); ++i) {
                    localret |= pred(i);
                }
                return localret;
            },
            std::bit_or<bool>()
        );
#endif

        return ret;
    }

} // end namespace
