// Copyright 2009-2014 Intel Corporation                                    //
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

#include "subdivpatch1.h"
#include "common/ray16.h"
#include "geometry/filter.h"
#include "subdiv_intersector16.h"
#include "bicubic_bezier_patch.h"

namespace embree
{


 void subdivide_intersect1(const size_t rayIndex, 
			   const mic_f &dir_xyz,
			   const mic_f &org_xyz,
			   Ray16& ray16,
			   const IrregularCatmullClarkPatch &patch,
			   const unsigned int subdiv_level)
 {
   if (subdiv_level == 0)
     {
       __aligned(64) FinalQuad finalQuad;
       patch.init( finalQuad );
       intersect1_quad(rayIndex,dir_xyz,org_xyz,ray16,finalQuad);      
     }
   else
     {
       IrregularCatmullClarkPatch subpatches[4];
       patch.subdivide(subpatches);
       for (size_t i=0;i<4;i++)
	 subdivide_intersect1(rayIndex, 
			      dir_xyz,
			      org_xyz,
			      ray16,
			      subpatches[i],
			      subdiv_level - 1);	    
     }
   
 }


 void subdivide_intersect1(const size_t rayIndex, 
			   const mic_f &dir_xyz,
			   const mic_f &org_xyz,
			   Ray16& ray16,
			   const RegularCatmullClarkPatch &patch,
			   const unsigned int subdiv_level)
 {
   if (subdiv_level == 0)
     {
       __aligned(64) FinalQuad finalQuad;
       patch.init( finalQuad );
       intersect1_quad(rayIndex,dir_xyz,org_xyz,ray16,finalQuad);      
     }
   else
     {
       RegularCatmullClarkPatch subpatches[4];
       patch.subdivide(subpatches);
       for (size_t i=0;i<4;i++)
	 subdivide_intersect1(rayIndex, 
			      dir_xyz,
			      org_xyz,
			      ray16,
			      subpatches[i],
			      subdiv_level - 1);	    
     }
   
 }

};
