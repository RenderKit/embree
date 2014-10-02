// ======================================================================== //
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

#include "subdivpatch1_intersector1.h"
#include "bicubic_bezier_patch.h"

namespace embree
{

  void SubdivPatch1Intersector1::subdivide_intersect1(const Precalculations& pre,
			    Ray& ray,
			    const IrregularCatmullClarkPatch &patch,
			    const unsigned int subdiv_level)
  {
    if (subdiv_level == 0)
      {
	__aligned(64) FinalQuad finalQuad;
	patch.init( finalQuad );
        
	intersectTri(finalQuad.vtx[0],
                     finalQuad.vtx[1],
                     finalQuad.vtx[2],
                     ray,
                     finalQuad.geomID,
                     finalQuad.primID,NULL); 

	intersectTri(finalQuad.vtx[2],
                     finalQuad.vtx[3],
                     finalQuad.vtx[0],
                     ray,
                     finalQuad.geomID,
                     finalQuad.primID,NULL); 
      }
    else
      {
	IrregularCatmullClarkPatch subpatches[4];
	patch.subdivide(subpatches);
	for (size_t i=0;i<4;i++)
	  if (intersectBounds(pre,ray,subpatches[i].bounds()))
	    subdivide_intersect1(pre,
				 ray,
				 subpatches[i],
				 subdiv_level - 1);	    
      }   
  }

  bool SubdivPatch1Intersector1::subdivide_occluded1(const Precalculations& pre,
			   Ray& ray,
			   const IrregularCatmullClarkPatch &patch,
			   const unsigned int subdiv_level)
  {
    if (subdiv_level == 0)
      {
	__aligned(64) FinalQuad finalQuad;
	patch.init( finalQuad );
        
	if (occludedTri(finalQuad.vtx[0],
			finalQuad.vtx[1],
			finalQuad.vtx[2],
			ray,
			finalQuad.geomID,
			finalQuad.primID,NULL)) return true; 

	if (occludedTri(finalQuad.vtx[2],
			finalQuad.vtx[3],
			finalQuad.vtx[0],
			ray,
			finalQuad.geomID,
			finalQuad.primID,NULL)) return false;
      }
    else
      {
	IrregularCatmullClarkPatch subpatches[4];
	patch.subdivide(subpatches);
	for (size_t i=0;i<4;i++)
	  if (intersectBounds(pre,ray,subpatches[i].bounds()))
	    if (subdivide_occluded1(pre,
				    ray,
				    subpatches[i],
				    subdiv_level - 1)) return true;
      }   
    return false;
  }

						      

  void SubdivPatch1Intersector1::subdivide_intersect1(const Precalculations& pre,
						      Ray& ray,
						      const RegularCatmullClarkPatch &patch,
						      const unsigned int subdiv_level)
 {
   if (subdiv_level == 0)
     {
       __aligned(64) FinalQuad finalQuad;
       patch.init( finalQuad );

       intersectTri(finalQuad.vtx[0],
		    finalQuad.vtx[1],
		    finalQuad.vtx[2],
		    ray,
		    finalQuad.geomID,
		    finalQuad.primID,NULL); 
       
       intersectTri(finalQuad.vtx[2],
		    finalQuad.vtx[3],
		    finalQuad.vtx[0],
		    ray,
		    finalQuad.geomID,
		    finalQuad.primID,NULL); 
     }
   else
     {
       RegularCatmullClarkPatch subpatches[4];
       patch.subdivide(subpatches);
       for (size_t i=0;i<4;i++)
	 if (intersectBounds(pre,ray,subpatches[i].bounds()))
	   subdivide_intersect1(pre,
				ray,
				subpatches[i],
				subdiv_level - 1);	    
       
     } 
 }

  bool SubdivPatch1Intersector1::subdivide_occluded1(const Precalculations& pre,
						     Ray& ray,
						     const RegularCatmullClarkPatch &patch,
						     const unsigned int subdiv_level)
  {
    if (subdiv_level == 0)
      {
	__aligned(64) FinalQuad finalQuad;
	patch.init( finalQuad );
        
	if (occludedTri(finalQuad.vtx[0],
			finalQuad.vtx[1],
			finalQuad.vtx[2],
			ray,
			finalQuad.geomID,
			finalQuad.primID,NULL)) return true; 

	if (occludedTri(finalQuad.vtx[2],
			finalQuad.vtx[3],
			finalQuad.vtx[0],
			ray,
			finalQuad.geomID,
			finalQuad.primID,NULL)) return false;
      }
    else
      {
	RegularCatmullClarkPatch subpatches[4];
	patch.subdivide(subpatches);
	for (size_t i=0;i<4;i++)
	  if (intersectBounds(pre,ray,subpatches[i].bounds()))
	    if (subdivide_occluded1(pre,
				    ray,
				    subpatches[i],
				    subdiv_level - 1)) return true;
      }   
    return false;
  }

};
