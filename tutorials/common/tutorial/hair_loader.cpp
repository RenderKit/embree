// ======================================================================== //
// Copyright 2009-2013 Intel Corporation                                    //
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

#include "hair_loader.h"

namespace embree
{
  void loadHair(const FileName& fileName, OBJScene& scene, const Vec3fa& offset)
  {
    /* open hair file */
    FILE* f = fopen(fileName.c_str(),"r");
    if (!f) throw std::runtime_error("could not open " + fileName.str());

    /* add new hair set to scene */
    OBJScene::HairSet* hairset = new OBJScene::HairSet;
    scene.hairsets.push_back(hairset);

    char line[10000];
    fgets(line,10000,f);
    int curveID = 0;
    
    while (fgets(line,10000,f) && !feof(f))
    {
      /* comment */
      if (line[0] == '#')
	continue;
      
      if (!strncmp(line,"Curve:",strlen("Curve:")))
      {
        char name[1000];
        unsigned int tracks, points;
        sscanf(line,"Curve: %s %d Tracks %d Points",name,&tracks,&points);

        /* skip Tracks line */
        fgets(line,10000,f);
        
        const int vertex_start_id = hairset->v.size();
        
        double x,y,z,w;
        unsigned int id = 0;
        for (int i=0; i<points; i++)
        {
          fgets(line,10000,f);

          /* comment */
          if (line[0] == '#' || !strncmp(line," Tracks:",strlen(" Tracks:")))
            continue;

          Vec3fa v;
          if (i == 0) sscanf(line,"%d : Bezier %f %f %f %f",&id,&v.x,&v.y,&v.z,&v.w);
          else        sscanf(line,"%d : %f %f %f %f",&id,&v.x,&v.y,&v.z,&v.w);
          //printf("%d %d : %f %f %f %f \n",id,vertex_start_id+id,v.x,v.y,v.z,v.w);		
          v.x-=offset.x;
          v.y-=offset.y;
          v.z-=offset.z;
          hairset->v.push_back(v);
        }
        
        /* add indices to hair starts */
        for (int i=0; i<points-1; i+=3)
          hairset->hairs.push_back(OBJScene::Hair(vertex_start_id + i,curveID));
	
        if (id != points-1) 
          throw std::runtime_error("hair parsing error");

        curveID++;
      }
    }
    fclose(f);
  }
}

  
