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

#include "hair_loader.h"

#define CONVERT_TO_BINARY 0

namespace embree
{
  const int hair_bin_magick = 0x12EF3F90;

  size_t loadHairASCII(const FileName& fileName, Ref<SceneGraph::HairSetNode> hairset)
  {  
    /* open hair file */
    FILE* f = fopen(fileName.c_str(),"r");
    if (!f) THROW_RUNTIME_ERROR("could not open " + fileName.str());

    try
    {
      char line[10000];
      if (fgets(line,10000,f) != line)
        THROW_RUNTIME_ERROR("error reading line from file " + fileName.str());
      size_t numCurves = 0;
      
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
          if (fgets(line,10000,f) != line)
            THROW_RUNTIME_ERROR("error reading line from file " + fileName.str());
          
          const unsigned vertex_start_id = (unsigned) hairset->numVertices();
          
          unsigned int id = 0;
          for (size_t i=0; i<points; i++)
          {
            if (fgets(line,10000,f) != line)
              THROW_RUNTIME_ERROR("error reading line from file " + fileName.str());
            
            /* comment */
            if (line[0] == '#' || !strncmp(line," Tracks:",strlen(" Tracks:")))
              continue;
            
            Vec3fa v;
            if (i == 0) sscanf(line,"%d : Bezier %f %f %f %f",&id,&v.x,&v.y,&v.z,&v.w);
            else        sscanf(line,"%d : %f %f %f %f",&id,&v.x,&v.y,&v.z,&v.w);
            //printf("%d %d : %f %f %f %f \n",id,vertex_start_id+id,v.x,v.y,v.z,v.w);		
            hairset->positions[0].push_back(v);
          }
          
          /* add indices to hair starts */
          for (unsigned i=0; i<points-1; i+=3)
            hairset->hairs.push_back(SceneGraph::HairSetNode::Hair(unsigned(vertex_start_id + i),unsigned(numCurves)));
          
          if (id != points-1) 
            THROW_RUNTIME_ERROR("hair parsing error");
          
          numCurves++;
        }
      }
      fclose(f);
      return numCurves;
    }
    catch (...) {
      fclose(f);
      throw;
    }
  }

  size_t loadHairBin(const FileName& fileName, Ref<SceneGraph::HairSetNode> hairset)
  {  
    FILE* fin = fopen(fileName.c_str(),"rb");
    if (!fin) THROW_RUNTIME_ERROR("could not open " + fileName.str());
    try {
      int magick; 
      if (fread(&magick,sizeof(int),1,fin) != 1)
        THROW_RUNTIME_ERROR("invalid binary hair file " + fileName.str());
      if (magick != hair_bin_magick)
        THROW_RUNTIME_ERROR("invalid binary hair file " + fileName.str());
      unsigned int numHairs, numPoints, numSegments; 
      if (fread(&numHairs,sizeof(int),1,fin) != 1) 
        THROW_RUNTIME_ERROR("invalid binary hair file " + fileName.str());
      if (fread(&numPoints,sizeof(int),1,fin) != 1)
        THROW_RUNTIME_ERROR("invalid binary hair file " + fileName.str());
      if (fread(&numSegments,sizeof(int),1,fin) != 1)
        THROW_RUNTIME_ERROR("invalid binary hair file " + fileName.str());
      hairset->positions[0].resize(numPoints);
      hairset->hairs.resize(numSegments);
      if (numPoints) {
        if (fread(hairset->positions[0].data(),sizeof(Vec3fa),numPoints,fin) != numPoints)
          THROW_RUNTIME_ERROR("invalid binary hair file " + fileName.str());
      }
      if (numSegments) {
        if (fread(hairset->hairs.data(),sizeof(SceneGraph::HairSetNode::Hair),numSegments,fin) != numSegments)
          THROW_RUNTIME_ERROR("invalid binary hair file " + fileName.str());
      }
      fclose(fin);
      return numHairs;
    } 
    catch (...) {
      fclose(fin);
      throw;
    }
  }

  Ref<SceneGraph::Node> SceneGraph::loadBinHair(const FileName& fileName)
  {
    Material objmtl; new (&objmtl) OBJMaterial;
    Ref<SceneGraph::MaterialNode> material = new SceneGraph::MaterialNode(objmtl);
    Ref<SceneGraph::HairSetNode> hairset = new SceneGraph::HairSetNode(true,material,1); 
    loadHairBin(fileName,hairset);
    hairset->verify();
    return hairset.cast<SceneGraph::Node>();
  }

  Ref<SceneGraph::Node> SceneGraph::loadTxtHair(const FileName& fileName)
  {
    Material objmtl; new (&objmtl) OBJMaterial;
    Ref<SceneGraph::MaterialNode> material = new SceneGraph::MaterialNode(objmtl);
    Ref<SceneGraph::HairSetNode> hairset = new SceneGraph::HairSetNode(true,material,1); 
    size_t numHairs MAYBE_UNUSED = loadHairASCII(fileName,hairset);
    hairset->verify();

#if CONVERT_TO_BINARY
    size_t numPoints = hairset->numVertices();
    size_t numSegments = hairset->hairs.size();
    FILE* fout = fopen(fileName.setExt(".bin").c_str(),"wb");
    if (!fout) THROW_RUNTIME_ERROR("could not open " + fileName.str());
    fwrite(&hair_bin_magick,sizeof(int),1,fout);
    fwrite(&numHairs,sizeof(int),1,fout);
    fwrite(&numPoints,sizeof(int),1,fout);
    fwrite(&numSegments,sizeof(int),1,fout);
    if (numPoints) fwrite(hairset->positions[0].data(),sizeof(Vec3fa),numPoints,fout);
    if (numSegments) fwrite(hairset->hairs->data(),sizeof(SceneGraph::HairSet::Hair),numSegments,fout);
    fclose(fout);
#endif
    
    return hairset.cast<SceneGraph::Node>();
  }
}
