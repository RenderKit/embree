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

#ifndef __EMBREE_BUILDER_REGISTRY_H__
#define __EMBREE_BUILDER_REGISTRY_H__

#include "default.h"
#include "primitive.h"
#include "common/accel.h"
#include "builders/builder.h"
#include "builders/build_source.h"
#include "rtcore/scene.h"

namespace embree
{
  /*! Registry for acceleration structure builders */
  class BuilderRegistry 
  {
  public:
    typedef Builder* (*BuilderConstruction)(void* accel, BuildSource* source, void* geometry, const size_t maxLeafSize, const size_t forceLeafSize);
   
    struct BuildClosure 
    {
      BuildClosure () {}
      BuildClosure (int cpu_features, BuilderConstruction create, const size_t minLeafSize, const size_t maxLeafSize)
      : cpu_features(cpu_features), create(create), minLeafSize(minLeafSize), maxLeafSize(maxLeafSize) {}

      static bool compare (const BuildClosure& a, const BuildClosure& b) {
        return a.cpu_features > b.cpu_features;
      }

    public:
      int cpu_features;
      BuilderConstruction create;
      size_t minLeafSize;
      size_t maxLeafSize;
    };
        
  public:

    /*! adds a new builder to the registry */
    void add(int cpu_features, const std::string& name, BuilderConstruction builder, const size_t minLeafSize = 1, const size_t maxLeafSize = inf);
      
    /*! builds an acceleration structure using the specified builder */
    Builder* get(std::string builderName, void* accel, BuildSource* source, void* geometry);
    
    /*! prints the registry */
    void print();

    /*! clears the registry */
    void clear();

  private:
    std::map<std::string, std::vector<BuildClosure> > table; 
  }; 

  /*! builder registry */
  extern BuilderRegistry builders; 

#define ADD_BUILDER(NAME,BUILDER,LEAFMIN,LEAFMAX)              \
  builders.add(ISA,NAME,BUILDER,LEAFMIN,LEAFMAX);

  typedef Builder* (*TriangleMeshBuilderFunc)(void* accel, TriangleMeshScene::TriangleMesh* mesh, const size_t minLeafSize, const size_t maxLeafSize);
  typedef Builder* (*BuilderFunc)            (void* accel, BuildSource* source, Scene* scene, const size_t minLeafSize, const size_t maxLeafSize);

#define DECLARE_TRIANGLEMESH_BUILDER(symbol)                            \
  namespace isa   { extern Builder* symbol(void* accel, TriangleMeshScene::TriangleMesh* mesh, const size_t minLeafSize, const size_t maxLeafSize); } \
  namespace sse41 { extern Builder* symbol(void* accel, TriangleMeshScene::TriangleMesh* mesh, const size_t minLeafSize, const size_t maxLeafSize); } \
  namespace avx   { extern Builder* symbol(void* accel, TriangleMeshScene::TriangleMesh* mesh, const size_t minLeafSize, const size_t maxLeafSize); } \
  namespace avx2  { extern Builder* symbol(void* accel, TriangleMeshScene::TriangleMesh* mesh, const size_t minLeafSize, const size_t maxLeafSize); } \
  TriangleMeshBuilderFunc symbol;
  
#define DECLARE_BUILDER(symbol)                                         \
  namespace isa   { extern Builder* symbol(void* accel, BuildSource* source, Scene* scene, const size_t minLeafSize, const size_t maxLeafSize); } \
  namespace sse41 { extern Builder* symbol(void* accel, BuildSource* source, Scene* scene, const size_t minLeafSize, const size_t maxLeafSize); } \
  namespace avx   { extern Builder* symbol(void* accel, BuildSource* source, Scene* scene, const size_t minLeafSize, const size_t maxLeafSize); } \
  namespace avx2  { extern Builder* symbol(void* accel, BuildSource* source, Scene* scene, const size_t minLeafSize, const size_t maxLeafSize); } \
  BuilderFunc symbol;
}

#endif

