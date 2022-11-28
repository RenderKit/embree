// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "RTASFile.h"
#include "rtas_loader.h"
#include "texture.h"
#include "d3d12.h"
#include "RTASToEmbree.h"

namespace embree
{
  /*! Three-index vertex, indexing start at 0, -1 means invalid vertex. */
  struct Vertex {
    unsigned int v, vt, vn;
    Vertex() {};
    Vertex(unsigned int v) : v(v), vt(v), vn(v) {};
    Vertex(unsigned int v, unsigned int vt, unsigned int vn) : v(v), vt(vt), vn(vn) {};
  };


  class RTASLoader
  {
  public:

    /*! Constructor. */
    RTASLoader(const FileName& fileName,  const bool combineIntoSingleObject);
 
    /*! output model */
    Ref<SceneGraph::GroupNode> group;
  
  private:

    /*! file to load */
    FileName path;
  
    /*! Geometry buffer. */
    avector<Vec3fa> v;
    avector<Vec3fa> vn;

    /*! Material handling. */
    std::string curMaterialName;
    Ref<SceneGraph::MaterialNode> curMaterial;
    std::map<std::string, Ref<SceneGraph::MaterialNode> > material;
    std::map<std::string, std::shared_ptr<Texture>> textureMap; 

  };

  static RTASFile::D3D12_RAYTRACING_GEOMETRY_DESC convertGeoDesc( RTASFile::Geo& geo, const Ref<SceneGraph::TriangleMeshNode>& mesh, size_t geomID )
  {
    RTASFile::D3D12_RAYTRACING_GEOMETRY_DESC dxr_desc;
    memset(&dxr_desc,0,sizeof(dxr_desc));

    // translate flags
    size_t flags = RTASFile::D3D12_RAYTRACING_GEOMETRY_FLAG_NONE;
    if (geo.Flags & (uint8_t) RTASFile::GEOMETRY_FLAG_OPAQUE)
        flags |= RTASFile::D3D12_RAYTRACING_GEOMETRY_FLAG_OPAQUE;
    if (geo.Flags & (uint8_t) RTASFile::GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION)
        flags |= RTASFile::D3D12_RAYTRACING_GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION;
    
    dxr_desc.Flags = (RTASFile::D3D12_RAYTRACING_GEOMETRY_FLAGS) flags;
    dxr_desc.ShaderIndex = geomID;
    dxr_desc.Mask = 0xff;

    if( geo.Type == RTASFile::GEOMETRY_TYPE_PROCEDURAL )
    {
        RTASFile::GeometryProcedural& boxes = geo.Desc.Procedural;
        
        // our builder requires a tight AABB array
        RTASFile::CompactAABBs((RTASFile::AABB*) boxes.pAABBs, boxes.pAABBs, boxes.AABBCount, boxes.AABBByteStride);

        dxr_desc.AABBs.AABBCount            = boxes.AABBCount;
        dxr_desc.AABBs.AABBs.StrideInBytes  = sizeof(RTASFile::AABB);
        dxr_desc.AABBs.AABBs.StartAddress   = boxes.pAABBs;
    }
    else
    {
        RTASFile::GeometryTriangles& tris = geo.Desc.Triangles;

        dxr_desc.Type = RTASFile::D3D12_RAYTRACING_GEOMETRY_TYPE_TRIANGLES;

        size_t nPrims = mesh->numPrimitives();
        
        dxr_desc.Triangles.IndexFormat = RTASFile::DXGI_FORMAT_R32_UINT;
        dxr_desc.Triangles.IndexCount  = 3*(unsigned)nPrims;
        dxr_desc.Triangles.IndexBuffer = (void*) mesh->triangles.data();
       
        // translate transform
        dxr_desc.Triangles.Transform = tris.pTransform;
       
        dxr_desc.Triangles.VertexBuffer.StrideInBytes = 16;
        dxr_desc.Triangles.VertexBuffer.StartAddress = (void*) mesh->positions[0].data();
        dxr_desc.Triangles.VertexCount = tris.VertexCount;
        dxr_desc.Triangles.VertexFormat = RTASFile::DXGI_FORMAT_R32G32B32_FLOAT;

        // dxr_desc.Triangles.TimeStepCount = 1;
        // dxr_desc.Triangles.StartTime = 0.0f;
        // dxr_desc.Triangles.EndTime = 1.0f;
    }

    return dxr_desc;
  }


  RTASLoader::RTASLoader(const FileName &sceneFilename, const bool combineIntoSingleObject) 
    : group(new SceneGraph::GroupNode), path(sceneFilename.path())
  {
    PING;

    RTASFile::RTASDataSet rtas_data;
    if (sceneFilename != "")
    {
      // if (sceneFilename.ext() == "scn")
      //   scene->add(embree::SceneGraph::loadRTScene(sceneFilename));
      if( sceneFilename.ext() == "rtas" )
      {
          if ( !RTASFile::DeserializeDataSetFromFile( &rtas_data, sceneFilename.c_str() ) )
              throw std::runtime_error( std::string("Couldn't load rtas from: ") + sceneFilename.str() );
      }
      else
        FATAL("HERE");
    }
#if 0
    // flatten rtas data
    PRINT("FLATTEN RTAS MODEL");
    RTASFile::RTASDataSet flattened;
    RTASFile::FlattenDXRScene(&data, rtas_data.pRTAS );
    RTASFile::FreeDataSet(&rtas_data);
    rtas_data = flattened;
#else
    RTASFile::RTASDataSet flattened;
    RTASFile::RTAS* pTLAS = nullptr;
    for (size_t i = 0; i < rtas_data.nRTAS; ++i)
    {
      if (rtas_data.pRTAS[i].NumInstances) {
        pTLAS = &rtas_data.pRTAS[i];
        break;
      }
    }
    if (pTLAS)
    {
      RTASFile::FlattenDXRScene(&flattened, pTLAS);
      RTASFile::FreeDataSet(&rtas_data);
    }
    else
    {
      if (rtas_data.nRTAS > 1) {
        FATAL(":..more than 1 blas and no tlas in data Set. How are we supposed to flatten that? \n");
      }
      // single blas
      flattened = rtas_data;
    }      
#endif    
    size_t hg_offset = 0;
    std::vector<Ref<SceneGraph::GroupNode>> blscenes;

    // RTAS[0] can be a TLAS without geos.
    size_t blas_start_idx = flattened.pRTAS[0].NumGeos>0 ? 0 : 1;
    PRINT(flattened.nRTAS);
    PRINT(blas_start_idx);
    for( size_t i= blas_start_idx; i<flattened.nRTAS; i++ )
    {
      RTASFile::RTAS& rtas = flattened.pRTAS[i];
      assert( rtas.NumInstances == 0 );

      // convert the geos
      std::vector<RTASFile::D3D12_RAYTRACING_GEOMETRY_DESC> geoms;
      std::vector<Ref<SceneGraph::TriangleMeshNode> > meshes;
      //Ref<SceneGraph::GroupNode> group = new SceneGraph::GroupNode;

      //PRINT(rtas.NumGeos);
      for (size_t g = 0; g < rtas.NumGeos; g++)
      {
        RTASFile::Geo& geo = rtas.pGeos[g];
            
        Ref<SceneGraph::TriangleMeshNode> mesh = RTASFile::GeoToEmbree(geo);

        RTASFile::D3D12_RAYTRACING_GEOMETRY_DESC dxr_geo = convertGeoDesc( geo, mesh, g );
        size_t numPrimitives = RTASFile::CountGeoPrimitives( geo );
        //PRINT(numPrimitives);
        
        geoms.push_back(dxr_geo);
        meshes.push_back(mesh);
        group->add(mesh.dynamicCast<SceneGraph::Node>());
      }
      blscenes.push_back(group);
    }
    PRINT("DONE");
    
  }

   
  Ref<SceneGraph::Node> loadRTAS(const FileName& fileName, const bool combineIntoSingleObject) {
    RTASLoader loader(fileName,combineIntoSingleObject); 
    return loader.group.cast<SceneGraph::Node>();
  }
}

