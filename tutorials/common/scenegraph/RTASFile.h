// ======================================================================== //
//                                                                          //
// Copyright 2009-2019 Intel Corporation                                    //
//                                                                          //
// This program is the sole property of the Intel Corporation and           //
// contains proprietary and confidential information. Unauthorized          //
// use or distribution will be subject to action as prescribed by           //
// the license agreement.                                                   //
//                                                                          //
// ======================================================================== //

// Self-contained library for read/write/manipulation of RTAS data in an API-agnostic form
// This is intended to be a single-file library which can be dropped into tools and test scripts

#pragma once

// NOTE:  If migrating RTASFile into a non-GRL codebase, this include may be removed and replaced with:  #define GRL_CALL
#include "GRLDefines.h"


// *** DO NOT ***  include additional headers here!!!!
//
//  RTASFile is intended to be portable to other codebases, and must remain self-contained
//

#include <cstdint>
#include <cstddef>
#include <cstring>

namespace RTASFile
{
    struct RTAS;

    // NOTE: Flag and enum values are embedded in serialized blobs and MUST NOT BE CHANGED, only extended

    enum InstanceFlags : uint8_t
    {
        INSTANCE_FLAG_TRIANGLE_CULL_DISABLE           = 0x1,
        INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE = 0x2,
        INSTANCE_FLAG_FORCE_OPAQUE                    = 0x4,
        INSTANCE_FLAG_FORCE_NON_OPAQUE                = 0x8,
    };

    enum RTASFlags : uint8_t
    {
        BUILD_FLAG_ALLOW_UPDATE         = 0x01,
        BUILD_FLAG_ALLOW_COMPACTION     = 0x02,
        BUILD_FLAG_PREFER_FAST_TRACE    = 0x04,
        BUILD_FLAG_PREFER_FAST_BUILD    = 0x08,
        BUILD_FLAG_MINIMIZE_MEMORY      = 0x10,
    };

    enum GeometryFlags : uint8_t
    {
        GEOMETRY_FLAG_OPAQUE = 0x1,
        GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION = 0x2,
    };

    enum IndexFormat : uint8_t
    {
        INDEX_FORMAT_NONE      = 0,         // INDEX_FORMAT_NONE Indicates non-indexed geometry
        INDEX_FORMAT_R16_UINT  = 2,
        INDEX_FORMAT_R32_UINT  = 4,
    };

    inline size_t GetBytesPerIndex( IndexFormat e ) { return e; }


    enum GeometryType : uint8_t
    {
        GEOMETRY_TYPE_TRIANGLES     = 0,
        GEOMETRY_TYPE_PROCEDURAL    = 1,

        NUM_GEOMETRY_TYPES = 2
    };

    enum VertexFormat : uint8_t
    {
        VERTEX_FORMAT_R32G32_FLOAT          = 0,
        VERTEX_FORMAT_R32G32B32_FLOAT       = 1,
        VERTEX_FORMAT_R16G16_FLOAT          = 2,
        VERTEX_FORMAT_R16G16B16A16_FLOAT    = 3,
        VERTEX_FORMAT_R16G16_SNORM          = 4,
        VERTEX_FORMAT_R16G16B16A16_SNORM    = 5,
        VERTEX_FORMAT_R16G16B16A16_UNORM    = 6,
        VERTEX_FORMAT_R16G16_UNORM          = 7,
        VERTEX_FORMAT_R10G10B10A2_UNORM     = 8,
        VERTEX_FORMAT_R8G8B8A8_UNORM        = 9,
        VERTEX_FORMAT_R8G8_UNORM            = 10,
        VERTEX_FORMAT_R8G8B8A8_SNORM        = 11,
        VERTEX_FORMAT_R8G8_SNORM            = 12
    };

    struct Matrix
    {
        float    Transform[3][4];

        const Matrix scale(float factor) const
        {
            return
            {
              this->Transform[0][0] * factor, 0.0f, 0.0f, 0.0f,
              0.0f, this->Transform[1][1] * factor, 0.0f, 0.0f,
              0.0f, 0.0f, this->Transform[2][2] * factor, 0.0f
            };
        }

        static const Matrix identityMatrix()
        {
            return
            {
                1.0f, 0.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f, 0.0f,
                0.0f, 0.0f, 1.0f, 0.0f,
            };
        }

        Matrix& operator*= (float value)
        {
            for (size_t row = 0; row < 3; row++)
            {
                for (size_t column = 0; column < 4; column++)
                {
                    this->Transform[row][column] *= value;
                }
            }

            return *this;
        }
    };

    struct AABB
    {
        float Min[3];
        float Max[3];
    };

    struct RTAS;

    struct Instance
    {
        Matrix   Transform;
        RTAS*    pRTAS;         // If non-null, this must point into the RTAS array for serialization to work correctly
        uint64_t RTASID;
        uint32_t InstanceID;
        uint32_t InstanceContributionToHitGroupIndex;
        uint8_t  InstanceMask;
        uint8_t  Flags;
    };

    struct GeometryTriangles
    {
        Matrix*        pTransform;
        void*          pIndexBuffer;
        void*          pVertexBuffer;
        uint64_t       VertexBufferByteStride;
        uint32_t       IndexCount;
        uint32_t       VertexCount;
        IndexFormat    indexFormat;
        VertexFormat   vertexFormat;
    };

    struct GeometryProcedural
    {
        void*    pAABBs;
        uint64_t AABBByteStride;
        uint32_t AABBCount;
    };

    struct Geo
    {
        union
        {
            GeometryTriangles Triangles;
            GeometryProcedural Procedural;
        } Desc;

        GeometryType Type;
        uint8_t Flags;
    };

    struct ProceduralInstanceInfo
    {
        void* pAABBs;
        uint32_t AABBByteStride;
        uint32_t AABBCount;
        uint8_t* pIsProcedural; // One byte per instance, non-zero if the instance is procedural

        AABB GetBox( size_t idx )
        {
            char* boxes = (char*)pAABBs;
            boxes = boxes + idx*AABBByteStride;
            return *(AABB*)(boxes);
        }
    };

    struct RTAS
    {
        // current contents
        Geo*      pGeos = nullptr;
        Instance* pInstances = nullptr;
        uint32_t  NumGeos = 0;
        uint32_t  NumInstances = 0;

        // Todo: Initial struct are here for backward compatibility.
        //       We need to ditch them in the future.
        // initial contents... if different from current contents
        uint32_t NumInitialGeos = 0;
        uint32_t NumInitialInstances = 0;
        Geo* pInitialGeos = nullptr;
        Instance* pInitialInstances = nullptr;

        ProceduralInstanceInfo* pInitialProceduralsInfo = nullptr;
        ProceduralInstanceInfo* pProceduralsInfo = nullptr;

        uint8_t   Flags = 0;
        uint8_t   WasCompacted = 0;

        uint64_t srcID = 0;

        const bool IsUpdate() const
        {
            return srcID != 0;
        }

        uint32_t GetNumBuildGeos()
        {
            return NumInitialGeos > 0 ? NumInitialGeos : NumGeos;
        }

        uint32_t GetNumBuildInstances()
        {
            return NumInitialInstances > 0 ? NumInitialInstances : NumInstances;
        }

        uint32_t GetNumUpdateGeos()
        {
            return NumGeos;
        }

        uint32_t GetNumUpdateInstances()
        {
            return NumInstances;
        }

        Geo* GetBuildGeos()
        {
            return NumInitialGeos > 0 ? pInitialGeos : pGeos;
        }

        Instance* GetBuildInstances()
        {
            return NumInitialInstances > 0 ? pInitialInstances : pInstances;
        }

        Geo* GetUpdateGeos()
        {
            return NumInitialGeos > 0 ? pGeos : nullptr;
        }

        Instance* GetUpdateInstances()
        {
            return NumInitialInstances > 0 ? pInstances : nullptr;
        }

        ProceduralInstanceInfo* GetProceduralInstanceInfo()
        {
            return NumInitialInstances > 0 ? pInitialProceduralsInfo : pProceduralsInfo;
        }
    };


    // API-agnostic description of a serialized RTAS
    struct SerializedRTAS
    {
        enum Flags
        {
            FLAG_DXR    = 1, // Blob was produced by a DXR device
            FLAG_VULKAN = 2, // Blob was produced by a Vulkan device
            FLAG_GRL    = 4, // Blob was produced by an implementation of GRL and we can read it
        };

        //  All the APIs store BLAS data in  something like the following form:
        //
        //   ------------------------
        //   |
        //   |    API-specific Header
        //   |       Contains serializedSize, DeserializedSize, and BLAS handle count
        //   |
        //   -----
        //   |  *
        //   |  *   Blas GPUVA array (64b each)
        //   |  *
        //   -----------
        //   |
        //   |       OPAQUE RTAS DATA
        //   |
        //   |
        //   V
        //

        uint64_t OriginalHandle;    // Original handle (GPUVA) of this RTAS when it was serialized
        uint64_t Flags;
        uint64_t BlobSize;          // Size of pRawBlob in bytes
        uint64_t DeserializedSize;  // Size of resulting deserialized RTAS in bytes
        uint64_t NumBLASHandles;
        uint64_t BlasHandleOffset;  // Distance in bytes from start of blob to handle array

        uint64_t RTASDataSize;      // Size in bytes of the RTAS data
        uint64_t RTASDataOffset;    // Distance in bytes from start of blob to start of RTAS data
                                    //  (skipping header and BLAS handles)

        uint64_t DriverIdentifierOffset;  // Distance in bytes from start of blob to API-specific "driver identifier"
        uint64_t DriverIdentifierSize;    // Size in bytes of driver identifier

        uint64_t BatchId;

        uint8_t* pRawBlob;
    };



    struct RTASDataSet
    {
        RTAS* pRTAS = nullptr;
        size_t nRTAS = 0;

        SerializedRTAS* pSerializedRTAS = nullptr;
        size_t nSerializedRTAS = 0;

        void* pOpaqueAllocation = nullptr;  // If non-null, points to a memory allocation for the dataset.   'FreeDeserializedDataSet must be called to prevent leaks'
    };

    class IStreamWriter
    {
    public:
        /// Write specified number of bytes.   Failure handling is up to the implementation
        virtual void GRL_CALL Write( const void* pBytes, size_t nBytes ) = 0;
    };

    class IStreamReader
    {
    public:
        /// Read specified number of bytes.  Return number of bytes successfully read
        virtual size_t GRL_CALL Read( void* pBytes, size_t nBytes ) = 0;
    };

    void GRL_CALL SerializeDataSet(IStreamWriter* pWriter, const RTAS* pRTAS, size_t nRTAS);
    void GRL_CALL SerializeDataSet(IStreamWriter* pWriter, const RTAS* pRTAS, size_t nRTAS, const SerializedRTAS* pSerialized, size_t nSerialized);

    bool GRL_CALL SerializeDataSetToFile( const RTAS* pRTAS, size_t nRTAS, const SerializedRTAS* pSerialized, size_t nSerialized, const char* path );
    inline bool GRL_CALL SerializeDataSetToFile(const RTAS* pRTAS, size_t nRTAS, const char* path)
    {
        return SerializeDataSetToFile(pRTAS, nRTAS, nullptr, 0, path);
    }
    inline bool GRL_CALL SerializeSerializedRTASToFile(const SerializedRTAS* pSerialized, size_t nSerialized, const char* path)
    {
        return SerializeDataSetToFile(nullptr, 0, pSerialized, nSerialized, path);
    }

    bool GRL_CALL DeserializeDataSet( RTASDataSet* pResult, IStreamReader* pReader );
    bool GRL_CALL DeserializeDataSetFromFile( RTASDataSet* pResult, const char* path );

    bool GRL_CALL IsRTASFile( const char* path );

    void GRL_CALL FreeDataSet( RTASDataSet* pData );

    // Create an exact duplicate of a data set.  Caller must use 'FreeDataSet' to cleanup the memory
    void GRL_CALL CloneDataSet( RTASDataSet* pDest, const RTASDataSet* pSrc );

    void GRL_CALL DeleteGeometryByType( RTAS* pRTAS, size_t nRTAS, GeometryType eType );

    void GRL_CALL ConvertIndicesToU32(uint32_t* pDst, IndexFormat eSrcFormat, const void* pSrcData, size_t nIndices );

    void GRL_CALL ConvertVerticesToF32a(float* pDst, VertexFormat eSrcFormat, const void* pSrcData, size_t nVerts, size_t nSrcStride);

    size_t GRL_CALL CountGeoPrimitives( const Geo& geo );

    void GRL_CALL CompactAABBs(AABB* pDst, const void* pSrc, size_t nAABBs, size_t nSrcStride);


    void GRL_CALL FlattenDXRScene( RTASDataSet* pDataSet, RTAS* pTlas );

    void GRL_CALL CreateProceduralDataSetFromTriangleDataSet(RTASDataSet* pResult, RTASDataSet* pSource);

    void GRL_CALL TransformPosition( float* pDst, const float* pSrc, const Matrix& m );

    Matrix GRL_CALL IdentityMatrix();


    Matrix MatrixRotateX( float degrees ) ;
    Matrix MatrixRotateY( float degrees ) ;
    Matrix MatrixRotateZ( float degrees ) ;
    Matrix MatrixTranslation( float x, float y, float z );
    Matrix MatrixScale( float scale );


    // Deduplicate RTAS data buffers from 'input' and put the result in 'output'
    //  Caller must use 'FreeDataSet'  to cleanup the output memory
    void DeduplicateBuffers( RTASDataSet* output, const RTASDataSet* input );


    bool CompareRTASDataSet( const RTASFile::RTASDataSet* a, const RTASFile::RTASDataSet* b );
    bool CompareRTAS( const RTASFile::RTAS* a, const RTASFile::RTAS* b );
    bool CompareInstances( const RTASFile::Instance* a, const RTASFile::Instance* b );
    bool CompareGeos( const RTASFile::Geo* a, const RTASFile::Geo* b );
    bool CompareTriangles( const RTASFile::GeometryTriangles& a, const RTASFile::GeometryTriangles& b );
    bool CompareProcedurals( const RTASFile::GeometryProcedural& a, const RTASFile::GeometryProcedural& b );


    void DeIndexTriangles( RTASDataSet* output, const RTASDataSet* input );

}
