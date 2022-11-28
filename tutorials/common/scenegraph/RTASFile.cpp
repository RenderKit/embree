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
//
// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) Contributors to the OpenEXR Project.
//

#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif

#include "RTASFile.h"

// *** DO NOT ***  include additional headers here!!!!
//
//  RTASFile is intended to be portable to other codebases, and must remain self-contained
//

#include <vector>
#include <algorithm>
#include <unordered_map>
#include <assert.h>
#include <math.h>
#include <iostream>
#include <map>
#include <memory>


#include <stdlib.h>
#if defined(__WIN32__)
#include <intrin.h>
#endif


namespace RTASFile
{
    static_assert( INSTANCE_FLAG_TRIANGLE_CULL_DISABLE           == (RTASFile::InstanceFlags) 0x1, "Flags are embedded in serialized files and must not be changed");
    static_assert( INSTANCE_FLAG_TRIANGLE_FRONT_COUNTERCLOCKWISE == (RTASFile::InstanceFlags) 0x2, "Flags are embedded in serialized files and must not be changed");
    static_assert( INSTANCE_FLAG_FORCE_OPAQUE                    == (RTASFile::InstanceFlags) 0x4, "Flags are embedded in serialized files and must not be changed");
    static_assert( INSTANCE_FLAG_FORCE_NON_OPAQUE                == (RTASFile::InstanceFlags) 0x8, "Flags are embedded in serialized files and must not be changed");

    static_assert(  BUILD_FLAG_ALLOW_UPDATE         == (RTASFile::RTASFlags) 0x01,  "Flags are embedded in serialized files and must not be changed");
    static_assert(  BUILD_FLAG_ALLOW_COMPACTION     == (RTASFile::RTASFlags) 0x02,  "Flags are embedded in serialized files and must not be changed");
    static_assert(  BUILD_FLAG_PREFER_FAST_TRACE    == (RTASFile::RTASFlags) 0x04,  "Flags are embedded in serialized files and must not be changed");
    static_assert(  BUILD_FLAG_PREFER_FAST_BUILD    == (RTASFile::RTASFlags) 0x08,  "Flags are embedded in serialized files and must not be changed");
    static_assert(  BUILD_FLAG_MINIMIZE_MEMORY      == (RTASFile::RTASFlags) 0x10,  "Flags are embedded in serialized files and must not be changed");

    static_assert( GEOMETRY_FLAG_OPAQUE                         == (RTASFile::GeometryFlags) 0x1, "Flags are embedded in serialized files and must not be changed");
    static_assert( GEOMETRY_FLAG_NO_DUPLICATE_ANYHIT_INVOCATION == (RTASFile::GeometryFlags) 0x2, "Flags are embedded in serialized files and must not be changed");

    static_assert( INDEX_FORMAT_NONE      == (RTASFile::IndexFormat) 0,   "Enums are embedded in serialized files and must not be changed");
    static_assert( INDEX_FORMAT_R16_UINT  == (RTASFile::IndexFormat) 2,   "Enums are embedded in serialized files and must not be changed");
    static_assert( INDEX_FORMAT_R32_UINT  == (RTASFile::IndexFormat) 4,   "Enums are embedded in serialized files and must not be changed");

    static_assert( GEOMETRY_TYPE_TRIANGLES     == (RTASFile::GeometryType) 0,  "Enums are embedded in serialized files and must not be changed" );
    static_assert( GEOMETRY_TYPE_PROCEDURAL    == (RTASFile::GeometryType) 1,  "Enums are embedded in serialized files and must not be changed" );

    static_assert( VERTEX_FORMAT_R32G32_FLOAT          == (RTASFile::VertexFormat)  0, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R32G32B32_FLOAT       == (RTASFile::VertexFormat)  1, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R16G16_FLOAT          == (RTASFile::VertexFormat)  2, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R16G16B16A16_FLOAT    == (RTASFile::VertexFormat)  3, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R16G16_SNORM          == (RTASFile::VertexFormat)  4, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R16G16B16A16_SNORM    == (RTASFile::VertexFormat)  5, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R16G16B16A16_UNORM    == (RTASFile::VertexFormat)  6, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R16G16_UNORM          == (RTASFile::VertexFormat)  7, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R10G10B10A2_UNORM     == (RTASFile::VertexFormat)  8, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R8G8B8A8_UNORM        == (RTASFile::VertexFormat)  9, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R8G8_UNORM            == (RTASFile::VertexFormat) 10, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R8G8B8A8_SNORM        == (RTASFile::VertexFormat) 11, "Enums are embedded in serialized files and must not be changed");
    static_assert( VERTEX_FORMAT_R8G8_SNORM            == (RTASFile::VertexFormat) 12, "Enums are embedded in serialized files and must not be changed");

    static_assert( SerializedRTAS::FLAG_DXR == 1,    "Enums are embedded in serialized files and must not be changed");
    static_assert( SerializedRTAS::FLAG_VULKAN == 2, "Enums are embedded in serialized files and must not be changed");

    namespace _INTERNAL
    {
        struct SerializationLogicalBuffer
        {
            char* Contents;
            size_t Size;
            size_t PhysicalIndex;
            size_t PhysicalOffset;
        };

        struct SerializationPhysicalBuffer
        {
            size_t Size;
            char* Contents;
        };

        namespace
        {
        namespace GRL
        {
            constexpr uint32_t BUFFER_ALIGN = 8;
            constexpr uint32_t MAGIC = 'R' |
                                       'T' << 8 |
                                       'A' << 16 |
                                       'S' << 24;
            // Version history:
            // ----------------
            //   Version 1:  Initial version
            //   Version 2:  Added optional 'SerializedRTAS' support
            //   Version 3:  Added buffer de-duplication during serialization
            //   Version 4:  Added procedural instances
            //   Version 5:  Added pointer to base RTAS and data describing RTAS structure
            //   Version 6:  Added rtas id to instances
            constexpr uint32_t VERSION = 6;
        }


            size_t  RoundBufferSize(size_t n)
        {
            return ((n + GRL::BUFFER_ALIGN - 1) / GRL::BUFFER_ALIGN) * GRL::BUFFER_ALIGN;
        }

            size_t  RoundBufferSize(size_t n, size_t align)
        {
            return align*((n + align - 1) / align);
        }


        class OStream
        {
        public:
                 OStream(IStreamWriter* pSerializer) : m_pSerializer(pSerializer) {}

                void  WriteU64(uint64_t n)
            {
                m_pSerializer->Write( &n, 8 );
            }
                void  WriteU32(uint32_t n)
            {
                m_pSerializer->Write( &n, 4 );
            }

                void  WriteTransform(const Matrix& xform)
            {
                m_pSerializer->Write( &xform, sizeof(xform) );
            }

                void  WriteRawBuffer(void* pBuffer, size_t nBytes)
            {
                if( pBuffer == nullptr )
                    WriteZeros( nBytes );
                else
                {
                    m_pSerializer->Write(pBuffer,nBytes);
                    if( nBytes % GRL::BUFFER_ALIGN)
                        WriteZeros(GRL::BUFFER_ALIGN - (nBytes % GRL::BUFFER_ALIGN) );
                }
            }

                void  WriteZeros(size_t nBytes)
            {
                char ZERO[64] = {0};
                while( nBytes >= 64 )
                {
                    m_pSerializer->Write( ZERO, 64 );
                    nBytes -= 64;
                }
                if( nBytes )
                    m_pSerializer->Write( ZERO, nBytes );
            }

        private:
            IStreamWriter* m_pSerializer;
        };


        class AllocContainer
        {
        public:

            ~AllocContainer()
            {
                for( void* p : m_Allocs )
                    free(p);
            }
          
            template< class T >
            T*  New(size_t n = 1)
            {
                return (T*) Malloc(n*sizeof(T));
            }

            void*  Malloc(size_t n)
            {
                n = RoundBufferSize(n);

                if( n > SMALLBLOCK_SIZE )
                {
                    // large allocs get their own block
                    void* p = malloc(n);
                    assert(p != nullptr);
                    m_Allocs.push_back(p);
                    return p;
                }
                else
                {
                    if( n > m_nSmallBlockLeft )
                    {
                        // start a new small-block if we're out of space
                        m_pSmallBlock = (uint8_t*) malloc(SMALLBLOCK_SIZE);
                        m_nSmallBlockLeft = SMALLBLOCK_SIZE;
                        m_Allocs.push_back(m_pSmallBlock);
                    }

                    // sub-allocate from small-block
                    void* pRet = m_pSmallBlock;
                    m_nSmallBlockLeft -= n;
                    m_pSmallBlock += n;
                    return pRet;
                }
            }

            void*  ExportAllocs()
            {
                std::vector<void*>* allocs = new std::vector<void*>( std::move(m_Allocs) );
                assert(allocs != nullptr);
                return reinterpret_cast<void*>(allocs);
            }

            static void  FreeExportedAllocs(void* p)
            {
                std::vector<void*>* allocs = reinterpret_cast< std::vector<void*>* >(p);
                for(void* a : *allocs )
                    free(a);
                delete allocs;
            }

            // make a shallow copy of a buffer and return the copy
            template< class T >
            T* ShallowCopy( const T* input, size_t n )
            {
                if ( n == 0 )
                    return nullptr;
                if ( input == nullptr )
                    return nullptr;

                T* mem = New<T>( n );
                if (mem != nullptr)
                    memcpy( mem, input, n * sizeof( T ) );
                return mem;
            }

        private:

            enum
            {
                SMALLBLOCK_SIZE = 64*1024,
            };

            uint8_t* m_pSmallBlock   = nullptr;
            size_t m_nSmallBlockLeft = 0;

            std::vector<void*> m_Allocs;
        };


        class IStream
        {
        public:

            enum
            {
                PREFETCH_LIMIT = 512
            };

                 IStream(IStreamReader* pReader)
                : m_pReader(pReader)
            {
            }

                bool  PrefetchBytes(size_t n)
            {
                assert( n <= PREFETCH_LIMIT );
                size_t nBytes = m_pReader->Read( m_PrefetchBuffer, n );
                m_nPrefetchHead = 0;
                m_nPrefetchSize = n;
                return (nBytes >= n );
            }

                uint64_t  PopU64()
            {
                uint64_t val;
                PopPrefetch(&val,sizeof(val));
                return val;
            }
                uint32_t  PopU32()
            {
                uint32_t val;
                PopPrefetch( &val, sizeof( val ) );
                return val;
            }
                void  PopTransform(Matrix* pOut)
            {
                PopPrefetch(pOut,sizeof(Matrix));
            }

                uint64_t  ReadU64()
            {
                uint64_t val;
                ReadPrefetch(&val, sizeof(val));
                return val;
            }
                uint32_t  ReadU32()
            {
                uint32_t val;
                ReadPrefetch(&val, sizeof(val));
                return val;
            }

                bool  ReadRawBuffer(void* pPlace, size_t nSize)
            {
                return m_pReader->Read( pPlace,nSize ) == nSize;
            }

        private:

            void  PopPrefetch(void* place, size_t size)
            {
                ReadPrefetch(place, size);
                m_nPrefetchHead += size;
            }

            void  ReadPrefetch(void* place, size_t size)
            {
                memcpy(place, &m_PrefetchBuffer[m_nPrefetchHead], size);
                assert(m_nPrefetchHead + size <= m_nPrefetchSize);
            }

            IStreamReader* m_pReader;
            uint8_t m_PrefetchBuffer[PREFETCH_LIMIT];
            size_t m_nPrefetchHead=0;
            size_t m_nPrefetchSize=0;

        };

        SerializationLogicalBuffer* FindBufferInSortedSet( std::vector<SerializationLogicalBuffer>& buffers, void* pContents )
        {
            auto it = std::lower_bound( buffers.begin(), buffers.end(), pContents,
                []( const SerializationLogicalBuffer& it, void* pContents ) { return it.Contents < pContents; } );

            if ( it == buffers.end() || it->Contents != pContents )
                return nullptr;
            return &(*it);
        }

        void WriteBufferReference( OStream& stream, void* pContents, std::vector<SerializationLogicalBuffer>& buffers )
        {
            auto it = FindBufferInSortedSet( buffers, pContents );
            if( it == nullptr )
            {
                assert( false );
                stream.WriteU64( 0 );
                stream.WriteU64( 0 );
            }
            else
            {
                stream.WriteU64( it->PhysicalIndex );
                stream.WriteU64( it->PhysicalOffset );
            }
        }

            void  WriteGeos( OStream& stream, const Geo* pGeos, size_t nGeos, std::vector<SerializationLogicalBuffer>& buffers )
            {
                for ( size_t j = 0; j < nGeos; j++ )
                {
                    stream.WriteU32( pGeos[j].Type );
                    stream.WriteU32( pGeos[j].Flags );

                    static_assert(NUM_GEOMETRY_TYPES == (RTASFile::GeometryType)2, "Maintain me");

                    if ( pGeos[j].Type == GEOMETRY_TYPE_TRIANGLES )
                    {
                        const GeometryTriangles& tris = pGeos[j].Desc.Triangles;

                        stream.WriteU64( tris.VertexBufferByteStride );
                        stream.WriteU32( tris.IndexCount );
                        stream.WriteU32( tris.indexFormat );
                        stream.WriteU32( tris.VertexCount );
                        stream.WriteU32( tris.vertexFormat );

                        bool have_matrix = tris.pTransform != nullptr;
                        stream.WriteU64( have_matrix ? 1 : 0 );

                        if ( have_matrix )
                            stream.WriteTransform( *tris.pTransform );

                        size_t nIndexBytes = GetBytesPerIndex( tris.indexFormat );
                        size_t nIndexBufferSize = static_cast<size_t>(nIndexBytes * tris.IndexCount);
                        if ( nIndexBufferSize )
                        {
                            WriteBufferReference( stream, tris.pIndexBuffer, buffers );
                        }

                        size_t nVertexBufferSize = static_cast<size_t>(tris.VertexCount * tris.VertexBufferByteStride);
                        if ( nVertexBufferSize )
                        {
                            WriteBufferReference( stream, tris.pVertexBuffer, buffers );
                        }
                    }
                    else if ( pGeos[j].Type == GEOMETRY_TYPE_PROCEDURAL )
                    {
                        const GeometryProcedural& boxes = pGeos[j].Desc.Procedural;
                        stream.WriteU64( boxes.AABBByteStride );
                        stream.WriteU64( boxes.AABBCount );

                        size_t nSize = static_cast<size_t>(boxes.AABBByteStride * boxes.AABBCount);
                        if ( nSize )
                        {
                            WriteBufferReference( stream, boxes.pAABBs, buffers );
                        }
                    }
                    else
                    {
                        assert( false );
                    }
                }
            }

            void  WriteInstances(OStream& stream, const Instance* pInstances, size_t nInstances, const RTAS* pRTAS, size_t nRTAS)
            {
                for(size_t j=0; j < nInstances; j++)
                {
                    // push instance
                    const Instance& instance = pInstances[j];

                    bool bIsNull = instance.pRTAS == nullptr;
                    uint32_t nIndex = 0;
                    if(!bIsNull)
                    {
                        if(instance.pRTAS < pRTAS || instance.pRTAS >= (pRTAS + nRTAS))
                        {
                            assert( !"RTAS Instances must point into the RTAS array for serialization" );
                            bIsNull = true;
                        }
                        else
                        {
                            nIndex = (uint32_t) (instance.pRTAS - pRTAS);
                        }
                    }

                    stream.WriteTransform( instance.Transform );
                    stream.WriteU32( bIsNull ? 1 : 0 );
                    stream.WriteU32( nIndex );
                    stream.WriteU64( instance.RTASID );

                    stream.WriteU32( instance.Flags );
                    stream.WriteU32( instance.InstanceContributionToHitGroupIndex );

                    stream.WriteU32( instance.InstanceID );
                    stream.WriteU32( instance.InstanceMask );
                }
            }

            bool  ReadInstances(IStream& stream, size_t nInstances, Instance* pInstances, RTAS* pRTAS, uint32_t version)
            {
                for (size_t j = 0; j < nInstances; j++)
                {
                    size_t prefetchSize = 24;
                    if (version >= 6)
                        prefetchSize = 32;
                    if (!stream.PrefetchBytes(prefetchSize + sizeof(Matrix)))
                        return false;

                    Instance& instance = pInstances[j];

                    stream.PopTransform(&instance.Transform);

                    uint32_t bRTASIsNull = stream.PopU32();
                    uint32_t nRTASIndex = stream.PopU32();
                    if (bRTASIsNull)
                        instance.pRTAS = nullptr;
                    else
                        instance.pRTAS = pRTAS + nRTASIndex;

                    if (version >= 6)
                        instance.RTASID = stream.PopU64();
                    else
                        instance.RTASID = 0;
                    instance.Flags                               = (uint8_t) stream.PopU32();
                    instance.InstanceContributionToHitGroupIndex = stream.PopU32();
                    instance.InstanceID                          = stream.PopU32();
                    instance.InstanceMask                        = (uint8_t) stream.PopU32();
                }

                return true;
            }

            bool ReadRawBuffer( void*& place, size_t nExpectedSize, AllocContainer& allocs, IStream& stream, void** ppBufferPointers )
            {
                if ( ppBufferPointers == nullptr )
                {
                    // file version 2 and under embeds all buffers directly in the stream
                    place = allocs.Malloc( nExpectedSize );
                    if ( !stream.ReadRawBuffer( place, nExpectedSize ) )
                        return false;
                }
                else
                {
                    // file version 3 and up separates out buffers
                    if ( !stream.PrefetchBytes( 16 ) )
                        return false;

                    uint64_t idx  = stream.PopU64();
                    uint64_t offs = stream.PopU64();
                    uint8_t* buffer = (uint8_t*)ppBufferPointers[idx];
                    place = buffer + offs;
                }

                return true;
            }

            bool  ReadGeos(IStream& stream, size_t nGeos, Geo* pGeos, AllocContainer& allocs, size_t nAlign, void** ppBufferPointers )
            {
                for(size_t j=0; j < nGeos; j++)
                {
                    if(!stream.PrefetchBytes( 8 ))
                        return false;

                    Geo& geo = pGeos[j];
                    geo.Type = (GeometryType) stream.PopU32();
                    geo.Flags = (uint8_t) stream.PopU32();

                    static_assert(NUM_GEOMETRY_TYPES == (RTASFile::GeometryType) 2, "Maintain me");

                    if(geo.Type == GEOMETRY_TYPE_TRIANGLES)
                    {
                        if(!stream.PrefetchBytes( 32 ))
                            return false;

                        GeometryTriangles& tris = geo.Desc.Triangles;
                        tris.VertexBufferByteStride = stream.PopU64();
                        tris.IndexCount   = stream.PopU32();
                        tris.indexFormat  = (IndexFormat) stream.PopU32();
                        tris.VertexCount  = stream.PopU32();
                        tris.vertexFormat = (VertexFormat) stream.PopU32();
                        tris.pTransform = nullptr;
                        tris.pIndexBuffer = nullptr;
                        tris.pVertexBuffer = nullptr;

                        bool have_matrix = stream.PopU64() != 0;
                        if(have_matrix)
                        {
                            if(!stream.PrefetchBytes( sizeof( Matrix ) ))
                                return false;

                            tris.pTransform = allocs.New<Matrix>();
                            if (tris.pTransform == nullptr)
                                return false;
                            stream.PopTransform( tris.pTransform );
                        }

                        size_t nIndexBytes = GetBytesPerIndex( tris.indexFormat );
                        size_t nIndexBufferSize = _INTERNAL::RoundBufferSize( static_cast<size_t>(nIndexBytes * tris.IndexCount), nAlign );
                        if(nIndexBufferSize)
                        {
                            if ( !ReadRawBuffer( tris.pIndexBuffer, nIndexBufferSize, allocs, stream, ppBufferPointers ) )
                                return false;
                        }

                        size_t nVertexBufferSize = _INTERNAL::RoundBufferSize( static_cast<size_t>(tris.VertexCount * tris.VertexBufferByteStride), nAlign );
                        if(nVertexBufferSize)
                        {
                            if ( !ReadRawBuffer( tris.pVertexBuffer, nVertexBufferSize, allocs, stream, ppBufferPointers ) )
                                return false;
                        }
                    }
                    else if(geo.Type == GEOMETRY_TYPE_PROCEDURAL)
                    {
                        if(!stream.PrefetchBytes( 16 ))
                            return false;

                        GeometryProcedural& proc = pGeos[j].Desc.Procedural;

                        size_t nAABBStride = static_cast<size_t>(stream.PopU64());
                        size_t nAABBCount  = static_cast<size_t>(stream.PopU64());
                        size_t nSize = _INTERNAL::RoundBufferSize( nAABBStride * nAABBCount, nAlign );
                        if(nSize)
                        {
                            if ( !ReadRawBuffer( proc.pAABBs, nSize, allocs, stream, ppBufferPointers ) )
                                return false;
                        }

                        proc.AABBByteStride = nAABBStride;
                        proc.AABBCount = (uint32_t) nAABBCount;
                    }
                }

                return true;
            }


            void WriteProceduralsInfo( OStream& stream, const ProceduralInstanceInfo* pInfo, std::vector<SerializationLogicalBuffer>& buffers )
            {
                stream.WriteU32( pInfo->AABBByteStride );
                stream.WriteU32( pInfo->AABBCount );
                if ( pInfo->AABBCount )
                {
                    WriteBufferReference( stream, pInfo->pAABBs, buffers );
                    WriteBufferReference( stream, pInfo->pIsProcedural, buffers );
                }
            }

            ProceduralInstanceInfo* ReadProceduralsInfo( IStream& stream, AllocContainer& allocs, size_t nAlign, void** ppBufferPointers )
            {
                stream.PrefetchBytes( 8 );

                ProceduralInstanceInfo* pInfo = allocs.New<ProceduralInstanceInfo>();
                if (pInfo == nullptr)
                {
                    return nullptr;
                }

                pInfo->AABBByteStride = stream.PopU32();
                pInfo->AABBCount      = stream.PopU32();

                if (pInfo->AABBCount)
                {
                    void* pAABBs        = nullptr;
                    void* pIsProcedural = nullptr;
                    size_t aabb_size   = _INTERNAL::RoundBufferSize( pInfo->AABBByteStride*pInfo->AABBCount, nAlign );
                    size_t isproc_size = _INTERNAL::RoundBufferSize( pInfo->AABBCount, nAlign );
                    ReadRawBuffer( pAABBs, aabb_size, allocs, stream, ppBufferPointers );
                    ReadRawBuffer( pIsProcedural, isproc_size, allocs, stream, ppBufferPointers );
                    pInfo->pAABBs = pAABBs;
                    pInfo->pIsProcedural = (uint8_t*)pIsProcedural;
                }
                return pInfo;
            }




            bool DeserializeGrlDataSet(RTASDataSet* pResult, IStream& stream, AllocContainer& allocs, uint32_t version)
            {
                if (version > _INTERNAL::GRL::VERSION)
                    return false;

                if ( version >= 3 )
                    stream.PrefetchBytes( 24 );
                else
                    stream.PrefetchBytes( 16 );

                size_t nAlign = static_cast<size_t>(stream.PopU64());
                size_t nRTAS = static_cast<size_t>(stream.PopU64());

                // read the bulk-data buffers
                void** ppBufferPointers = nullptr;
                if (version >= 3)
                {
                    size_t nBuffers = static_cast<size_t>(stream.PopU64());
                    if ( nBuffers )
                    {
                        uint64_t* pBufferSizes = nullptr;
                        pBufferSizes = reinterpret_cast<uint64_t*>(allocs.Malloc( sizeof( uint64_t ) * nBuffers ));
                        if (!pBufferSizes || !stream.ReadRawBuffer( pBufferSizes, sizeof( uint64_t ) * nBuffers ))
                            return false;

                        ppBufferPointers = reinterpret_cast<void**>(allocs.Malloc( sizeof( void* ) * nBuffers ));
                        if (!ppBufferPointers)
                            return false;
                        for ( size_t i = 0; i < nBuffers; i++ )
                        {
                            size_t n = static_cast<size_t>( pBufferSizes[i] );
                            n = _INTERNAL::RoundBufferSize( n, nAlign );
                            void* p = allocs.Malloc( n );
                            if (!stream.ReadRawBuffer( p, n ))
                                return false;
                            ppBufferPointers[i] = p;
                        }
                    }
                }

                RTAS* pRTAS = nullptr;
                if (nRTAS > 0)
                {
                    pRTAS = allocs.New<RTAS>(nRTAS);
                    if (pRTAS == nullptr)
                       return false;
                }

                for (size_t i = 0; i < nRTAS; i++)
                {
                    pRTAS[i] = RTAS{};

                    if (!stream.PrefetchBytes(24))
                        return false;

                    uint32_t nInstances = stream.PopU32();
                    uint32_t nGeos = stream.PopU32();
                    uint32_t nFlags = stream.PopU32();
                    uint32_t bCompacted = stream.PopU32();

                    pRTAS[i].Flags = (uint8_t)nFlags;
                    pRTAS[i].WasCompacted = (uint8_t)bCompacted;
                    if (nInstances > 0)
                    {
                        pRTAS[i].pInstances = allocs.New<Instance>(nInstances);
                        if (pRTAS[i].pInstances == nullptr)
                        {
                            return false;
                        }
                    }
                    else
                    {
                        pRTAS[i].pInstances = nullptr;
                    }
                    if (nGeos > 0)
                    {
                        pRTAS[i].pGeos = allocs.New<Geo>(nGeos);
                        if (pRTAS[i].pGeos == nullptr)
                        {
                            return false;
                        }
                    }
                    else
                    {
                        pRTAS[i].pGeos = nullptr;
                    }
                    pRTAS[i].NumGeos = nGeos;
                    pRTAS[i].NumInstances = nInstances;

                    uint32_t nInitialGeos = stream.PopU32();
                    uint32_t nInitialInstances = stream.PopU32();

                    _INTERNAL::ReadInstances(stream, nInstances, pRTAS[i].pInstances, pRTAS, version);
                    _INTERNAL::ReadGeos(stream, nGeos, pRTAS[i].pGeos, allocs, nAlign, ppBufferPointers );

                    if (nInitialInstances > 0)
                    {
                        pRTAS[i].pInitialInstances = allocs.New<Instance>(nInitialInstances);
                        if (pRTAS[i].pInitialInstances == nullptr)
                        {
                            return false;
                        }
                    }
                    else
                    {
                        pRTAS[i].pInitialInstances = nullptr;
                    }
                    if (nInitialGeos > 0)
                    {
                        pRTAS[i].pInitialGeos = allocs.New<Geo>(nInitialGeos);
                        if (pRTAS[i].pInitialGeos == nullptr)
                        {
                            return false;
                        }
                    }
                    else
                    {
                        pRTAS[i].pInitialGeos = nullptr;
                    }
                    pRTAS[i].NumInitialGeos = nInitialGeos;
                    pRTAS[i].NumInitialInstances = nInitialInstances;
                    if (nInitialGeos)
                    {
                        _INTERNAL::ReadGeos(stream, nInitialGeos, pRTAS[i].pInitialGeos, allocs, nAlign, ppBufferPointers);
                    }

                    if (nInitialInstances)
                    {
                        _INTERNAL::ReadInstances(stream, nInitialInstances, pRTAS[i].pInitialInstances, pRTAS, version);
                    }

                    pRTAS[i].pProceduralsInfo = nullptr;
                    pRTAS[i].pInitialProceduralsInfo = nullptr;
                    if (version >= 4)
                    {
                        stream.PrefetchBytes(8);
                        uint32_t HaveProcedurals = stream.PopU32();
                        uint32_t HaveProceduralsInitial = stream.PopU32();

                        if( HaveProcedurals )
                        {
                            pRTAS[i].pProceduralsInfo = _INTERNAL::ReadProceduralsInfo(stream, allocs, nAlign, ppBufferPointers);
                            if (pRTAS[i].pProceduralsInfo == nullptr)
                            {
                                return false;
                            }
                        }
                        if (HaveProceduralsInitial)
                        {
                            pRTAS[i].pInitialProceduralsInfo = _INTERNAL::ReadProceduralsInfo(stream, allocs, nAlign, ppBufferPointers);
                            if (pRTAS[i].pInitialProceduralsInfo == nullptr)
                            {
                                return false;
                            }
                        }
                    }
                    uint64_t srcID = 0;
                    if (version >= 5)
                    {
                        stream.PrefetchBytes(8);
                        uint64_t isUpdate = stream.PopU64();
                        if (isUpdate)
                        {
                            stream.PrefetchBytes(8);
                            srcID = stream.PopU64();
                        }
                    }

                    pRTAS[i].srcID = srcID;
                }

                // read serialized RTAS array
                SerializedRTAS* pSerializedRTAS = nullptr;
                size_t nSerializedRTAS = 0;
                if (version >= 2)
                {
                    if (!stream.PrefetchBytes(8))
                        return false;

                    nSerializedRTAS = (size_t)stream.PopU64();
                    if (nSerializedRTAS)
                    {
                        pSerializedRTAS = allocs.New<SerializedRTAS>(nSerializedRTAS);
                        if (pSerializedRTAS == nullptr)
                            return false;

                        for (size_t i = 0; i < nSerializedRTAS; i++)
                        {
                            if (!stream.PrefetchBytes(version >= 6 ? 88 : 80))
                                return false;

                            if (version >= 6)
                            {
                                pSerializedRTAS[i].BatchId = stream.PopU64();
                            }
                            else
                            {
                                pSerializedRTAS[i].BatchId = 0;
                            }

                            pSerializedRTAS[i].OriginalHandle = stream.PopU64();
                            pSerializedRTAS[i].Flags = stream.PopU64();
                            pSerializedRTAS[i].BlobSize = stream.PopU64();
                            pSerializedRTAS[i].DeserializedSize = stream.PopU64();
                            pSerializedRTAS[i].NumBLASHandles = stream.PopU64();
                            pSerializedRTAS[i].BlasHandleOffset = stream.PopU64();
                            pSerializedRTAS[i].RTASDataOffset = stream.PopU64();
                            pSerializedRTAS[i].RTASDataSize = stream.PopU64();
                            pSerializedRTAS[i].DriverIdentifierOffset = stream.PopU64();
                            pSerializedRTAS[i].DriverIdentifierSize = stream.PopU64();



                            pSerializedRTAS[i].pRawBlob = nullptr;

                            size_t nSize = _INTERNAL::RoundBufferSize((size_t)pSerializedRTAS[i].BlobSize, nAlign);
                            if (nSize)
                            {
                                pSerializedRTAS[i].pRawBlob = (uint8_t*)allocs.Malloc(nSize);
                                if (!stream.ReadRawBuffer(pSerializedRTAS[i].pRawBlob, nSize))
                                    return false;
                            }
                        }
                    }
                }

                // successful parse.  Store the output
                pResult->pRTAS = pRTAS;
                pResult->nRTAS = nRTAS;
                pResult->pSerializedRTAS = pSerializedRTAS;
                pResult->nSerializedRTAS = nSerializedRTAS;
                pResult->pOpaqueAllocation = allocs.ExportAllocs();
                return true;
            }
        } // blank namespace



        void BuildLogicalBuffers(std::vector<SerializationLogicalBuffer>& logicalBuffers, const Geo* g )
        {
            switch ( g->Type )
            {
            case GEOMETRY_TYPE_TRIANGLES:
                {
                    SerializationLogicalBuffer ib;
                    ib.Contents = (char*)g->Desc.Triangles.pIndexBuffer;
                    ib.Size = GetBytesPerIndex( g->Desc.Triangles.indexFormat ) * g->Desc.Triangles.IndexCount;

                    SerializationLogicalBuffer vb;
                    vb.Contents = (char*)g->Desc.Triangles.pVertexBuffer;
                    vb.Size = static_cast<size_t>(g->Desc.Triangles.VertexCount * g->Desc.Triangles.VertexBufferByteStride);

                    if ( ib.Size )
                        logicalBuffers.push_back( ib );
                    if ( vb.Size )
                        logicalBuffers.push_back( vb );
                }
                break;

            case GEOMETRY_TYPE_PROCEDURAL:
                if ( g->Desc.Procedural.AABBCount > 0 )
                {
                    SerializationLogicalBuffer lb;
                    lb.Contents = (char*)g->Desc.Procedural.pAABBs;
                    lb.Size = static_cast<size_t>(g->Desc.Procedural.AABBCount * g->Desc.Procedural.AABBByteStride);
                    logicalBuffers.push_back( lb );
                }
                break;
            default:
                assert( false );
            }

        }

        size_t CountLogicalBuffers(const Geo* g)
        {
            switch ( g->Type )
            {
            case GEOMETRY_TYPE_PROCEDURAL:
                if ( g->Desc.Procedural.AABBCount > 0 )
                    return 1;
                else
                    return 0;

            case GEOMETRY_TYPE_TRIANGLES:
                {
                    size_t n = 0;

                    if ( GetBytesPerIndex( g->Desc.Triangles.indexFormat ) > 0 && g->Desc.Triangles.IndexCount > 0 )
                        n++;
                    if ( g->Desc.Triangles.VertexCount > 0 )
                        n++;
                    return n;
                }
                break;

            default:
                assert(false);
                return 0;
            }
        }

        void BuildLogicalBuffers( std::vector<SerializationLogicalBuffer>& logicalBuffers, const ProceduralInstanceInfo* p )
        {
            if ( p && p->AABBCount )
            {
                SerializationLogicalBuffer lb;
                lb.Contents = (char*) p->pIsProcedural;
                lb.Size = static_cast<size_t>(p->AABBCount);
                logicalBuffers.push_back( lb );
            }
            if ( p && p->AABBCount )
            {
                SerializationLogicalBuffer lb;
                lb.Contents = (char*)p->pAABBs;
                lb.Size = static_cast<size_t>(p->AABBCount * p->AABBByteStride);
                logicalBuffers.push_back( lb );
            }
        }

        size_t CountLogicalBuffers( const ProceduralInstanceInfo* pInfo )
        {
            size_t n=0;
            if( pInfo && pInfo->AABBCount )
                n++;
            if( pInfo && pInfo->AABBCount )
                n++;
            return n;
        }

        size_t BuildSerializationLogicalBuffers( std::vector<SerializationLogicalBuffer>& logicalBuffers, const RTAS* pRTAS, size_t nRTAS )
        {
            // count logical buffers
            size_t nBuffers = 0;
            for ( size_t i = 0; i < nRTAS; i++ )
            {
                for ( size_t g = 0; g < pRTAS[i].NumGeos; g++ )
                    nBuffers += CountLogicalBuffers( pRTAS[i].pGeos + g );
                for ( size_t g = 0; g < pRTAS[i].NumInitialGeos; g++ )
                    nBuffers += CountLogicalBuffers( pRTAS[i].pInitialGeos + g );

                nBuffers += CountLogicalBuffers(pRTAS[i].pProceduralsInfo);
                nBuffers += CountLogicalBuffers(pRTAS[i].pInitialProceduralsInfo);
            }

            logicalBuffers.reserve( nBuffers );

            // build logical buffers
            for ( size_t i = 0; i < nRTAS; i++ )
            {
                for ( size_t g = 0; g < pRTAS[i].NumGeos; g++ )
                    BuildLogicalBuffers( logicalBuffers, pRTAS[i].pGeos + g );
                for ( size_t g = 0; g < pRTAS[i].NumInitialGeos; g++ )
                    BuildLogicalBuffers( logicalBuffers, pRTAS[i].pInitialGeos + g );
            }

            // sort logical buffers by contents VA and size
            std::sort( logicalBuffers.begin(), logicalBuffers.end(),
                []( const SerializationLogicalBuffer& a, const SerializationLogicalBuffer& b )
                {
                    if ( a.Contents < b.Contents )
                        return true;
                    if ( a.Contents > b.Contents )
                        return false;

                    return a.Size < b.Size;
                } );

            return nBuffers;
        }

        void BuildSerializationBuffers( std::vector<SerializationPhysicalBuffer>& physicalBuffers, std::vector<SerializationLogicalBuffer>& logicalBuffers, const RTAS* pRTAS, size_t nRTAS )
        {
            size_t nBuffers = BuildSerializationLogicalBuffers( logicalBuffers, pRTAS, nRTAS );
            physicalBuffers.reserve( nBuffers );


            // identify overlapping logical buffers and map to same physical buffer
            for ( size_t i = 0; i < nBuffers; )
            {
                char* physical_start = logicalBuffers[i].Contents;
                char* physical_end   = logicalBuffers[i].Contents + logicalBuffers[i].Size;

                // map this logical buffer to the physical buffer
                logicalBuffers[i].PhysicalIndex  = physicalBuffers.size();
                logicalBuffers[i].PhysicalOffset = 0;
                i++;

                // step forwards and identify any overlapping buffers which should fuse with this physical buffer
                while( i < nBuffers )
                {

                    char* logical_start  = logicalBuffers[i].Contents;
                    char* logical_end    = logicalBuffers[i].Contents + logicalBuffers[i].Size;

                    // this buffer doesn't overlap the current one, stop here
                    if ( logical_start >= physical_end )
                        break;

                    //If the end point overruns the end of the physical buffer, enlarge the physical buffer
                    if ( logical_end > physical_end )
                        physical_end = logical_end;

                    // map logical buffer onto physical buffer
                    logicalBuffers[i].PhysicalIndex  = physicalBuffers.size();
                    logicalBuffers[i].PhysicalOffset = logical_start - physical_start;
                    i++;
                }

                // store physical buffer structure
                SerializationPhysicalBuffer pb;
                pb.Contents = physical_start;
                pb.Size     = (physical_end - physical_start);
                physicalBuffers.push_back( pb );
            }
        }
    }

    void GRL_CALL SerializeDataSet( IStreamWriter* pWriter, const RTAS* pRTAS, size_t nRTAS )
    {
        SerializeDataSet( pWriter, pRTAS, nRTAS, nullptr, 0 );
    }

    void GRL_CALL SerializeDataSet( IStreamWriter* pWriter, const RTAS* pRTAS, size_t nRTAS, const SerializedRTAS* pSerialized, size_t nSerialized )
    {
        _INTERNAL::OStream stream( pWriter );
        stream.WriteU32( _INTERNAL::GRL::MAGIC );
        stream.WriteU32( _INTERNAL::GRL::VERSION );
        stream.WriteU64( _INTERNAL::GRL::BUFFER_ALIGN );

        stream.WriteU64( nRTAS );

        std::vector<_INTERNAL::SerializationPhysicalBuffer> physical;
        std::vector<_INTERNAL::SerializationLogicalBuffer> logical;
        _INTERNAL::BuildSerializationBuffers( physical, logical, pRTAS, nRTAS );

        stream.WriteU64( physical.size() );
        for ( size_t i = 0; i < physical.size(); i++ )
            stream.WriteU64( physical[i].Size );
        for( size_t i = 0; i<physical.size(); i++ )
            stream.WriteRawBuffer( physical[i].Contents, physical[i].Size );

        for( size_t i=0; i<nRTAS; i++ )
        {
            uint32_t nInstances = pRTAS[i].NumInstances;
            uint32_t nGeos = pRTAS[i].NumGeos;

            stream.WriteU32( nInstances );
            stream.WriteU32( nGeos );
            stream.WriteU32( pRTAS[i].Flags );
            stream.WriteU32( pRTAS[i].WasCompacted );

            uint32_t nInitialGeos      = pRTAS[i].pInitialGeos ? pRTAS[i].NumInitialGeos : 0;
            uint32_t nInitialInstances = pRTAS[i].pInitialInstances ? pRTAS[i].NumInitialInstances : 0;
            stream.WriteU32( nInitialGeos );
            stream.WriteU32( nInitialInstances );

            _INTERNAL::WriteInstances( stream, pRTAS[i].pInstances, nInstances, pRTAS, nRTAS );

            _INTERNAL::WriteGeos( stream, pRTAS[i].pGeos, nGeos, logical );

            if( nInitialGeos )
                _INTERNAL::WriteGeos( stream, pRTAS[i].pInitialGeos, nInitialGeos, logical );

            if( nInitialInstances )
                _INTERNAL::WriteInstances( stream, pRTAS[i].pInitialInstances, nInitialInstances, pRTAS, nRTAS );

            uint32_t have_procedurals         = pRTAS[i].pProceduralsInfo != nullptr        ? 1 : 0;
            uint32_t have_initial_procedurals = pRTAS[i].pInitialProceduralsInfo != nullptr ? 1 : 0;

            stream.WriteU32( have_procedurals );
            stream.WriteU32( have_initial_procedurals) ;

            if ( have_procedurals )
                _INTERNAL::WriteProceduralsInfo(stream, pRTAS[i].pProceduralsInfo, logical);

            if ( have_initial_procedurals )
                _INTERNAL::WriteProceduralsInfo(stream, pRTAS[i].pInitialProceduralsInfo, logical);

            bool isUpdate = false;
            uint64_t srcID = 0;
            if (pRTAS[i].srcID != 0)
            {
                isUpdate = true;
                srcID = pRTAS[i].srcID;
            }

            stream.WriteU64(isUpdate);
            if (isUpdate)
            {
                stream.WriteU64(srcID);
            }
        }


        stream.WriteU64( nSerialized );
        for( size_t i=0; i<nSerialized; i++ )
        {
            stream.WriteU64( pSerialized[i].BatchId          );
            stream.WriteU64( pSerialized[i].OriginalHandle   );
            stream.WriteU64( pSerialized[i].Flags            );
            stream.WriteU64( pSerialized[i].BlobSize         );
            stream.WriteU64( pSerialized[i].DeserializedSize );
            stream.WriteU64( pSerialized[i].NumBLASHandles   );
            stream.WriteU64( pSerialized[i].BlasHandleOffset );
            stream.WriteU64( pSerialized[i].RTASDataOffset   );
            stream.WriteU64( pSerialized[i].RTASDataSize );
            stream.WriteU64( pSerialized[i].DriverIdentifierOffset );
            stream.WriteU64( pSerialized[i].DriverIdentifierSize );

            stream.WriteRawBuffer( pSerialized[i].pRawBlob, (size_t) pSerialized[i].BlobSize );

        }
    }

    bool GRL_CALL DeserializeDataSet( RTASDataSet* pResult, IStreamReader* pReader )
    {
        pResult->pRTAS = nullptr;
        pResult->nRTAS = 0;
        pResult->nSerializedRTAS = 0;
        pResult->pSerializedRTAS = nullptr;
        pResult->pOpaqueAllocation = nullptr;

        _INTERNAL::IStream stream(pReader);
        _INTERNAL::AllocContainer allocs;

        if( !stream.PrefetchBytes(8) )
            return false;

        uint32_t magic32 = stream.ReadU32();
        if (magic32 == _INTERNAL::GRL::MAGIC)
        {
            stream.PopU32();
            uint32_t version = stream.ReadU32();
            stream.PopU32();
            return _INTERNAL::DeserializeGrlDataSet(pResult, stream, allocs, version);
        }

        return false;
    }

    bool GRL_CALL SerializeDataSetToFile(const RTAS* pRTAS, size_t nRTAS, const SerializedRTAS* pSerialized, size_t nSerialized, const char* path)
    {
        class FileStream : public IStreamWriter
        {
        public:
            FileStream( FILE* fp ) : m_fp(fp)
            {
            }
            ~FileStream() { fclose(m_fp);  }

            virtual void GRL_CALL Write( const void* p, size_t n ) override
            {
                m_ok = m_ok && (fwrite(p, 1, n, m_fp) == n);
                assert(m_ok);
            }
            bool HadError() const {
                return !m_ok;
            }

        private:
            FILE* m_fp;
            bool m_ok = true;
        };

        FILE* fp = fopen(path, "wb");
        if (fp == nullptr)
            return false;

        FileStream stream(fp);
        SerializeDataSet(&stream, pRTAS, nRTAS, pSerialized, nSerialized);
        return !stream.HadError();
    }

    bool GRL_CALL DeserializeDataSetFromFile(RTASDataSet* pData, const char* path)
    {
        class FileStream : public IStreamReader
        {
        public:
            FileStream( FILE* fp ) : m_fp(fp)
            {
            }
            ~FileStream() { fclose(m_fp);  }

            virtual size_t GRL_CALL Read( void* p, size_t n ) override
            {
                return fread(p, 1, n, m_fp);
            }

        private:
            FILE* m_fp;
        };

        FILE* fp = fopen(path, "rb");
        if (fp == nullptr)
            return false;

        FileStream stream(fp);
        return DeserializeDataSet(pData, &stream);
    }

    bool GRL_CALL IsRTASFile( const char* path )
    {
        FILE* fp = fopen( path, "rb" );
        if ( fp == nullptr )
            return false;

        uint32_t magic = 0;
        size_t n = fread( &magic, 4, 1, fp );
        fclose( fp );

        return (n == 1 && magic == _INTERNAL::GRL::MAGIC);
    }

    //
    //
    // Below functions are externaly used
    //
    //

    void GRL_CALL FreeDataSet( RTASDataSet* pData )
    {
        if( pData->pOpaqueAllocation )
        {
            _INTERNAL::AllocContainer::FreeExportedAllocs( pData->pOpaqueAllocation );
            pData->pRTAS = nullptr;
            pData->nRTAS = 0;
			pData->nSerializedRTAS = 0;
			pData->pSerializedRTAS = nullptr;
            pData->pOpaqueAllocation = nullptr;
        }
    }

    void GRL_CALL CloneDataSet(RTASDataSet* pDest, const RTASDataSet* pSrc)
    {
        // Reusing the serialization plumbing because it's quick, easy, and self-maintaining
        //  This does redundant mallocs and copies but the tradeoff is probably worthwhile for our purposes
        class MemStream : public IStreamWriter, public IStreamReader
        {
        public:
            virtual size_t GRL_CALL Read(void* pBytes, size_t nBytes)
            {
                if (nBytes > m_Bytes.size() - m_nBytesRead)
                    nBytes = m_Bytes.size() - m_nBytesRead;

                memcpy(pBytes, m_Bytes.data() + m_nBytesRead, nBytes);
                m_nBytesRead += nBytes;
                return nBytes;
            }

            virtual void GRL_CALL Write(const void* bytes, size_t nBytes) override
            {
                uint8_t* pBytes = (uint8_t*)bytes;
                m_Bytes.insert(m_Bytes.end(), pBytes, pBytes + nBytes);
            }

        private:
            std::vector<uint8_t> m_Bytes;
            size_t m_nBytesRead = 0;
        };

        MemStream stream;
        SerializeDataSet(&stream, pSrc->pRTAS, pSrc->nRTAS, pSrc->pSerializedRTAS, pSrc->nSerializedRTAS);
        DeserializeDataSet(pDest, &stream);

    }

    void GRL_CALL DeleteGeometryByType( RTAS* pRTAS, size_t nRTAS, GeometryType eType )
    {
        for (size_t i = 0; i < nRTAS; i++)
        {
            RTAS& rtas = pRTAS[i];

            uint32_t nKeep = 0;
            for( size_t g=0; g<rtas.NumGeos; g++ )
            {
                if (rtas.pGeos[g].Type != eType)
                    rtas.pGeos[nKeep++] = rtas.pGeos[g];
            }
            rtas.NumGeos = nKeep;
            if (rtas.NumGeos == 0)
                rtas.pGeos = nullptr;

            nKeep = 0;
            for( size_t g=0; g<rtas.NumInitialGeos; g++ )
            {
                if (rtas.pInitialGeos[g].Type != eType)
                    rtas.pInitialGeos[nKeep++] = rtas.pInitialGeos[g];
            }
            rtas.NumInitialGeos = nKeep;
            if (rtas.NumInitialGeos == 0)
                rtas.pInitialGeos = nullptr;
        }
    }


    void GRL_CALL ConvertIndicesToU32(uint32_t* pDst, IndexFormat eSrcFormat, const void* pSrcData, size_t nIndices )
    {
        switch( eSrcFormat )
        {
        case INDEX_FORMAT_NONE:
            {
                // create a trivial IB
                for (size_t i = 0; i < nIndices; i++)
                    pDst[i] = (uint32_t)i;
            }
            break;
        case INDEX_FORMAT_R16_UINT:
            {
                const uint16_t* pSrc = reinterpret_cast<const uint16_t*>(pSrcData);
                for (size_t i = 0; i < nIndices; i++)
                    pDst[i] = pSrc[i];
            }
            break;
        case INDEX_FORMAT_R32_UINT:
            {
                memcpy(pDst, pSrcData, nIndices * 4);
            }
            break;
        }
    }


    //---------------------------------------------------
    // Interpret an unsigned short bit pattern as a half,
    // and convert that half to the corresponding float's
    // bit pattern.
    //---------------------------------------------------
    unsigned int halfToFloatBits (unsigned short y)
    {
        int s = (y >> 15) & 0x00000001;
        int e = (y >> 10) & 0x0000001f;
        int m =  y        & 0x000003ff;
        if (e == 0)
        {
	        if (m == 0)
	        {
	            //
	            // Plus or minus zero
	            //
	            return s << 31;
	        }
	        else
	        {
	            //
	            // Denormalized number -- renormalize it
	            //
	            while (!(m & 0x00000400))
	            {
		            m <<= 1;
		            e -=  1;
	            }
	            e += 1;
	            m &= ~0x00000400;
	        }
        }
        else if (e == 31)
        {
	        if (m == 0)
	        {
	            //
	            // Positive or negative infinity
	            //
	            return (s << 31) | 0x7f800000;
	        }
	        else
	        {
	            //
	            // Nan -- preserve sign and significand bits
	            //
	            return (s << 31) | 0x7f800000 | (m << 13);
	        }
        }
        //
        // Normalized number
        //
        e = e + (127 - 15);
        m = m << 13;
        //
        // Assemble s, e and m.
        //
        uint32_t float_bits = (s << 31) | (e << 23) | m;
        return float_bits;
    }

    float halfToFloat( unsigned short s )
    {
        uint32_t float_bits = halfToFloatBits(s);
        float f;
        memcpy(&f, &float_bits, sizeof(f));
        return f;
    }

    inline float snormToFloat(short v)
    {
        return std::fmin(1.0f, std::fmax(-1.0f, ((float)v) * (1.0f / 32767.0f)));
    }

    inline float snorm8ToFloat(signed char v)
    {
        return std::fmin(1.0f, std::fmax(-1.0f, ((float)v) * (1.0f / 127.0f)));
    }

    inline float unormToFloat(unsigned short v)
    {
        return std::fmin(1.0f, std::fmax(0.0f, ((float)v) * (1.0f / 65535.0f)));
    }

    inline float unorm10ToFloat(unsigned v)
    {
        const unsigned short mask = (unsigned short)((1u << 10u) - 1u);
        const unsigned short v10 = (unsigned short)v & mask;
        return std::fmin(1.0f, std::fmax(0.0f, ((float)v10) * (1.0f / 1023.0f)));
    }

    inline float unorm8ToFloat(unsigned char v)
    {
        return std::fmin(1.0f, std::fmax(0.0f, ((float)v) * (1.0f / 255.0f)));
    }


    void GRL_CALL ConvertVerticesToF32a(float* pDst, VertexFormat eSrcFormat, const void* pSrcData, size_t nVerts, size_t nSrcStride)
    {

        switch (eSrcFormat)
        {
        case VERTEX_FORMAT_R32G32_FLOAT:
        {
            assert(nSrcStride % 4 == 0); // dxr requirement
            const float* pSrc = reinterpret_cast<const float*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                pDst[0] = pSrc[0];
                pDst[1] = pSrc[1];
                pDst[2] = 0.0f;
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += (nSrcStride / 4);
            }
        }
        break;

        case VERTEX_FORMAT_R32G32B32_FLOAT:
        {
            assert(nSrcStride % 4 == 0); // dxr requirement
            const float* pSrc = reinterpret_cast<const float*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                pDst[0] = pSrc[0];
                pDst[1] = pSrc[1];
                pDst[2] = pSrc[2];
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += (nSrcStride / 4);
            }
        }
        break;

        case VERTEX_FORMAT_R16G16_FLOAT:
        {
            assert(nSrcStride % 2 == 0); // dxr requirement
            const uint16_t* pSrc = reinterpret_cast<const uint16_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                for (size_t j = 0; j < 2; j++)
                    pDst[j] = halfToFloat(pSrc[j]);
                pDst[2] = 0.0f;
                pDst[3] = 0.0f;

                pDst += 4;
                pSrc += (nSrcStride / 2);
            }

#if 0 // for those with recent CPUs
            for (size_t i = 0; i < nVerts; i++)
            {
                __m128i rg16 = _mm_cvtsi32_si128(*reinterpret_cast<const uint32_t*>(pSrc));
                __m128 rg32 = _mm_cvtph_ps(rg16);
                _mm_storeu_ps(pDst, rg32);
                pDst += 4;
                pSrc += (nSrcStride / 2);
            }
#endif
        }
        break;

        case VERTEX_FORMAT_R16G16B16A16_FLOAT:
        {
            assert(nSrcStride % 2 == 0); // dxr requirement
            const uint16_t* pSrc = reinterpret_cast<const uint16_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                for (size_t j = 0; j < 4; j++)
                    pDst[j] = halfToFloat(pSrc[j]);
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += (nSrcStride / 2);
            }
#if 0
            for (size_t i = 0; i < nVerts; i++)
            {
                __m128i rgba16 = _mm_loadu_si128(reinterpret_cast<const __m128i*>(pSrc));
                __m128 rgba32 = _mm_cvtph_ps(rgba16);
                _mm_storeu_ps(pDst, rgba32);
                pDst += 4;
                pSrc += (nSrcStride / 2);
            }
#endif
        }
        break;

        case VERTEX_FORMAT_R16G16_SNORM:
        {
            assert(nSrcStride % 2 == 0); // dxr requirement
            const int16_t* pSrc = reinterpret_cast<const int16_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                for (size_t j = 0; j < 2; j++)
                    pDst[j] = snormToFloat(pSrc[j]);
                pDst[2] = 0.0f;
                pDst[3] = 0.0f;

                pDst += 4;
                pSrc += (nSrcStride / 2);
            }
        }
        break;

        case VERTEX_FORMAT_R16G16B16A16_SNORM:
        {
            assert(nSrcStride % 2 == 0); // dxr requirement
            const int16_t* pSrc = reinterpret_cast<const int16_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                for (size_t j = 0; j < 3; j++)
                    pDst[j] = snormToFloat(pSrc[j]);
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += (nSrcStride / 2);
            }
        }
        break;
        case VERTEX_FORMAT_R16G16B16A16_UNORM:
        {
            assert(nSrcStride % 2 == 0); // dxr requirement
            const uint16_t* pSrc = reinterpret_cast<const uint16_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                for (size_t j = 0; j < 3; j++)
                    pDst[j] = unormToFloat(pSrc[j]);
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += (nSrcStride / 2);
            }
        }
        break;

        case VERTEX_FORMAT_R16G16_UNORM:
        {
            assert(nSrcStride % 2 == 0); // dxr requirement
            const uint16_t* pSrc = reinterpret_cast<const uint16_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                for (size_t j = 0; j < 2; j++)
                    pDst[j] = unormToFloat(pSrc[j]);
                pDst[2] = 0.0f;
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += (nSrcStride / 2);
            }
        }
        break;

        case VERTEX_FORMAT_R10G10B10A2_UNORM:
        {
            assert(nSrcStride % 4 == 0); // dxr requirement
            const uint32_t* pSrc = reinterpret_cast<const uint32_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                uint32_t data = *pSrc;
                for (size_t j = 0; j < 3; j++)
                    pDst[j] = unorm10ToFloat(data >> (j * 10));
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += (nSrcStride / 4);
            }
        }
        break;

        case VERTEX_FORMAT_R8G8B8A8_UNORM:
        {
            const uint8_t* pSrc = reinterpret_cast<const uint8_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                for (size_t j = 0; j < 3; j++)
                    pDst[j] = unorm8ToFloat(pSrc[j]);
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += nSrcStride;
            }
        }
        break;

        case VERTEX_FORMAT_R8G8_UNORM:
        {
            const uint8_t* pSrc = reinterpret_cast<const uint8_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                for (size_t j = 0; j < 2; j++)
                    pDst[j] = unorm8ToFloat(pSrc[j]);
                pDst[2] = 0.0f;
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += nSrcStride;
            }
        }
        break;

        case VERTEX_FORMAT_R8G8B8A8_SNORM:
        {
            const int8_t* pSrc = reinterpret_cast<const int8_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                for (size_t j = 0; j < 3; j++)
                    pDst[j] = snorm8ToFloat(pSrc[j]);
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += nSrcStride;
            }
        }
        break;

        case VERTEX_FORMAT_R8G8_SNORM:
        {
            const int8_t* pSrc = reinterpret_cast<const int8_t*>(pSrcData);
            for (size_t i = 0; i < nVerts; i++)
            {
                for (size_t j = 0; j < 2; j++)
                    pDst[j] = snorm8ToFloat(pSrc[j]);
                pDst[2] = 0.0f;
                pDst[3] = 0.0f;
                pDst += 4;
                pSrc += nSrcStride;
            }
        }
        break;

        default: assert(0); break;
        }
    }



    size_t GRL_CALL CountGeoPrimitives( const Geo& geo )
    {
        switch( geo.Type )
        {
        case GEOMETRY_TYPE_PROCEDURAL:
            return geo.Desc.Procedural.AABBCount;

        case GEOMETRY_TYPE_TRIANGLES:
            {
                if (geo.Desc.Triangles.indexFormat == INDEX_FORMAT_NONE)
                    return geo.Desc.Triangles.VertexCount / 3;
                else
                    return geo.Desc.Triangles.IndexCount / 3;
            }

        default:
            return 0;
        }
    }


    void GRL_CALL CompactAABBs( AABB* pDst, const void* pSrc, size_t nAABBs, size_t nSrcStride )
    {
        for( size_t i=0; i<nAABBs; i++ )
            memcpy(&pDst[i], ((uint8_t*)pSrc) + i * nSrcStride, sizeof(AABB));
    }

    void MatrixMulPosition( float xyz[3], const Matrix& m )
    {
        float x = xyz[0];
        float y = xyz[1];
        float z = xyz[2];
        for (size_t i = 0; i < 3; i++)
            xyz[i] = m.Transform[i][0]*x + m.Transform[i][1]*y + m.Transform[i][2]*z + m.Transform[i][3];

    }

    void MatrixMulVector( float xyz[3], const Matrix& m )
    {
        float x = xyz[0];
        float y = xyz[1];
        float z = xyz[2];
        for (size_t i = 0; i < 3; i++)
            xyz[i] = m.Transform[i][0] * x + m.Transform[i][1] * y + m.Transform[i][2] * z;
    }


    void TransformAABBs( const GeometryProcedural& geo, const Matrix& m )
    {
        // Use abs-matrix to transform the AABB extent
        //   See: https://zeuxcg.org/2010/10/17/aabb-from-obb-with-component-wise-abs/
        Matrix abs_m;
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 4; j++)
                abs_m.Transform[i][j] = fabsf( m.Transform[i][j] );

        uint8_t* pBytes = (uint8_t*)geo.pAABBs;
        for( size_t i=0; i< geo.AABBCount; i++ )
        {
            AABB* pBox = (AABB*)pBytes;

            float Center[3];
            float Extent[3];
            for( size_t j=0; j<3; j++ )
            {
                Center[j] = (pBox->Max[j] + pBox->Min[j])*0.5f;
                Extent[j] = (pBox->Max[j] - pBox->Min[j])*0.5f;
            }

            MatrixMulPosition(Center, m);
            MatrixMulVector(Extent, abs_m);

            for( size_t j=0; j<3; j++ )
            {
                pBox->Min[j] = Center[j] - Extent[j];
                pBox->Max[j] = Center[j] + Extent[j];
            }

            pBytes += geo.AABBByteStride;
        }

    }

    Matrix ConcatenateTransforms(const Matrix& a, const Matrix& b)
    {
        //   Transform is 3x3 plus translation. So:  f(x) = Mx + t
        //    To concatenate these right to left, we do:
        //      Ma ( Mbx + Tb) + Ta ==  MaMbx + (MaTb + Ta)

        // multiply upper 3x3
        Matrix m;
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 3; j++)
                m.Transform[i][j] = a.Transform[i][0] * b.Transform[0][j] +
                                    a.Transform[i][1] * b.Transform[1][j] +
                                    a.Transform[i][2] * b.Transform[2][j];
        // add translation
        float bx = b.Transform[0][3];
        float by = b.Transform[1][3];
        float bz = b.Transform[2][3];
        for (size_t i = 0; i < 3; i++)
            m.Transform[i][3] = bx * a.Transform[0][i] +
                                by * a.Transform[1][i] +
                                bz * a.Transform[2][i] + a.Transform[i][3];
        return m;
    }

    Matrix GRL_CALL IdentityMatrix(  )
    {
        Matrix m;
        for (size_t i = 0; i < 3; i++)
            for (size_t j = 0; j < 4; j++)
                m.Transform[i][j] = (i == j) ? 1.0f : 0.0f;
        return m;
    }


    void GRL_CALL FlattenDXRScene(RTASDataSet* pResult, RTAS* pDXRTLAS )
    {
        _INTERNAL::AllocContainer allocs;

        auto Flags = (uint8_t)BUILD_FLAG_PREFER_FAST_TRACE | (uint8_t)BUILD_FLAG_ALLOW_COMPACTION;

        pResult->nRTAS             = 0;
        pResult->pOpaqueAllocation = nullptr;
        pResult->pRTAS             = nullptr;

        pResult->nRTAS = 2;
        pResult->pRTAS = allocs.New<RTAS>(2);
        assert(pResult->pRTAS != nullptr);

        RTAS* pTlas = pResult->pRTAS + 0;
        RTAS* pBlas = pResult->pRTAS + 1;

        pTlas->Flags = (uint8_t) Flags;
        pTlas->NumGeos = 0;
        pTlas->NumInitialGeos = 0;
        pTlas->NumInitialInstances = 0;
        pTlas->NumInstances = 1;
        pTlas->pGeos = nullptr;
        pTlas->pInitialGeos = nullptr;
        pTlas->pInitialInstances = nullptr;
        pTlas->pProceduralsInfo = nullptr;
        pTlas->pInitialProceduralsInfo = nullptr;
        pTlas->pInstances = allocs.New<Instance>(1);
        assert(pTlas->pInstances != nullptr);
        pTlas->WasCompacted = 0;
        pTlas->srcID = 0;

        pTlas->pInstances->Flags = 0;
        pTlas->pInstances->InstanceMask = 0xff;
        pTlas->pInstances->InstanceContributionToHitGroupIndex = 0;
        pTlas->pInstances->InstanceID = 0;
        pTlas->pInstances->pRTAS = pBlas;
        pTlas->pInstances->Transform = IdentityMatrix();

        // count geos.  Skip any fully masked stuff
        size_t nGeos = 0;
        for (size_t i = 0; i < pDXRTLAS->NumInstances; i++)
        {
            if (pDXRTLAS->pInstances[i].InstanceMask != 0)
                nGeos += pDXRTLAS->pInstances[i].pRTAS->NumGeos;
        }

        Geo* pGeos = allocs.New<Geo>(nGeos);
        assert(pGeos != nullptr);
        pBlas->NumGeos = (uint32_t)nGeos;
        pBlas->NumInitialGeos = 0;
        pBlas->NumInitialInstances = 0;
        pBlas->NumInstances = 0;
        pBlas->Flags = (uint8_t)Flags;
        pBlas->WasCompacted = 0;
        pBlas->pInitialGeos = nullptr;
        pBlas->pInitialInstances = nullptr;
        pBlas->pProceduralsInfo = nullptr;
        pBlas->pInitialProceduralsInfo = nullptr;
        pBlas->pInstances = nullptr;
        pBlas->pGeos = pGeos;
        pBlas->srcID = 0;

        nGeos = 0;
        for( size_t i=0; i<pDXRTLAS->NumInstances; i++ )
        {
            if ( pDXRTLAS->pInstances[i].InstanceMask == 0 )
                continue;

            RTAS* pInstancedBlas = pDXRTLAS->pInstances[i].pRTAS;
            Matrix* pInstanceTransform = &pDXRTLAS->pInstances[i].Transform;
            bool bOpaqueInstance    = pDXRTLAS->pInstances[i].Flags & (uint8_t)INSTANCE_FLAG_FORCE_OPAQUE;
            bool bNonOpaqueInstance = pDXRTLAS->pInstances[i].Flags & (uint8_t)INSTANCE_FLAG_FORCE_NON_OPAQUE;

            for( size_t g=0; g<pInstancedBlas->NumGeos; g++ )
            {
                Geo* pFlatGeo = &pGeos[nGeos++];
                *pFlatGeo = pInstancedBlas->pGeos[g];

                // adjust opaque flags based on instance flags
                if (bOpaqueInstance)
                    pFlatGeo->Flags |= GEOMETRY_FLAG_OPAQUE;
                if( bNonOpaqueInstance )
                    pFlatGeo->Flags &= ~GEOMETRY_FLAG_OPAQUE;

                if( pFlatGeo->Type == GEOMETRY_TYPE_TRIANGLES )
                {
                    Matrix xform = *pInstanceTransform;

                    // concatenate instance transform onto any existing geo transform
                    if( pFlatGeo->Desc.Triangles.pTransform != nullptr )
                        xform = ConcatenateTransforms( xform, *pFlatGeo->Desc.Triangles.pTransform );

                    pFlatGeo->Desc.Triangles.pTransform = nullptr;

                    // Convert verts to float for transformation
                    size_t nVerts = pFlatGeo->Desc.Triangles.VertexCount;
                    float* pVB = allocs.New<float>( 4 * nVerts );
                    assert(pVB != nullptr);
                    ConvertVerticesToF32a( pVB, pFlatGeo->Desc.Triangles.vertexFormat,
                                                pFlatGeo->Desc.Triangles.pVertexBuffer,
                                                pFlatGeo->Desc.Triangles.VertexCount,
                                                (size_t)pFlatGeo->Desc.Triangles.VertexBufferByteStride );

                    // transform positions
                    for (size_t j = 0; j < nVerts; j++)
                        MatrixMulPosition( pVB + 4 * j, xform );

                    pFlatGeo->Desc.Triangles.pVertexBuffer = pVB;
                    pFlatGeo->Desc.Triangles.vertexFormat = VERTEX_FORMAT_R32G32B32_FLOAT;
                    pFlatGeo->Desc.Triangles.VertexBufferByteStride = 16;

                    // clone index buffer that output is fully self-contained
                    size_t nIBSize = GetBytesPerIndex(pFlatGeo->Desc.Triangles.indexFormat) * pFlatGeo->Desc.Triangles.IndexCount;
                    if( nIBSize )
                    {
                        void* pCopy = allocs.Malloc(nIBSize);
                        memcpy(pCopy, pFlatGeo->Desc.Triangles.pIndexBuffer, nIBSize);
                        pFlatGeo->Desc.Triangles.pIndexBuffer = pCopy;
                    }

                }
                else
                {
                    // transform AABBs
                    size_t nAABBSize = (size_t)(pFlatGeo->Desc.Procedural.AABBByteStride * pFlatGeo->Desc.Procedural.AABBCount);
                    if( nAABBSize )
                    {
                        void* pAABBs = allocs.Malloc(nAABBSize);
                        memcpy(pAABBs, pFlatGeo->Desc.Procedural.pAABBs, nAABBSize);
                        pFlatGeo->Desc.Procedural.pAABBs = pAABBs;

                        TransformAABBs( pFlatGeo->Desc.Procedural, *pInstanceTransform );
                    }

                }

            }
        }

        pResult->pOpaqueAllocation = allocs.ExportAllocs();
    }

    void GRL_CALL CreateProceduralDataSetFromTriangleDataSet(RTASDataSet* pResult, RTASDataSet* pSource)
    {
#if __cplusplus >= 201402L
        _INTERNAL::AllocContainer allocs;

        pResult->nRTAS = pSource->nRTAS;
        pResult->pOpaqueAllocation = nullptr;
        pResult->pRTAS = allocs.New<RTAS>(pSource->nRTAS);
        assert(pResult->pRTAS != nullptr);
        size_t idxgeo = 0;
        for (size_t i = 0; i < pSource->nRTAS; ++i)
        {
            auto& srcRtas = pSource->pRTAS[i];
            auto& dstRtas = pResult->pRTAS[i];

            dstRtas = srcRtas;

            if (srcRtas.NumInstances > 0)
            {
                dstRtas.pInstances = allocs.New<Instance>(srcRtas.NumInstances);
                assert(dstRtas.pInstances != nullptr);
                for (size_t j = 0; j < srcRtas.NumInstances; ++j)
                {
                    dstRtas.pInstances[j] = srcRtas.pInstances[j];
                    size_t idx = std::distance(pSource->pRTAS, srcRtas.pInstances[j].pRTAS);
                    dstRtas.pInstances[j].pRTAS = &pResult->pRTAS[idx];
                }
            }
            if (srcRtas.NumGeos > 0)
            {
                dstRtas.pGeos = allocs.New<Geo>(srcRtas.NumGeos);
                assert(dstRtas.pGeos != nullptr);
                for (size_t j = 0; j < srcRtas.NumGeos; ++j)
                {
                    auto& srcGeo = srcRtas.pGeos[j];
                    auto& dstGeo = dstRtas.pGeos[j];
                    dstGeo = srcGeo;
                    if (srcGeo.Type == GEOMETRY_TYPE_TRIANGLES)
                    {
                        size_t nVerts = srcGeo.Desc.Triangles.VertexCount;
                        auto pVertices = std::make_unique<float[]>(4 * nVerts);
                        ConvertVerticesToF32a(pVertices.get(), srcGeo.Desc.Triangles.vertexFormat,
                            srcGeo.Desc.Triangles.pVertexBuffer,
                            nVerts,
                            static_cast<size_t>(srcGeo.Desc.Triangles.VertexBufferByteStride));

                        if (srcGeo.Desc.Triangles.pTransform)
                        {
                            for (size_t k = 0; k < nVerts; ++k)
                                MatrixMulPosition(pVertices.get() + 4 * k, *srcGeo.Desc.Triangles.pTransform);
                        }

                        size_t numPrims = CountGeoPrimitives(srcGeo);
                        dstGeo.Type = GEOMETRY_TYPE_PROCEDURAL;
                        dstGeo.Desc.Procedural.AABBCount = static_cast<uint32_t>(numPrims);
                        dstGeo.Desc.Procedural.AABBByteStride = sizeof(float) * 6;
                        size_t procSize = static_cast<size_t>(dstGeo.Desc.Procedural.AABBCount * dstGeo.Desc.Procedural.AABBByteStride);
                        dstGeo.Desc.Procedural.pAABBs = allocs.Malloc(procSize);

                        auto pIndices = std::make_unique<uint32_t[]>(numPrims * 3);
                        ConvertIndicesToU32(pIndices.get(), srcGeo.Desc.Triangles.indexFormat, srcGeo.Desc.Triangles.pIndexBuffer, numPrims * 3);
                        float* pAABBs = static_cast<float*>(dstGeo.Desc.Procedural.pAABBs);
                        for (size_t k = 0; k < 3 * numPrims; k += 3)
                        {
                            uint32_t indices[3] = { pIndices[k], pIndices[k + 1], pIndices[k + 2] };
                            if (indices[0] >= nVerts || indices[1] >= nVerts || indices[2] >= nVerts)
                            {
                                for (size_t l = 0; l < 6; ++l)
                                {
                                    pAABBs[2 * k + l] = NAN;
                                }
                                continue;
                            }
                            float tri[3][3];
                            for (size_t axis = 0; axis < 3; ++axis)
                            {
                                tri[0][axis] = pVertices[4 * indices[0] + axis];
                                tri[1][axis] = pVertices[4 * indices[1] + axis];
                                tri[2][axis] = pVertices[4 * indices[2] + axis];
                            }
                            AABB box;
                            for (size_t l = 0; l < 3; ++l)
                            {
                                for (size_t axis = 0; axis < 3; axis++)
                                {
                                    if (l == 0)
                                    {
                                        box.Min[axis] = tri[l][axis];
                                        box.Max[axis] = tri[l][axis];
                                    }
                                    else
                                    {
                                        box.Min[axis] = std::min(tri[l][axis], box.Min[axis]);
                                        box.Max[axis] = std::max(tri[l][axis], box.Max[axis]);
                                    }
                                }
                            }
                            for (size_t axis = 0; axis < 3; ++axis)
                            {
                                pAABBs[2 * k + axis] = box.Min[axis];
                                pAABBs[2 * k + 3 + axis] = box.Max[axis];
                            }
                        }
                    }
                    else if (srcGeo.Type == GEOMETRY_TYPE_PROCEDURAL)
                    {
                        size_t procSize = static_cast<size_t>(srcGeo.Desc.Procedural.AABBCount * srcGeo.Desc.Procedural.AABBByteStride);
                        if (procSize)
                        {
                            dstGeo.Desc.Procedural.pAABBs = allocs.Malloc(procSize);
                            memcpy(dstGeo.Desc.Procedural.pAABBs, srcGeo.Desc.Procedural.pAABBs, procSize);
                        }
                    }
                    else
                        assert(0);
                    idxgeo++;
                }
            }
        }
        pResult->pOpaqueAllocation = allocs.ExportAllocs();
#endif        
    }

    void GRL_CALL TransformPosition( float* pDst, const float* pSrc, const Matrix& m )
    {
        float xyz[3];
        for( size_t i=0; i<3; i++ )
            xyz[i] = pSrc[i];

        MatrixMulPosition( xyz, m );

        for( size_t i=0; i<3; i++ )
            pDst[i] = xyz[i];
    }

#define PI 3.14159265358979323846f


    Matrix MatrixRotateX( float degrees )
    {
        float radians = degrees * (PI / 180.0f);
        float cosT = cosf( radians );
        float sinT = sinf( radians );
        return {
            1,  0,        0, 0,
            0,  cosT, -sinT, 0,
            0,  sinT,  cosT, 0
        };
    }

    Matrix MatrixRotateY( float degrees )
    {
        float radians = degrees * (PI / 180.0f);
        float cosT = cosf( radians );
        float sinT = sinf( radians );
        return {
            cosT,  0,  -sinT, 0,
            0,     1,     0,  0,
            sinT,  0,  cosT,  0
        };
    }

    Matrix MatrixRotateZ( float degrees )
    {
        float radians = degrees * (PI / 180.0f);
        float cosT = cosf( radians );
        float sinT = sinf( radians );
        return {
            cosT, -sinT, 0, 0,
            sinT,  cosT, 0, 0,
            0,       0,  1, 0
        };
    }

    Matrix MatrixTranslation( float x, float y, float z )
    {
       return {
           1, 0, 0, x,
           0, 1, 0, y,
           0, 0, 1, z
        };
    }

    Matrix MatrixScale( float s )
    {
        return {
            s, 0, 0, 0,
            0, s, 0, 0,
            0, 0, s, 0
        };
    }


    // dopey little checksum algorithm for buffer deduplification
    uint64_t CheckSum( void* buffer, size_t n )
    {
        uint8_t* bytes = (uint8_t*)buffer;
        uint64_t accu = 0;
        size_t num_words = (n & (~8ull))/8;
        uint64_t* words = (uint64_t*)bytes;
        for ( size_t i = 0; i < num_words; i++ )
            accu += words[i];

        words += num_words;
        for ( size_t i = static_cast<size_t>(8 * num_words); i < n; i++ )
            accu += bytes[i];

        return accu;
    }

    void DeduplicateBuffers( RTASDataSet* output, const RTASDataSet* input )
    {
        _INTERNAL::AllocContainer allocs;
        output->nRTAS = input->nRTAS;
        output->nSerializedRTAS = input->nSerializedRTAS;
        output->pRTAS = nullptr;
        output->pSerializedRTAS = nullptr;

        // just copy any serialized rtas for now
        // TODO:  Could attempt to deduplicate these too
        if ( input->nSerializedRTAS )
            output->pSerializedRTAS = allocs.ShallowCopy<SerializedRTAS>(input->pSerializedRTAS, input->nSerializedRTAS );
        for ( size_t s = 0; s < input->nSerializedRTAS; s++ )
            output->pSerializedRTAS[s].pRawBlob = allocs.ShallowCopy<uint8_t>( input->pSerializedRTAS[s].pRawBlob, static_cast<size_t>(input->pSerializedRTAS[s].BlobSize) );

        // Start by making copies of all the meta-data

        if ( input->nRTAS )
            output->pRTAS = allocs.ShallowCopy<RTAS>( input->pRTAS, input->nRTAS );

        for ( size_t r = 0; r < input->nRTAS; r++ )
        {
            const RTAS& in = input->pRTAS[r];
            RTAS& out = output->pRTAS[r];
            out.pGeos               = allocs.ShallowCopy<Geo>( in.pGeos, in.NumGeos );
            out.pInitialGeos        = allocs.ShallowCopy<Geo>( in.pInitialGeos, in.NumInitialGeos );
            out.pInitialInstances   = allocs.ShallowCopy<Instance>( in.pInitialInstances, in.NumInitialInstances );
            out.pInstances          = allocs.ShallowCopy<Instance>( in.pInstances, in.NumInstances );

            // copy geo transforms
            for ( size_t g = 0; g < out.NumGeos; g++ )
            {
                if ( out.pGeos[g].Type == GEOMETRY_TYPE_TRIANGLES && out.pGeos[g].Desc.Triangles.pTransform != nullptr )
                    out.pGeos[g].Desc.Triangles.pTransform = allocs.ShallowCopy<Matrix>( out.pGeos[g].Desc.Triangles.pTransform, 1 );
            }
            for ( size_t g = 0; g < out.NumInitialGeos; g++ )
            {
                if ( out.pInitialGeos[g].Type == GEOMETRY_TYPE_TRIANGLES && out.pInitialGeos[g].Desc.Triangles.pTransform != nullptr )
                    out.pInitialGeos[g].Desc.Triangles.pTransform = allocs.ShallowCopy<Matrix>( out.pInitialGeos[g].Desc.Triangles.pTransform, 1 );
            }

            // rebase blas pointers
            for ( size_t i = 0; i < in.NumInstances; i++ )
            {
                Instance& outInst = out.pInstances[i];
                const Instance& inInst = in.pInstances[i];
                outInst.pRTAS = output->pRTAS + (inInst.pRTAS - input->pRTAS);
            }
            for ( size_t i = 0; i < in.NumInitialInstances; i++ )
            {
                Instance& outInst = out.pInitialInstances[i];
                const Instance& inInst = in.pInitialInstances[i];
                outInst.pRTAS = output->pRTAS + (inInst.pRTAS - input->pRTAS);
            }
        }

        // build logical buffers as for serialization
        std::vector<_INTERNAL::SerializationLogicalBuffer> buffers;
        _INTERNAL::BuildSerializationLogicalBuffers( buffers, output->pRTAS, output->nRTAS );

        // build a set of buffers with checksums.. sort it by checksum
        std::vector<uint64_t> CheckSums( buffers.size() );;
        for ( size_t i = 0; i < buffers.size(); i++ )
            CheckSums[i] = CheckSum( buffers[i].Contents, buffers[i].Size );

        struct CheckSumBuffer : public _INTERNAL::SerializationLogicalBuffer
        {
            uint64_t CheckSum;
        };

        std::vector<CheckSumBuffer> csBuffers( buffers.size() );
        for ( size_t i = 0; i < buffers.size(); i++ )
        {
            csBuffers[i].PhysicalIndex = buffers.size();
            csBuffers[i].Contents = buffers[i].Contents;
            csBuffers[i].Size     = buffers[i].Size;
            csBuffers[i].CheckSum = CheckSums[i];
        }

        std::sort( csBuffers.begin(), csBuffers.end(), []( const CheckSumBuffer& a, const CheckSumBuffer& b ) { return a.CheckSum < b.CheckSum; } );

        std::vector<_INTERNAL::SerializationLogicalBuffer> uniqueBuffers;

        // scan the original buffer...
        for ( size_t i = 0; i < buffers.size(); i++ )
        {
            char* Contents = buffers[i].Contents;
            size_t Size = buffers[i].Size;

            // find range of buffers with matching checksums
            auto start = std::lower_bound( csBuffers.begin(), csBuffers.end(), CheckSums[i], []( const CheckSumBuffer& a, uint64_t b ) { return a.CheckSum < b; } );
            auto end = start;
            do
            {
                ++end;
            } while ( end != csBuffers.end() && end->CheckSum == start->CheckSum );


            // if there is more than one matching checksum do byte-by-byte comparison to find the proper match
            auto match = start;
            if( start != end )
            {
                do
                {
                    if ( start->Size == Size )
                    {
                        if ( start->Contents == Contents || memcmp( start->Contents, Contents, Size ) == 0 )
                        {
                            match = start;
                            break;
                        }
                    }
                    ++start;
                } while ( start != end );
            }

            if ( match->PhysicalIndex == buffers.size() )
            {
                // allocate a physical buffer for each logical buffer the first time a match is found with it
                _INTERNAL::SerializationLogicalBuffer u;
                u.Contents = allocs.ShallowCopy<char>( Contents, Size );
                u.Size = Size;
                match->PhysicalIndex = uniqueBuffers.size();
                uniqueBuffers.push_back( u );
            }

            buffers[i].PhysicalIndex = match->PhysicalIndex;
        }


        // now go over the out geos and remap their pointers

        auto REMAP_BUFFER =
        [&uniqueBuffers, &buffers]( void* Contents, size_t size ) -> char*
        {
            if ( Contents == nullptr )
                return nullptr;
            if( size == 0 )
                return nullptr;

            auto logical = _INTERNAL::FindBufferInSortedSet( buffers, Contents );
            return uniqueBuffers[logical->PhysicalIndex].Contents;
        };

        auto REMAP_GEOS =
        [REMAP_BUFFER]( Geo* geos, size_t nGeos )
        {
            for ( size_t g = 0; g < nGeos; g++ )
            {
                switch ( geos[g].Type )
                {
                case GEOMETRY_TYPE_TRIANGLES:
                    {
                        auto& tri = geos[g].Desc.Triangles;
                        size_t nIBSize = RTASFile::GetBytesPerIndex( tri.indexFormat ) * tri.IndexCount;
                        size_t nVBSize = static_cast<size_t>(tri.VertexBufferByteStride * tri.VertexCount);
                        tri.pIndexBuffer  = REMAP_BUFFER( tri.pIndexBuffer, nIBSize );
                        tri.pVertexBuffer = REMAP_BUFFER( tri.pVertexBuffer, nVBSize );
                    }
                    break;
                case GEOMETRY_TYPE_PROCEDURAL:
                    {
                        auto& proc = geos[g].Desc.Procedural;
                        size_t size = static_cast<size_t>(proc.AABBByteStride * proc.AABBCount);
                        proc.pAABBs = REMAP_BUFFER( proc.pAABBs, size );
                    }
                    break;
                }
            }
        };


        for ( size_t r = 0; r < output->nRTAS; r++ )
        {
            if ( output->pRTAS[r].NumGeos )
                REMAP_GEOS( output->pRTAS[r].pGeos, output->pRTAS[r].NumGeos );
            if ( output->pRTAS[r].NumInitialGeos )
                REMAP_GEOS( output->pRTAS[r].pInitialGeos, output->pRTAS[r].NumInitialGeos );
        }

        assert( output->pOpaqueAllocation == nullptr );
        output->pOpaqueAllocation = allocs.ExportAllocs();
    }


    bool CompareProcedurals( const RTASFile::GeometryProcedural& a, const RTASFile::GeometryProcedural& b )
    {
        if ( a.AABBCount != b.AABBCount || a.AABBByteStride != b.AABBByteStride )
            return false;
        if ( memcmp( a.pAABBs, b.pAABBs, static_cast<size_t>(a.AABBByteStride * a.AABBCount) ) != 0 )
            return false;
        return true;
    }

    bool CompareTriangles( const RTASFile::GeometryTriangles& a, const RTASFile::GeometryTriangles& b )
    {
        if ( a.pTransform && b.pTransform )
        {
            if ( memcmp( a.pTransform, b.pTransform, sizeof( *a.pTransform ) ) != 0 )
                return false;
        }
        else if ( !(a.pTransform == nullptr && b.pTransform == nullptr) )
            return false;

        if ( a.IndexCount != b.IndexCount ) return false;
        if ( a.indexFormat != b.indexFormat ) return false;
        if ( a.VertexBufferByteStride != b.VertexBufferByteStride ) return false;
        if ( a.VertexCount != b.VertexCount ) return false;
        if ( a.vertexFormat != b.vertexFormat ) return false;

        size_t nIBSize = RTASFile::GetBytesPerIndex( a.indexFormat ) * a.IndexCount;
        size_t nVBSize = static_cast<size_t>(a.VertexBufferByteStride * a.VertexCount);

        if ( nIBSize && memcmp( a.pIndexBuffer, b.pIndexBuffer, nIBSize ) != 0 )
            return false;
        if ( nVBSize && memcmp( a.pVertexBuffer, b.pVertexBuffer, nVBSize ) != 0 )
            return false;
        return true;
    }

    bool CompareGeos( const RTASFile::Geo* a, const RTASFile::Geo* b )
    {
        if ( a->Type != b->Type ) return false;
        if ( a->Flags != b->Flags ) return false;

        switch ( a->Type )
        {
        case GEOMETRY_TYPE_TRIANGLES:
            return CompareTriangles( a->Desc.Triangles, b->Desc.Triangles );
            break;

        case GEOMETRY_TYPE_PROCEDURAL:
            return CompareProcedurals( a->Desc.Procedural, b->Desc.Procedural );
            break;
        default:
            return false;
        }
    }

    bool CompareInstances( const RTASFile::Instance* a, const RTASFile::Instance* b )
    {
        if ( a->Flags != b->Flags ) return false;
        if ( a->InstanceContributionToHitGroupIndex != b->InstanceContributionToHitGroupIndex ) return false;
        if ( a->InstanceID != b->InstanceID ) return false;
        if ( a->InstanceMask != b->InstanceMask ) return false;
        if ( memcmp( &a->Transform, &b->Transform, sizeof( a->Transform ) ) != 0 ) return false;
        if ( !CompareRTAS( a->pRTAS, b->pRTAS ) ) return false;
        return true;
    }

    bool CompareRTAS( const RTASFile::RTAS* a, const RTASFile::RTAS* b )
    {
        if ( a == nullptr && b == nullptr ) return true;
        if ( a == nullptr || b == nullptr ) return false;
        if ( a->NumInitialInstances != b->NumInitialInstances ) return false;
        if ( a->NumInitialGeos != b->NumInitialGeos ) return false;
        if ( a->NumInstances != b->NumInstances ) return false;
        if ( a->WasCompacted != b->WasCompacted ) return false;
        if ( a->Flags != b->Flags ) return false;
        if ( a->NumGeos != b->NumGeos ) return false;

        if ( a->NumGeos )
        {
            for ( size_t g = 0; g < a->NumGeos; g++ )
                if ( !CompareGeos( a->pGeos + g, b->pGeos + g ) )
                    return false;
        }
        if ( a->NumInitialGeos )
        {
            for ( size_t g = 0; g < a->NumGeos; g++ )
                if ( !CompareGeos( a->pInitialGeos + g, b->pInitialGeos + g ) )
                    return false;
        }
        if ( a->NumInstances )
        {
            for ( size_t g = 0; g < a->NumInstances; g++ )
                if ( !CompareInstances( a->pInstances + g, b->pInstances + g ) )
                    return false;
        }
        if ( a->NumInitialInstances )
        {
            for ( size_t g = 0; g < a->NumInitialInstances; g++ )
                if ( !CompareInstances( a->pInitialInstances + g, b->pInitialInstances + g ) )
                    return false;
        }

        return true;
    }

    bool CompareRTASDataSet( const RTASFile::RTASDataSet* a, const RTASFile::RTASDataSet* b )
    {
        if ( a->nRTAS != b->nRTAS || a->nSerializedRTAS != b->nSerializedRTAS )
        {
            return false;
        }

        for ( size_t r = 0; r < a->nSerializedRTAS; r++ )
        {
            if ( a->pSerializedRTAS[r].OriginalHandle != b->pSerializedRTAS[r].OriginalHandle ) return false;
            if ( a->pSerializedRTAS[r].Flags != b->pSerializedRTAS[r].Flags ) return false;
            if ( a->pSerializedRTAS[r].BlobSize != b->pSerializedRTAS[r].BlobSize ) return false;
            if ( a->pSerializedRTAS[r].DeserializedSize != b->pSerializedRTAS[r].DeserializedSize ) return false;
            if ( a->pSerializedRTAS[r].NumBLASHandles != b->pSerializedRTAS[r].NumBLASHandles ) return false;
            if ( a->pSerializedRTAS[r].BlasHandleOffset != b->pSerializedRTAS[r].BlasHandleOffset ) return false;
            if ( a->pSerializedRTAS[r].RTASDataOffset != b->pSerializedRTAS[r].RTASDataOffset ) return false;
            if ( a->pSerializedRTAS[r].RTASDataSize != b->pSerializedRTAS[r].RTASDataSize ) return false;
            if ( a->pSerializedRTAS[r].DriverIdentifierOffset != b->pSerializedRTAS[r].DriverIdentifierOffset ) return false;
            if ( a->pSerializedRTAS[r].DriverIdentifierSize != b->pSerializedRTAS[r].DriverIdentifierSize ) return false;

            if ( memcmp( a->pSerializedRTAS[r].pRawBlob, b->pSerializedRTAS[r].pRawBlob, static_cast<size_t>(a->pSerializedRTAS[r].BlobSize) ) != 0 )
                return false;
        }

        for ( size_t r = 0; r < a->nRTAS; r++ )
        {
            if ( !CompareRTAS( &a->pRTAS[r], &b->pRTAS[r] ) )
                return false;
        }

        return true;
    }


    void DeIndexTriangles( RTASDataSet* output, const RTASDataSet* input )
    {
        _INTERNAL::AllocContainer allocs;
        output->nRTAS = input->nRTAS;
        output->nSerializedRTAS = 0;
        output->pSerializedRTAS = nullptr;
        output->pRTAS = nullptr;

        if ( input->nSerializedRTAS )
            output->pSerializedRTAS = allocs.ShallowCopy<SerializedRTAS>( input->pSerializedRTAS, input->nSerializedRTAS );

        for ( size_t s = 0; s < input->nSerializedRTAS; s++ )
            output->pSerializedRTAS[s].pRawBlob = allocs.ShallowCopy<uint8_t>( input->pSerializedRTAS[s].pRawBlob, static_cast<size_t>(input->pSerializedRTAS[s].BlobSize) );

        if ( input->nRTAS )
        {
            output->pRTAS = allocs.ShallowCopy<RTAS>( input->pRTAS, input->nRTAS );
            for ( size_t r = 0; r < input->nRTAS; r++ )
            {
                const RTAS& in = input->pRTAS[r];
                RTAS& out = output->pRTAS[r];

                out.pGeos             = allocs.ShallowCopy<Geo>( in.pGeos, in.NumGeos );
                out.pInitialGeos      = allocs.ShallowCopy<Geo>( in.pInitialGeos, in.NumInitialGeos );
                out.pInitialInstances = allocs.ShallowCopy<Instance>( in.pInitialInstances, in.NumInitialInstances );
                out.pInstances        = allocs.ShallowCopy<Instance>( in.pInstances, in.NumInstances );

                auto _PROCESS_GEO = [&allocs]( Geo* outGeo )
                {
                    if ( outGeo->Type == GEOMETRY_TYPE_TRIANGLES )
                    {
                        // copy geo transforms
                        if ( outGeo->Desc.Triangles.pTransform != nullptr )
                            outGeo->Desc.Triangles.pTransform = allocs.ShallowCopy<Matrix>( outGeo->Desc.Triangles.pTransform, 1 );

                        size_t vertex_size = static_cast<size_t>(outGeo->Desc.Triangles.VertexBufferByteStride);
                        size_t num_verts = static_cast<size_t>(outGeo->Desc.Triangles.VertexCount);
                        auto index_format = outGeo->Desc.Triangles.indexFormat;

                        if ( index_format != INDEX_FORMAT_NONE )
                        {
                            uint32_t num_tris = outGeo->Desc.Triangles.IndexCount / 3;

                            // de-index indexed geos
                            const uint8_t* ib = (const uint8_t*)outGeo->Desc.Triangles.pIndexBuffer;
                            const uint8_t* vb = (const uint8_t*)outGeo->Desc.Triangles.pVertexBuffer;
                            uint8_t* flat_vb = (uint8_t*)allocs.Malloc( vertex_size * num_tris );

                            outGeo->Desc.Triangles.pVertexBuffer = flat_vb;
                            outGeo->Desc.Triangles.indexFormat = INDEX_FORMAT_NONE;
                            outGeo->Desc.Triangles.IndexCount = 0;
                            outGeo->Desc.Triangles.VertexCount = num_tris * 3;
                            outGeo->Desc.Triangles.pIndexBuffer = nullptr;

                            for ( size_t i = 0; i < num_tris * 3; i++ )
                            {
                                const uint8_t* old_vert;
                                switch ( index_format )
                                {
                                case INDEX_FORMAT_R16_UINT:
                                {
                                    old_vert = vb + vertex_size * (*((uint16_t*)ib));
                                    ib += sizeof( uint16_t );
                                }
                                break;
                                case INDEX_FORMAT_R32_UINT:
                                {
                                    old_vert = vb + vertex_size * (*((uint32_t*)ib));
                                    ib += sizeof( uint32_t );
                                }
                                break;
                                default:
                                    assert( false );
                                    continue;
                                }

                                memcpy( flat_vb, old_vert, vertex_size );
                                flat_vb += vertex_size;
                            }

                        }
                        else
                        {
                            // copy the existing VB
                            size_t VBSize = vertex_size * num_verts;
                            outGeo->Desc.Triangles.pVertexBuffer = allocs.ShallowCopy<char>( (const char*)outGeo->Desc.Triangles.pVertexBuffer, VBSize );
                        }
                    }
                    else
                    {
                        // copy AABB data
                        size_t nAABBBufferSize = static_cast<size_t>(outGeo->Desc.Procedural.AABBCount * outGeo->Desc.Procedural.AABBByteStride);
                        outGeo->Desc.Procedural.pAABBs = allocs.ShallowCopy<char>( (const char*)outGeo->Desc.Procedural.pAABBs, nAABBBufferSize );
                    }
                };

                for ( size_t g = 0; g < out.NumGeos; g++ )
                    _PROCESS_GEO( &out.pGeos[g] );
                for ( size_t g = 0; g < out.NumInitialGeos; g++ )
                    _PROCESS_GEO( &out.pInitialGeos[g] );


                // rebase blas pointers
                for ( size_t i = 0; i < in.NumInstances; i++ )
                {
                    Instance& outInst = out.pInstances[i];
                    const Instance& inInst = in.pInstances[i];
                    outInst.pRTAS = output->pRTAS + (inInst.pRTAS - input->pRTAS);
                }
                for ( size_t i = 0; i < in.NumInitialInstances; i++ )
                {
                    Instance& outInst = out.pInitialInstances[i];
                    const Instance& inInst = in.pInitialInstances[i];
                    outInst.pRTAS = output->pRTAS + (inInst.pRTAS - input->pRTAS);
                }
            }
        }

        output->pOpaqueAllocation = allocs.ExportAllocs();
    }

}
