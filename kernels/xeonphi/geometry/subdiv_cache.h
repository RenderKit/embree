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

#pragma once

#include "subdivpatch1.h"
#include "common/ray16.h"
#include "geometry/filter.h"
#include "bvh4i/bvh4i.h"

#define SUBDIV_CACHE_ENTRIES 2
#define SUBDIV_CACHE_BVH_NODES_PER_ENTRY 1+4
#define SUBDIV_CACHE_LEAVES_PER_ENTRY 4

namespace embree
{
  class __aligned(64) SubdivCache {
  public:
    
    static const unsigned int INVALID = (unsigned int)-1;

    struct __aligned(16) Tag {
      unsigned int geomID;
      unsigned int primID;
      unsigned int subdivLevel;
      unsigned int index;

      Tag(const unsigned int geomID,
	  const unsigned int primID,
	  const unsigned int subdivLevel) : geomID(geomID), primID(primID), subdivLevel(subdivLevel)
      {
      }

      Tag(const unsigned int geomID,
	  const unsigned int primID,
	  const unsigned int subdivLevel,
	  const unsigned int index) : geomID(geomID), primID(primID), subdivLevel(subdivLevel), index(index)
      {
      }
      
      Tag() {}

    __forceinline bool inCache(const unsigned int _geomID,
			       const unsigned int _primID,
			       const unsigned int _subdivLevel)
    {
      if (likely(geomID      == _geomID &&
		 primID      == _primID &&
		 subdivLevel == _subdivLevel))
	return true;
      return false;
    }

    };

    struct __aligned(64) Entry {
      BVH4i::Node bvh4i_node[SUBDIV_CACHE_BVH_NODES_PER_ENTRY];
      Vec4f uv_interval[SUBDIV_CACHE_LEAVES_PER_ENTRY];

      __forceinline BVH4i::NodeRef getRoot()
	{
	  return bvh4i_node[0].child(0);
	}
    };

    Tag     tag[SUBDIV_CACHE_ENTRIES];
    Entry entry[SUBDIV_CACHE_ENTRIES];


    SubdivCache() 
      {
	DBG_PRINT(sizeof(SubdivCache));

	for (size_t i=0;i<SUBDIV_CACHE_ENTRIES;i++)
	  tag[i] = Tag(INVALID,INVALID,INVALID,i);

	memset(entry,0,sizeof(Entry)*SUBDIV_CACHE_ENTRIES);
      }

    __forceinline Tag &lookupEntry(const unsigned int geomID,
				   const unsigned int primID,
				   const unsigned int subdivLevel)
    {
      return tag[primID % SUBDIV_CACHE_ENTRIES];
    }


    __forceinline Entry &getEntry(Tag &current)
    {
      return entry[current.index];
    }

    __forceinline const Entry &getEntry(Tag &current) const
    {
      return entry[current.index];
    }

  };

  __forceinline std::ostream& operator<<(std::ostream &o, SubdivCache::Tag &tag)
    {
      o << "geomID " << tag.geomID << " primID " << tag.primID << " subdivLevel " << tag.subdivLevel << " index " << tag.index;
      return o;
    }


  __forceinline std::ostream& operator<<(std::ostream &o, SubdivCache &cache)
    {
      o << "subdiv cache entries " << SUBDIV_CACHE_ENTRIES << std::endl;
      o << "tags: " << std::endl;
      for (size_t i=0;i<SUBDIV_CACHE_ENTRIES;i++)
	o << "i " << i << " = " << cache.tag[i] << std::endl;
      o << "entries: " << std::endl;
      for (size_t i=0;i<SUBDIV_CACHE_ENTRIES;i++)
	{
	  o << "i " << i << std::endl;
	  for (size_t j=0;j<SUBDIV_CACHE_BVH_NODES_PER_ENTRY;j++)
	    o << "j " << j << " = " << cache.entry[i].bvh4i_node[j] << std::endl;
	  for (size_t j=0;j<SUBDIV_CACHE_LEAVES_PER_ENTRY;j++)
	    o << "j " << j << " = " << cache.entry[i].uv_interval[j] << std::endl;
	}
      return o;
    }
    
};
