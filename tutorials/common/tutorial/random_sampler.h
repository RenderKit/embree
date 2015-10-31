// ======================================================================== //
// Copyright 2009-2015 Intel Corporation                                    //
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

#include "../../../common/math/vec2.h"

struct RandomSampler
{
  unsigned int s;
};

inline unsigned int MurmurHash3__mix(unsigned int hash, unsigned int k)
{
  const unsigned int c1 = 0xcc9e2d51;
  const unsigned int c2 = 0x1b873593;
  const unsigned int r1 = 15;
  const unsigned int r2 = 13;
  const unsigned int m = 5;
  const unsigned int n = 0xe6546b64;

  k *= c1;
  k = (k << r1) | (k >> (32 - r1));
  k *= c2;

  hash ^= k;
  hash = ((hash << r2) | (hash >> (32 - r2))) * m + n;

  return hash;
}

inline unsigned int MurmurHash3__finalize(unsigned int hash)
{
  hash ^= hash >> 16;
  hash *= 0x85ebca6b;
  hash ^= hash >> 13;
  hash *= 0xc2b2ae35;
  hash ^= hash >> 16;

  return hash;
}

inline void RandomSampler__init(RandomSampler& This, int pixelId, int sampleId)
{
  unsigned int hash = 0;
  hash = MurmurHash3__mix(hash, pixelId);
  hash = MurmurHash3__mix(hash, sampleId);
  hash = MurmurHash3__finalize(hash);

  This.s = hash;
}

inline void RandomSampler__init(RandomSampler& This, int x, int y, int sampleId)
{
  RandomSampler__init(This, x | (y << 16), sampleId);
}

inline float RandomSampler__get1D(RandomSampler& This)
{
  const unsigned int m = 1664525;
  const unsigned int n = 1013904223;
  This.s = This.s * m + n;

  // avoid expensive uint->float conversion
  return (float)(int)(This.s >> 1) * 4.656612873077392578125e-10f;
}

inline Vec2f RandomSampler__get2D(RandomSampler& This)
{
  return Vec2f(RandomSampler__get1D(This), RandomSampler__get1D(This));
}
