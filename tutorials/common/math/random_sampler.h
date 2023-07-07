// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../math/vec.h"
#include "sobol_matrices.h"

namespace embree {

__forceinline float safe_sample(float sample) {
  return min(sample, 1.f-(float)ulp);
}

__forceinline float to_float_unorm(uint32_t x)
{
  return float(x) * 2.3283064365386962890625e-10f; // x / 2^32
}
  

struct RandomSampler
{
  unsigned int s;
};

__forceinline unsigned int MurmurHash3_mix(unsigned int hash, unsigned int k)
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

__forceinline unsigned int MurmurHash3_finalize(unsigned int hash)
{
  hash ^= hash >> 16;
  hash *= 0x85ebca6b;
  hash ^= hash >> 13;
  hash *= 0xc2b2ae35;
  hash ^= hash >> 16;

  return hash;
}

__forceinline unsigned int hashToRandom(unsigned int value, unsigned int scramble)
{
  value = (value ^ 61) ^ scramble;
  value += value << 3;
  value ^= value >> 4;
  value *= 0x27d4eb2d;
  return value;
}
  

__forceinline unsigned int LCG_next(unsigned int value)
{
  const unsigned int m = 1664525;
  const unsigned int n = 1013904223;

  return value * m + n;
}

__forceinline void RandomSampler_init(RandomSampler& self, int id)
{
  unsigned int hash = 0;
  hash = MurmurHash3_mix(hash, id);
  hash = MurmurHash3_finalize(hash);

  self.s = hash;
}

__forceinline void RandomSampler_init(RandomSampler& self, int pixelId, int sampleId)
{
  unsigned int hash = 0;
  hash = MurmurHash3_mix(hash, pixelId);
  hash = MurmurHash3_mix(hash, sampleId);
  hash = MurmurHash3_finalize(hash);

  self.s = hash;
}

__forceinline void RandomSampler_init(RandomSampler& self, int x, int y, int sampleId)
{
  RandomSampler_init(self, x | (y << 16), sampleId);
}

__forceinline int RandomSampler_getInt(RandomSampler& self) {
  self.s = LCG_next(self.s); return self.s >> 1;
}

__forceinline unsigned int RandomSampler_getUInt(RandomSampler& self) {
  self.s = LCG_next(self.s); return self.s;
}

__forceinline float RandomSampler_getFloat(RandomSampler& self) {
  return (float)RandomSampler_getInt(self) * 4.656612873077392578125e-10f;
}

__forceinline float RandomSampler_get1D(RandomSampler& self) {
  return RandomSampler_getFloat(self);
}

__forceinline Vec2f RandomSampler_get2D(RandomSampler& self)
{
  const float u = RandomSampler_get1D(self);
  const float v = RandomSampler_get1D(self);
  return Vec2f(u,v);
}

__forceinline Vec3fa RandomSampler_get3D(RandomSampler& self)
{
  /*
  const float u = RandomSampler_get1D(self);
  const float v = RandomSampler_get1D(self);
  const float w = RandomSampler_get1D(self);
  return Vec3fa(u,v,w);
  */
  const int u = RandomSampler_getUInt(self);
  const int v = RandomSampler_getUInt(self);
  const int w = RandomSampler_getUInt(self);
  return Vec3fa(srl(Vec3ia(u,v,w), 1)) * 4.656612873077392578125e-10f;
}


class PCGSampler
{
public:

  __forceinline void InitSampler(unsigned x, unsigned y, unsigned sampleID, unsigned dim = 0)
  {
    unsigned pixelID = x | (y << 16);
    this->state = 0;
    this->stream = (sampleID << 1u) | 1u;

    // hash seed to reduce correlation artefacts
    this->state = MurmurHash3_mix(this->state, pixelID);
    this->state = MurmurHash3_finalize(this->state);

    RandomSampler__pcg32();
    this->state += pixelID;
    RandomSampler__pcg32();
  }

  __forceinline float Get1D() 
  {
    return GetFloat();
  }

  __forceinline Vec2f Get2D()
  {
  const float u = Get1D();
  const float v = Get1D();
  return Vec2f(u,v);
  }

  __forceinline float Get1D_uniform()
  {
    return Get1D();
  }
  
  __forceinline Vec2f Get2D_uniform()
  {
    return Get2D();
  }

  __forceinline Vec2f GetPixel2D()
  {
    return Get2D();
  }

private:

  __forceinline unsigned RandomSampler__pcg32()
  {
    unsigned long oldstate = this->state;
    this->state = oldstate * 6364136223846793005ULL + (this->stream | 1u);
    unsigned xorshifted = ((oldstate >> 18u) ^ oldstate) >> 27u;
    unsigned rot = oldstate >> 59u;
    return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

  __forceinline float GetFloat() 
  {
    return safe_sample((float)(RandomSampler__pcg32() >> 1)
        * 4.656612873077392578125e-10f);
  }

  unsigned long state;
  unsigned long stream;
};

class SobolSampler
{
public:

  __forceinline void InitSampler(unsigned pixelID, unsigned sampleID, unsigned dim)
  {
    // skip the first few samples to reduce correlation artifacts
    this->index = sampleID + 64;

    unsigned hash = 0;
    hash = MurmurHash3_mix(hash, pixelID);
    this->scramble = MurmurHash3_finalize(hash);

    hash = MurmurHash3_mix(hash, sampleID);
    this->dimension = dim;
  }

  __forceinline float Get1D() {
    return GetFloat();
  }

  __forceinline Vec2f Get2D()
  {
    const float u = Get1D();
    const float v = Get1D();
    return Vec2f(u,v);
  }

private:

  __forceinline float GetFloat()
  {
    // MAXIMUM 1024 dimensions!!!

    // Sample the Sobol sequence
    const float s = Sobol_sample(index, dimension);

    // Apply Cranley-Patterson rotation to reduce correlation artifacts
    const float shift = to_float_unorm(hashToRandom(dimension, scramble));
    dimension++;
    return safe_sample(CranleyPattersonRotation(s, shift));
  }

  unsigned index; // sample index
  unsigned scramble; // random number for scrambling the samples
  unsigned dimension;
};  

} // namespace embree
