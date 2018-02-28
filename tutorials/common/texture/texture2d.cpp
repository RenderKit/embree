// ======================================================================== //
// Copyright 2009-2018 Intel Corporation                                    //
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

#include "texture2d.h"

namespace embree {


// Low-level texel accessors
//////////////////////////////////////////////////////////////////////////////

// TODO blocking

inline Vec4f getTexel_RGBA8(const Texture2D *self, const Vec2i i)
{
  assert(self);
  const uint32_t c = ((const uint32_t *)self->data)[i.y*self->size.x + i.x];
  const uint32_t r = c         & 0xff;
  const uint32_t g = (c >>  8) & 0xff;
  const uint32_t b = (c >> 16) & 0xff;
  const uint32_t a = c >> 24;
  return Vec4f((float)r, (float)g, (float)b, (float)a)*(1.f/255.f);
}

inline Vec4f getTexel_RGB8(const Texture2D *self, const Vec2i i)
{
  assert(self);
  const uint8_t *texel = (const uint8_t *)self->data;
  const uint32_t texelOfs = 3*(i.y*self->size.x + i.x);
  const uint32_t r = texel[texelOfs];
  const uint32_t g = texel[texelOfs+1];
  const uint32_t b = texel[texelOfs+2];
  return Vec4f(Vec3fa((float)r, (float)g, (float)b)*(1.f/255.f), 1.f);
}

inline Vec4f getTexel_R8(const Texture2D *self, const Vec2i i)
{
  assert(self);
  const uint8_t c = ((const uint8_t *)self->data)[i.y*self->size.x + i.x];
  return Vec4f(c*(1.f/255.f), 0.0f, 0.0f, 1.f);
}

inline Vec4f getTexel_SRGBA(const Texture2D *self, const Vec2i i)
{
  return srgba_to_linear(getTexel_RGBA8(self, i));
}

inline Vec4f getTexel_SRGB(const Texture2D *self, const Vec2i i)
{
  return srgba_to_linear(getTexel_RGB8(self, i));
}

inline Vec4f getTexel_RGBA32F(const Texture2D *self, const Vec2i i)
{
  assert(self);
  return ((const Vec4f *)self->data)[i.y*self->size.x + i.x];
}

inline Vec4f getTexel_RGB32F(const Texture2D *self, const Vec2i i)
{
  assert(self);
  Vec3fa v = ((const Vec3fa*)self->data)[i.y*self->size.x + i.x];
  return Vec4f(v, 1.f);
}

inline Vec4f getTexel_R32F(const Texture2D *self, const Vec2i i)
{
  assert(self);
  float v = ((const float*)self->data)[i.y*self->size.x + i.x];
  return Vec4f(v, 0.f, 0.f, 1.f);
}


// Texture coordinate utilities
//////////////////////////////////////////////////////////////////////////////

inline Vec2i nearest_coords(const Texture2D *self, const Vec2f p)
{
  // repeat: get remainder within [0..1] parameter space
  Vec2f tc = frac(p);
  tc = max(tc, Vec2f(0.0f)); // filter out inf/NaN

  // scale by texture size
  tc = tc * self->sizef;

  // nearest
  return Vec2i(tc);
}

struct BilinCoords {
  Vec2i st0;
  Vec2i st1;
  Vec2f frac;
};

inline BilinCoords bilinear_coords(const Texture2D *self, const Vec2f p)
{
  BilinCoords coords;

  // repeat: get remainder within [0..1] parameter space
  // lower sample shifted by half a texel
  Vec2f tc = frac(p - self->halfTexel);
  tc = max(tc, Vec2f(0.0f)); // filter out inf/NaN

  // scale by texture size
  tc = tc * self->sizef;
  coords.frac = frac(tc);

  coords.st0 = Vec2i(tc);
  coords.st1 = coords.st0 + 1;
  // handle border cases
  if (coords.st1.x >= self->size.x)
    coords.st1.x = 0;
  if (coords.st1.y >= self->size.y)
    coords.st1.y = 0;

  return coords;
}

inline Vec4f bilerp(const Vec2f frac, const Vec4f c00, const Vec4f c01, const Vec4f c10, const Vec4f c11)
{
  return lerpr(frac.y,
              lerpr(frac.x, c00, c01),
              lerpr(frac.x, c10, c11));
}


// Implementations of Texture2D_get for different formats and filter modi
//////////////////////////////////////////////////////////////////////////////

#define __define_tex_get(FMT)                                                \
                                                                             \
static Vec4f Texture2D_nearest_##FMT(const Texture2D *self,  \
    const Vec2f &p)                                                          \
{                                                                            \
  return getTexel_##FMT(self, nearest_coords(self, p));                      \
}                                                                            \
                                                                             \
static Vec4f Texture2D_bilinear_##FMT(const Texture2D *self, \
    const Vec2f &p)                                                          \
{                                                                            \
  BilinCoords cs = bilinear_coords(self, p);                                 \
                                                                             \
  const Vec4f c00 = getTexel_##FMT(self, Vec2i(cs.st0.x, cs.st0.y));    \
  const Vec4f c01 = getTexel_##FMT(self, Vec2i(cs.st1.x, cs.st0.y));    \
  const Vec4f c10 = getTexel_##FMT(self, Vec2i(cs.st0.x, cs.st1.y));    \
  const Vec4f c11 = getTexel_##FMT(self, Vec2i(cs.st1.x, cs.st1.y));    \
                                                                             \
  return bilerp(cs.frac, c00, c01, c10, c11);                                \
}

#define __define_tex_get_case(FMT) \
  case TEXTURE_##FMT: return filter_nearest ?  &Texture2D_nearest_##FMT : \
                                                   &Texture2D_bilinear_##FMT;

#define __foreach_fetcher(FCT) \
  FCT(RGBA8)                   \
  FCT(SRGBA)                   \
  FCT(RGBA32F)                 \
  FCT(RGB8)                    \
  FCT(SRGB)                    \
  FCT(RGB32F)                  \
  FCT(R8)                      \
  FCT(R32F)   

__foreach_fetcher(__define_tex_get)

static Texture2D_get Texture2D_get_addr(const uint32_t type,
    const bool filter_nearest)
{
  switch (type) {
    __foreach_fetcher(__define_tex_get_case)
  }
  return 0;
};

#undef __define_tex_get
#undef __define_tex_get_addr
#undef __foreach_fetcher


// Exports (called from C++)
//////////////////////////////////////////////////////////////////////////////

extern "C" void *Texture2D_create(Vec2i &size, void *data,
    uint32_t type, uint32_t flags)
{
  Texture2D *self = (Texture2D*) alignedMalloc(sizeof(Texture2D),16);
  self->size      = size;

  // Due to float rounding frac(x) can be exactly 1.0f (e.g. for very small
  // negative x), although it should be strictly smaller than 1.0f. We handle
  // this case by having sizef slightly smaller than size, such that
  // frac(x)*sizef is always < size.
  self->sizef = Vec2f(nextafter((float)size.x, -1.0f), nextafter((float)size.y, -1.0f));
  self->halfTexel = Vec2f(0.5f/size.x, 0.5f/size.y);
  self->data = data;
  self->get = Texture2D_get_addr(type, flags & TEXTURE_FILTER_NEAREST);

  return self;
}

} // namespace embree
