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

#include "texture2d.isph"


// Low-level texel accessors
//////////////////////////////////////////////////////////////////////////////

// TODO blocking

inline Vec4f getTexel_RGBA8(const uniform Texture2D *uniform self, const Vec2i i)
{
  assert(self);
  const uint32 c = ((const uniform uint32 *uniform)self->data)[i.y*self->size.x + i.x];
  const uint32 r = c         & 0xff;
  const uint32 g = (c >>  8) & 0xff;
  const uint32 b = (c >> 16) & 0xff;
  const uint32 a = c >> 24;
  return make_Vec4f((float)r, (float)g, (float)b, (float)a)*(1.f/255.f);
}

inline Vec4f getTexel_RGB8(const uniform Texture2D *uniform self, const Vec2i i)
{
  assert(self);
  const uniform uint8 *uniform texel = (const uniform uint8 *uniform)self->data;
  const uint32 texelOfs = 3*(i.y*self->size.x + i.x);
  const uint32 r = texel[texelOfs];
  const uint32 g = texel[texelOfs+1];
  const uint32 b = texel[texelOfs+2];
  return make_Vec4f(make_Vec3f((float)r, (float)g, (float)b)*(1.f/255.f), 1.f);
}

inline Vec4f getTexel_R8(const uniform Texture2D *uniform self, const Vec2i i)
{
  assert(self);
  const uint8 c = ((const uniform uint8 *uniform)self->data)[i.y*self->size.x + i.x];
  return make_Vec4f(c*(1.f/255.f), 0.0f, 0.0f, 1.f);
}

inline Vec4f getTexel_SRGBA(const uniform Texture2D *uniform self, const Vec2i i)
{
  return srgba_to_linear(getTexel_RGBA8(self, i));
}

inline Vec4f getTexel_SRGB(const uniform Texture2D *uniform self, const Vec2i i)
{
  return srgba_to_linear(getTexel_RGB8(self, i));
}

inline Vec4f getTexel_RGBA32F(const uniform Texture2D *uniform self, const Vec2i i)
{
  assert(self);
  return ((const uniform Vec4f *uniform)self->data)[i.y*self->size.x + i.x];
}

inline Vec4f getTexel_RGB32F(const uniform Texture2D *uniform self, const Vec2i i)
{
  assert(self);
  Vec3f v = ((const uniform Vec3f*uniform )self->data)[i.y*self->size.x + i.x];
  return make_Vec4f(v, 1.f);
}

inline Vec4f getTexel_R32F(const uniform Texture2D *uniform self, const Vec2i i)
{
  assert(self);
  float v = ((const uniform float*uniform)self->data)[i.y*self->size.x + i.x];
  return make_Vec4f(v, 0.f, 0.f, 1.f);
}


// Texture coordinate utilities
//////////////////////////////////////////////////////////////////////////////

inline Vec2i nearest_coords(const uniform Texture2D *uniform self, const Vec2f p)
{
  // repeat: get remainder within [0..1] parameter space
  Vec2f tc = frac(p);
  tc = max(tc, make_Vec2f(0.0f)); // filter out inf/NaN

  // scale by texture size
  tc = tc * self->sizef;

  // nearest
  return make_Vec2i(tc);
}

struct BilinCoords {
  Vec2i st0;
  Vec2i st1;
  Vec2f frac;
};

inline BilinCoords bilinear_coords(const uniform Texture2D *uniform self, const Vec2f p)
{
  BilinCoords coords;

  // repeat: get remainder within [0..1] parameter space
  // lower sample shifted by half a texel
  Vec2f tc = frac(p - self->halfTexel);
  tc = max(tc, make_Vec2f(0.0f)); // filter out inf/NaN

  // scale by texture size
  tc = tc * self->sizef;
  coords.frac = frac(tc);

  coords.st0 = make_Vec2i(tc);
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
  return lerp(frac.y,
              lerp(frac.x, c00, c01),
              lerp(frac.x, c10, c11));
}


// Implementations of Texture2D_get for different formats and filter modi
//////////////////////////////////////////////////////////////////////////////

#define __define_tex_get(FMT)                                                \
                                                                             \
static Vec4f Texture2D_nearest_##FMT(const uniform Texture2D *uniform self,  \
    const Vec2f &p)                                                          \
{                                                                            \
  return getTexel_##FMT(self, nearest_coords(self, p));                      \
}                                                                            \
                                                                             \
static Vec4f Texture2D_bilinear_##FMT(const uniform Texture2D *uniform self, \
    const Vec2f &p)                                                          \
{                                                                            \
  BilinCoords cs = bilinear_coords(self, p);                                 \
                                                                             \
  const Vec4f c00 = getTexel_##FMT(self, make_Vec2i(cs.st0.x, cs.st0.y));    \
  const Vec4f c01 = getTexel_##FMT(self, make_Vec2i(cs.st1.x, cs.st0.y));    \
  const Vec4f c10 = getTexel_##FMT(self, make_Vec2i(cs.st0.x, cs.st1.y));    \
  const Vec4f c11 = getTexel_##FMT(self, make_Vec2i(cs.st1.x, cs.st1.y));    \
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

static uniform Texture2D_get Texture2D_get_addr(const uniform uint32 type,
    const uniform bool filter_nearest)
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

export void *uniform Texture2D_create(uniform Vec2i &size, void *uniform data,
    uniform uint32 type, uniform uint32 flags)
{
  uniform Texture2D *uniform self = uniform new uniform Texture2D;
  self->size      = size;

  // Due to float rounding frac(x) can be exactly 1.0f (e.g. for very small
  // negative x), although it should be strictly smaller than 1.0f. We handle
  // this case by having sizef slightly smaller than size, such that
  // frac(x)*sizef is always < size.
  self->sizef = make_Vec2f(nextafter((float)size.x, -1.0f), nextafter((float)size.y, -1.0f));
  self->halfTexel = make_Vec2f(0.5f/size.x, 0.5f/size.y);
  self->data = data;
  self->get = Texture2D_get_addr(type, flags & TEXTURE_FILTER_NEAREST);

  return self;
}
