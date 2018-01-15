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

#pragma once

#include "texture.h"
#include "../math/vec.h"

namespace embree {

struct Texture2D;

typedef Vec4f (*Texture2D_get)(const Texture2D *self,
                                       const Vec2f &p);

struct Texture2D {
  Vec2i         size;
  Vec2f         sizef;     // size, as floats; slightly smaller than 'size' to avoid range checks
  Vec2f         halfTexel; // 0.5/size, needed for bilinear filtering and clamp-to-edge
  Texture2D_get get;
  void         *data;
};

// XXX won't work with MIPmapping: clean implementation with clamping on integer coords needed then 
inline Vec2f clamp2edge(const Texture2D *self, const Vec2f p)
{
  return clamp(p, self->halfTexel, 1.0f - self->halfTexel);
}

/*! helper function that returns the sampled value for the first
  channel of the given texture

  Right now, this function always asks the texture for all four
  channels, and then discards all but one; later implementations may
  have specialized 'get1f' methods with the texture

  \note self may NOT be nullptr!
*/
inline float get1f(const Texture2D *self,
                   const Vec2f where)
{
  Vec4f ret = self->get(self, where);
  return ret.x;
}

/*! helper function that returns the sampled value for the first three
  channels of the given texture

  Right now, this function always asks the texture for all four
  channels, and then discards all but one; later implementations may
  have specialized 'get3f' methods with the texture

  \note self may NOT be nullptr!
*/
inline Vec3fa get3f(const Texture2D *self,
                   const Vec2f where)
{
  Vec4f ret = self->get(self, where);
  return Vec3fa(ret);
}

/*! helper function that returns the sampled value of the four
  channels of the given texture.

  Note that it's up to the texture to define clearly what happens if
  we ask for four channels even if the texture has less physical
  channels.

  \note self may NOT be nullptr!
*/
inline Vec4f get4f(const Texture2D *self,
                   const Vec2f where)
{
  return self->get(self, where);
}

/*! helper function: get1f() with a default value if the texture is nullptr */
inline float get1f(const Texture2D *self,
                   const Vec2f where,
                   const float defaultValue)
{
  if (self == nullptr) return defaultValue;
  else return get1f(self,where);
}

/*! helper function: get3f() with a default value if the texture is nullptr */
inline Vec3fa get3f(const Texture2D *self,
                   const Vec2f where,
                   const Vec3fa& defaultValue)
{
  if (self == nullptr) return defaultValue;
  else return get3f(self,where);
}

/*! helper function: get4f() with a default value if the texture is nullptr */
inline Vec4f get4f(const Texture2D *self,
                   const Vec2f where,
                   const Vec4f defaultValue)
{
  if (self == nullptr) return defaultValue;
  else return get4f(self,where);
}

} // namespace embree
