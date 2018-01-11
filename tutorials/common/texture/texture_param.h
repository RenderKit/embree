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

#include "texture2d.h"
#include "ospray/math/AffineSpace.ih"

namespace embree {


//! Texture2D including coordinate transformation, plus helpers

struct TextureParam
{
  Texture2D* map;
  affine2f xform;
};

inline TextureParam make_TextureParam(Texture2D * tex, const affine2f &xform)
{
  TextureParam t;
  t.map = tex;
  t.xform = xform;
  return t;
}

inline bool valid(const TextureParam &tex)
{
  return tex.map;
}

inline float get1f(const TextureParam &tex,
                   const Vec2f uv)
{
  return get1f(tex.map, tex.xform * uv);
}

inline float get1f(const TextureParam &tex,
                   const Vec2f uv,
                   const float defaultValue)
{
  if (tex.map == nullptr)
    return defaultValue;
  return get1f(tex.map, tex.xform * uv);
}

inline Vec3fa get3f(const TextureParam &tex,
                   const Vec2f uv)
{
  return get3f(tex.map, tex.xform * uv);
}

inline Vec3fa get3f(const TextureParam &tex,
                   const Vec2f uv,
                   const Vec3fa& defaultValue)
{
  if (tex.map == nullptr)
    return defaultValue;
  return get3f(tex.map, tex.xform * uv);
}

inline Vec4f get4f(const TextureParam &tex,
                   const Vec2f uv)
{
  return get4f(tex.map, tex.xform * uv);
}

inline Vec4f get4f(const TextureParam &tex,
                   const Vec2f uv,
                   const Vec4f defaultValue)
{
  if (tex.map == nullptr)
    return defaultValue;
  return get4f(tex.map, tex.xform * uv);
}


} // namespace embree
