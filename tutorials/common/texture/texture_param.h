// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

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
