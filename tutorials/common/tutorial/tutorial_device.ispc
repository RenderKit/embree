// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "tutorial_device.isph"
#include "../math/random_sampler.isph"
#include "../math/sampling.isph"
#include "scene_device.h"

/* the scene to render */
extern RTCScene g_scene;
extern uniform float g_debug;

/* returns the point seen through specified pixel */
export uniform bool device_pick(const uniform float x,
                                const uniform float y,
                                const uniform ISPCCamera& camera,
                                uniform Vec3f& hitPos)
{
  /* initialize ray */
  uniform Ray1 ray;
  ray.org = make_Vec3f_(camera.xfm.p);
  ray.dir = make_Vec3f_(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz));
  ray.tnear = 0.0f;
  ray.tfar = inf;
  ray.geomID = RTC_INVALID_GEOMETRY_ID;
  ray.primID = RTC_INVALID_GEOMETRY_ID;
  ray.mask = -1;
  ray.time = g_debug;

  /* intersect ray with scene */
  rtcIntersect1(g_scene,RTCRayHit1_(ray));

  /* shade pixel */
  if (ray.geomID == RTC_INVALID_GEOMETRY_ID) {
    hitPos = make_Vec3f(0.0f,0.0f,0.0f);
    return false;
  }
  else {
    hitPos = ray.org + ray.tfar*ray.dir;
    return true;
  }
}

Vec2f getTextureCoordinatesSubdivMesh(void* uniform _mesh, const unsigned int primID, const float u, const float v)
{
  uniform ISPCSubdivMesh *uniform mesh = (uniform ISPCSubdivMesh *uniform )_mesh;
  Vec2f st;
  st.x = u;
  st.y = v;
  if (mesh && mesh->texcoord_indices)
    {
      assert(primID < mesh->numFaces);
      const unsigned int face_offset = mesh->face_offsets[primID];
      if (mesh->verticesPerFace[primID] == 3)
	{
	  const unsigned int t0 = mesh->texcoord_indices[face_offset+0];
	  const unsigned int t1 = mesh->texcoord_indices[face_offset+1];
	  const unsigned int t2 = mesh->texcoord_indices[face_offset+2];
	  const Vec2f txt0 = mesh->texcoords[t0];
	  const Vec2f txt1 = mesh->texcoords[t1];
	  const Vec2f txt2 = mesh->texcoords[t2];
	  const float w = 1.0f - u - v;
	  st = w * txt0 + u * txt1 + v * txt2;
	}
      else if (mesh->verticesPerFace[primID] == 4)
	{
	  const unsigned int t0 = mesh->texcoord_indices[face_offset+0];
	  const unsigned int t1 = mesh->texcoord_indices[face_offset+1];
	  const unsigned int t2 = mesh->texcoord_indices[face_offset+2];
	  const unsigned int t3 = mesh->texcoord_indices[face_offset+3];
	  const Vec2f txt0 = mesh->texcoords[t0];
	  const Vec2f txt1 = mesh->texcoords[t1];
	  const Vec2f txt2 = mesh->texcoords[t2];
	  const Vec2f txt3 = mesh->texcoords[t3];
	  const float u0 = u;
	  const float v0 = v;
	  const float u1 = 1.0f - u;
	  const float v1 = 1.0f - v;
	  st = u1*v1 * txt0 + u0*v1* txt1 + u0*v0 * txt2 + u1*v0* txt3;
	}
    }
  return st;
}

float getTextureTexel1f(const uniform Texture* uniform texture, float s, float t)
{
  if (!texture) return 0.0f;

  int iu = (int)floor(s * (float)(texture->width));
  iu = iu % texture->width; if (iu < 0) iu += texture->width;
  int iv = (int)floor(t * (float)(texture->height));
  iv = iv % texture->height; if (iv < 0) iv += texture->height;

  if (texture->format == Texture_FLOAT32)
  {
    uniform float *uniform data = (uniform float *uniform)texture->data;
    return data[iv*texture->width + iu];
  }
  else if (texture->format == Texture_RGBA8)
  {
    const int offset = (iv * texture->width + iu) * 4;
    uniform unsigned int8 * uniform t = (uniform unsigned int8* uniform)texture->data;
    return t[offset+0]*(1.0f/255.0f);
  }
  return 0.0f;
}

Vec3f getTextureTexel3f(const uniform Texture* uniform texture, float s, float t)
{
  if (!texture) return make_Vec3f(0.0f,0.0f,0.0f);

  int iu = (int)floor(s * (float)(texture->width));
  iu = iu % texture->width; if (iu < 0) iu += texture->width;
  int iv = (int)floor(t * (float)(texture->height));
  iv = iv % texture->height; if (iv < 0) iv += texture->height;

  if (texture->format == Texture_RGBA8)
  {
    const int offset = (iv * texture->width + iu) * 4;
    uniform unsigned int8 * uniform t = (uniform unsigned int8* uniform)texture->data;
    const unsigned int8  r = t[offset+0];
    const unsigned int8  g = t[offset+1];
    const unsigned int8  b = t[offset+2];
    return make_Vec3f(  (float)r * 1.0f/255.0f, (float)g * 1.0f/255.0f, (float)b * 1.0f/255.0f );
  }
  return make_Vec3f(0.0f,0.0f,0.0f);
}
