// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "../../../common/sys/platform.h"
#include "../../../common/sys/ref.h"
#include "../../../common/math/math.h"
#include "../../../common/math/vec3.h"
#include "../../../common/math/affinespace.h"
#include "../scenegraph/scenegraph.h"
#include <sstream>

namespace embree
{
  /* camera settings */
  struct Camera
  {
    enum Handedness {
      LEFT_HANDED,
      RIGHT_HANDED
    };
    
    struct ISPCCamera
    {
    public:
      ISPCCamera (const AffineSpace3fa& xfm)
      : xfm(xfm) {}

    public:
      AffineSpace3fa xfm;
    };

  public:

    Camera ()
    : from(0.0001f,0.0001f,-3.0f), to(0,0,0), up(0,1,0), fov(90), handedness(RIGHT_HANDED) {}

    Camera (const Vec3fa& from, const Vec3fa& to, const Vec3fa& up, float fov, Handedness handedness)
    : from(from), to(to), up(up), fov(fov), handedness(handedness) {}

    Camera (const SceneGraph::PerspectiveCameraData& cam, Handedness handedness)
    : from(cam.from), to(cam.to), up(cam.up), fov(cam.fov), handedness(handedness) {}

    std::string str() const 
    {
      std::stringstream stream;
      stream.precision(10);
      stream << "--vp " << from.x    << " " << from.y    << " " << from.z    << " " 
             << "--vi " << to.x << " " << to.y << " " << to.z << " " 
             << "--vu " << up.x     << " " << up.y     << " " << up.z     << " " 
             << "--fov " << fov << " "
             << (handedness == LEFT_HANDED ? "--lefthanded" : "--righthanded");
      return stream.str();
    }
    
    AffineSpace3fa camera2world ()
    {
      AffineSpace3fa local2world = AffineSpace3fa::lookat(from, to, up);
      if (!(local2world == local2world))
        throw std::runtime_error("invalid camera specified");
      
      if (handedness == RIGHT_HANDED)
        local2world.l.vx = -local2world.l.vx;
      
      return local2world;
    }
    AffineSpace3fa world2camera () { return rcp(camera2world()); }
    Vec3fa world2camera(const Vec3fa& p) { return xfmPoint(world2camera(),p); }
    Vec3fa camera2world(const Vec3fa& p) { return xfmPoint(camera2world(),p); }

    ISPCCamera getISPCCamera (size_t width, size_t height)
    {
      const float fovScale = 1.0f/tanf(deg2rad(0.5f*fov));
      const AffineSpace3fa local2world = camera2world();
      Vec3fa vx = local2world.l.vx;
      Vec3fa vy = -local2world.l.vy;
      Vec3fa vz = -0.5f*width*local2world.l.vx + 0.5f*height*local2world.l.vy + 0.5f*height*fovScale*local2world.l.vz;
      Vec3fa p =  local2world.p;
      return ISPCCamera(AffineSpace3fa(vx,vy,vz,p));
    }

    void move (float dx, float dy, float dz)
    {
      AffineSpace3fa xfm = camera2world();
      Vec3fa ds = xfmVector(xfm,Vec3fa(dx,dy,dz));
      from += ds;
      to   += ds;
    }

    void rotate (float dtheta, float dphi)
    {
      if (handedness == RIGHT_HANDED) dtheta *= -1.0f;
      const Vec3fa up1 = normalize(up);
      Vec3fa view1 = normalize(to-from);
      view1 = xfmVector(AffineSpace3fa::rotate(up1, dtheta), view1);
      const float phi = acosf(dot(view1, up1));
      const float dphi2 = phi - clamp(phi-dphi, 0.001f*float(pi), 0.999f*float(pi));
      view1 = xfmVector(AffineSpace3fa::rotate(cross(view1, up1), dphi2), view1);
      to = from + length(to-from) * view1;
    }

    void rotateOrbit (float dtheta, float dphi)
    {
      if (handedness == RIGHT_HANDED) dtheta *= -1.0f;
      const Vec3fa up1 = normalize(up);
      Vec3fa view1 = normalize(to-from);
      view1 = xfmVector(AffineSpace3fa::rotate(up1, dtheta), view1);
      const float phi = acosf(dot(view1, up1));
      const float dphi2 = phi - clamp(phi-dphi, 0.001f*float(pi), 0.999f*float(pi));
      view1 = xfmVector(AffineSpace3fa::rotate(cross(view1, up1), dphi2), view1);
      from = to - length(to-from) * view1;
    }

    void dolly (float ds)
    {
      float dollySpeed = 0.01f;
      float k = powf((1.0f-dollySpeed), ds);
      from += length(to-from) * (1-k) * normalize(to-from);
    }

  public:
    Vec3fa from;   //!< position of camera
    Vec3fa to;     //!< look at point
    Vec3fa up;     //!< up vector
    float fov;     //!< field of view
    Handedness handedness;
  };

  typedef Camera::ISPCCamera ISPCCamera;
}
