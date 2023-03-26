#include "viewer_device.h"
#include "../common/tutorial/optics.h"
#include "../common/lights/ambient_light.cpp"
#include "../common/lights/directional_light.cpp"
#include "../common/lights/point_light.cpp"
#include "../common/lights/quad_light.cpp"
#include "../common/lights/spot_light.cpp"

namespace embree {

  // =================================================================================================================================================================================
  // ======================================================================== Simple Path Tracer =====================================================================================
  // =================================================================================================================================================================================  

  Light_SampleRes Lights_sample(const Light* self,
                                const DifferentialGeometry& dg, /*! point to generate the sample for >*/
                                const Vec2f s)                /*! random numbers to generate the sample >*/
  {
    TutorialLightType ty = self->type;
    switch (ty) {
    case LIGHT_AMBIENT    : return AmbientLight_sample(self,dg,s);
    case LIGHT_POINT      : return PointLight_sample(self,dg,s);
    case LIGHT_DIRECTIONAL: return DirectionalLight_sample(self,dg,s);
    case LIGHT_SPOT       : return SpotLight_sample(self,dg,s);
    case LIGHT_QUAD       : return QuadLight_sample(self,dg,s);
    default: {
      Light_SampleRes res;
      res.weight = Vec3fa(0,0,0);
      res.dir = Vec3fa(0,0,0);
      res.dist = 0;
      res.pdf = inf;
      return res;
    }
    }
  }
  
  Light_EvalRes Lights_eval(const Light* self,
                            const DifferentialGeometry& dg,
                            const Vec3fa& dir)
  {
    TutorialLightType ty = self->type;
    switch (ty) {
    case LIGHT_AMBIENT     : return AmbientLight_eval(self,dg,dir);
    case LIGHT_POINT       : return PointLight_eval(self,dg,dir);
    case LIGHT_DIRECTIONAL : return DirectionalLight_eval(self,dg,dir);
    case LIGHT_SPOT        : return SpotLight_eval(self,dg,dir);
    case LIGHT_QUAD        : return QuadLight_eval(self,dg,dir);
    default: {
      Light_EvalRes res;
      res.value = Vec3fa(0,0,0);
      res.dist = inf;
      res.pdf = 0.f;
      return res;
    }
    }
  }

////////////////////////////////////////////////////////////////////////////////
//                                 BRDF                                       //
////////////////////////////////////////////////////////////////////////////////

  struct BRDF
  {
    Vec3fa Kd;              /*< diffuse reflectivity */
  };

  struct Medium
  {
    Vec3fa transmission; //!< Transmissivity of medium.
    float eta;             //!< Refraction index of medium.
  };

  inline Medium make_Medium(const Vec3fa& transmission, const float eta)
  {
    Medium m;
    m.transmission = transmission;
    m.eta = eta;
    return m;
  }

  inline Medium make_Medium_Vacuum() {
    return make_Medium(Vec3fa((float)1.0f),1.0f);
  }

  inline bool eq(const Medium& a, const Medium& b) {
    return (a.eta == b.eta) && eq(a.transmission, b.transmission);
  }

  inline Vec3fa sample_component2(const Vec3fa& c0, const Sample3f& wi0, const Medium& medium0,
                                  const Vec3fa& c1, const Sample3f& wi1, const Medium& medium1,
                                  const Vec3fa& Lw, Sample3f& wi_o, Medium& medium_o, const float s)
  {
    const Vec3fa m0 = Lw*c0/wi0.pdf;
    const Vec3fa m1 = Lw*c1/wi1.pdf;

    const float C0 = wi0.pdf == 0.0f ? 0.0f : max(max(m0.x,m0.y),m0.z);
    const float C1 = wi1.pdf == 0.0f ? 0.0f : max(max(m1.x,m1.y),m1.z);
    const float C  = C0 + C1;

    if (C == 0.0f) {
      wi_o = make_Sample3f(Vec3fa(0,0,0),0);
      return Vec3fa(0,0,0);
    }

    const float CP0 = C0/C;
    const float CP1 = C1/C;
    if (s < CP0) {
      wi_o = make_Sample3f(wi0.v,wi0.pdf*CP0);
      medium_o = medium0; return c0;
    }
    else {
      wi_o = make_Sample3f(wi1.v,wi1.pdf*CP1);
      medium_o = medium1; return c1;
    }
  }



////////////////////////////////////////////////////////////////////////////////
//                          OBJ Material                                      //
////////////////////////////////////////////////////////////////////////////////

  void OBJMaterial__preprocess(ISPCOBJMaterial* material, BRDF& brdf, const Vec3fa& wo, const DifferentialGeometry& dg, const Medium& medium)
  {
    float d = material->d;
    //if (material->map_d) d *= getTextureTexel1f(material->map_d,dg.u,dg.v);
    brdf.Kd = d * Vec3fa(material->Kd);
    //if (material->map_Kd) brdf.Kd = brdf.Kd * getTextureTexel3f(material->map_Kd,dg.u,dg.v);
  }

  Vec3fa OBJMaterial__eval(ISPCOBJMaterial* material, const BRDF& brdf, const Vec3fa& wo, const DifferentialGeometry& dg, const Vec3fa& wi)
  {
    Vec3fa R = Vec3fa(0.0f);
    const float Md = max(max(brdf.Kd.x,brdf.Kd.y),brdf.Kd.z);
    if (Md > 0.0f) {
      R = R + (1.0f/float(M_PI)) * clamp(dot(wi,dg.Ns)) * brdf.Kd;
    }
    return R;
  }

  Vec3fa OBJMaterial__sample(ISPCOBJMaterial* material, const BRDF& brdf, const Vec3fa& Lw, const Vec3fa& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
  {
    Vec3fa cd = Vec3fa(0.0f);
    Sample3f wid = make_Sample3f(Vec3fa(0.0f),0.0f);
    if (max(max(brdf.Kd.x,brdf.Kd.y),brdf.Kd.z) > 0.0f) {
      wid = cosineSampleHemisphere(s.x,s.y,dg.Ns);
      cd = float(one_over_pi) * clamp(dot(wid.v,dg.Ns)) * brdf.Kd;
    }

    const Vec3fa md = Lw*cd/wid.pdf;

    const float Cd = wid.pdf == 0.0f ? 0.0f : max(max(md.x,md.y),md.z);
    const float C  = Cd;

    if (C == 0.0f) {
      wi_o = make_Sample3f(Vec3fa(0,0,0),0);
      return Vec3fa(0,0,0);
    }

    wi_o = make_Sample3f(wid.v,wid.pdf);
    return cd;
  }


////////////////////////////////////////////////////////////////////////////////
//                              Material                                      //
////////////////////////////////////////////////////////////////////////////////

  inline void Material__preprocess(ISPCMaterial** materials, unsigned int materialID, unsigned int numMaterials, BRDF& brdf, const Vec3fa& wo, const DifferentialGeometry& dg, const Medium& medium)
  {
    auto id = materialID;
    if (id < numMaterials) // FIXME: workaround for ISPC bug, location reached with empty execution mask
    {
      ISPCMaterial* material = materials[id];
      OBJMaterial__preprocess  ((ISPCOBJMaterial*)  material,brdf,wo,dg,medium);
    }
  }

  inline Vec3fa Material__eval(ISPCMaterial** materials, unsigned int materialID, unsigned int numMaterials, const BRDF& brdf, const Vec3fa& wo, const DifferentialGeometry& dg, const Vec3fa& wi)
  {
    Vec3fa c = Vec3fa(0.0f);
    auto id = materialID;
    if (id < numMaterials) // FIXME: workaround for ISPC bug, location reached with empty execution mask
    {
      ISPCMaterial* material = materials[id];
      c = OBJMaterial__eval  ((ISPCOBJMaterial*)  material, brdf, wo, dg, wi);
    }
    return c;
  }

  inline Vec3fa Material__sample(ISPCMaterial** materials, unsigned int materialID, unsigned int numMaterials, const BRDF& brdf, const Vec3fa& Lw, const Vec3fa& wo, const DifferentialGeometry& dg, Sample3f& wi_o, Medium& medium, const Vec2f& s)
  {
    Vec3fa c = Vec3fa(0.0f);
    auto id = materialID;
    if (id < numMaterials) // FIXME: workaround for ISPC bug, location reached with empty execution mask
    {
      ISPCMaterial* material = materials[id];
      c = OBJMaterial__sample  ((ISPCOBJMaterial*)  material, brdf, Lw, wo, dg, wi_o, medium, s);
    }
    return c;
  }

    
  inline int postIntersect(const TutorialData& data, const Ray& ray, DifferentialGeometry& dg)
  {
    dg.eps = 32.0f*1.19209e-07f*max(max(abs(dg.P.x),abs(dg.P.y)),max(abs(dg.P.z),ray.tfar));   
    int materialID = 0;
    return materialID;
  }

  inline Vec3fa face_forward(const Vec3fa& dir, const Vec3fa& _Ng) {
    const Vec3fa Ng = _Ng;
    return dot(dir,Ng) < 0.0f ? Ng : neg(Ng);
  }


  Vec3fa renderPixelFunction(const TutorialData& data, float x, float y, RandomSampler& sampler, const ISPCCamera& camera, GBuffer &gb, const RTCFeatureFlags features)
  {
    /* radiance accumulator and weight */
    Vec3fa L = Vec3fa(0.0f);
    Vec3fa Lw = Vec3fa(1.0f);
    Medium medium = make_Medium_Vacuum();
    float time = RandomSampler_get1D(sampler);

    /* initialize ray */
    Ray ray(Vec3fa(camera.xfm.p),
            Vec3fa(normalize(x*camera.xfm.l.vx + y*camera.xfm.l.vy + camera.xfm.l.vz)),0.0f,inf,time);

    DifferentialGeometry dg;
 
    /* iterative path tracer loop */
    for (int i=0; i<data.max_path_length; i++)
    {
      /* terminate if contribution too low */
      if (max(Lw.x,max(Lw.y,Lw.z)) < 0.01f)
        break;

      /* intersect ray with scene */
      RayQueryContext context;
      InitIntersectionContext(&context);
      context.tutorialData = (void*) &data;
    
      RTCIntersectArguments args;
      rtcInitIntersectArguments(&args);
      args.context = &context.context;
      args.feature_mask = features;
  
      rtcIntersect1(data.scene,RTCRayHit_(ray),&args);
      const Vec3fa wo = neg(ray.dir);

      /* invoke environment lights if nothing hit */
      if (ray.geomID == RTC_INVALID_GEOMETRY_ID)
      {
        //L = L + Lw*Vec3fa(1.0f);

        /* iterate over all lights */
        for (unsigned int i=0; i<data.ispc_scene->numLights; i++)
        {
          const Light* l = data.ispc_scene->lights[i];
          //Light_EvalRes le = l->eval(l,dg,ray.dir);
          Light_EvalRes le = Lights_eval(l,dg,ray.dir);
          L = L + Lw*le.value;
        }

        break;
      }

      Vec3fa Ns = normalize(ray.Ng);
      if (i == 0) gb.normal = fp_convert(Ns);
      
      /* compute differential geometry */
    
      dg.geomID = ray.geomID;
      dg.primID = ray.primID;
      dg.u = ray.u;
      dg.v = ray.v;
      dg.P  = ray.org+ray.tfar*ray.dir;
      dg.Ng = ray.Ng;
      dg.Ns = Ns;
      int materialID = postIntersect(data,ray,dg);
      dg.Ng = face_forward(ray.dir,normalize(dg.Ng));
      dg.Ns = face_forward(ray.dir,normalize(dg.Ns));

      /*! Compute  simple volumetric effect. */
      Vec3fa c = Vec3fa(1.0f);
      const Vec3fa transmission = medium.transmission;
      if (ne(transmission,Vec3fa(1.0f)))
        c = c * pow(transmission,ray.tfar);

      /* calculate BRDF */
      BRDF brdf;
      int numMaterials = data.ispc_scene->numMaterials;
      ISPCMaterial** material_array = &data.ispc_scene->materials[0];
      Material__preprocess(material_array,materialID,numMaterials,brdf,wo,dg,medium);

      if (i == 0) gb.albedo = fp_convert(brdf.Kd);
      
      /* sample BRDF at hit point */
      Sample3f wi1;
      c = c * Material__sample(material_array,materialID,numMaterials,brdf,Lw, wo, dg, wi1, medium, RandomSampler_get2D(sampler));

      /* iterate over lights */
      for (unsigned int i=0; i<data.ispc_scene->numLights; i++)
      {
        const Light* l = data.ispc_scene->lights[i];
        //Light_SampleRes ls = l->sample(l,dg,RandomSampler_get2D(sampler));
        Light_SampleRes ls = Lights_sample(l,dg,RandomSampler_get2D(sampler));
        if (ls.pdf <= 0.0f) continue;
        Vec3fa transparency = Vec3fa(1.0f);
        Ray shadow(dg.P,ls.dir,dg.eps,ls.dist,time);
        context.userRayExt = &transparency;

        RTCOccludedArguments sargs;
        rtcInitOccludedArguments(&sargs);
        sargs.context = &context.context;
        sargs.feature_mask = features;
        rtcOccluded1(data.scene,RTCRay_(shadow),&sargs);
        if (shadow.tfar > 0.0f)
          L = L + Lw*ls.weight*transparency*Material__eval(material_array,materialID,numMaterials,brdf,wo,dg,ls.dir);
      }

      if (wi1.pdf <= 1E-4f /* 0.0f */) break;
      Lw = Lw*c/wi1.pdf;

      /* setup secondary ray */
      float sign = dot(wi1.v,dg.Ng) < 0.0f ? -1.0f : 1.0f;
      dg.P = dg.P + sign*dg.eps*dg.Ng;
      init_Ray(ray, dg.P,normalize(wi1.v),dg.eps,inf,time);
    }
    return L;
  }



  Vec3f renderPixelPathTracer(const TutorialData& data,
                              int x, int y,
                              int* pixels,
                              const unsigned int width,
                              const unsigned int height,
                              const ISPCCamera& camera,
                              GBuffer &gb,
                              const RTCFeatureFlags features)
  {
    RandomSampler sampler;

    Vec3fa L = Vec3fa(0.0f);

    for (int i=0; i<data.spp; i++)
    {
      RandomSampler_init(sampler, x, y, data.spp+i);

      /* calculate pixel color */
      float fx = x + RandomSampler_get1D(sampler);
      float fy = y + RandomSampler_get1D(sampler);
      L = L + renderPixelFunction(data,fx,fy,sampler,camera,gb,features);
    }
    L = L/(float)data.spp;

    return Vec3f(L.x,L.y,L.z);
  }
  
  sycl::event renderFramePathTracer (int* pixels,
                                     const unsigned int width,
                                     const unsigned int height,
                                     const float time,
                                     const ISPCCamera& camera,
                                     TutorialData &data,
                                     uint user_spp,
                                     GBuffer *gbuffer,
                                     bool denoise)
  {
    /* render all pixels */
#if defined(EMBREE_SYCL_TUTORIAL)
    {
      TutorialData ldata = data;
      ldata.spp = user_spp;
#if 0
      int numMaterials = ldata.ispc_scene->numMaterials;      
      PRINT( numMaterials );
      PRINT(  ((ISPCOBJMaterial*)ldata.ispc_scene->materials[0])->Kd );
      PRINT(  ((ISPCOBJMaterial*)ldata.ispc_scene->materials[0])->Ks );   
#endif
      
      sycl::event event = global_gpu_queue->submit([=](sycl::handler& cgh) {
        const sycl::nd_range<2> nd_range = make_nd_range(height,width);
        cgh.parallel_for(nd_range,[=](sycl::nd_item<2> item) EMBREE_SYCL_SIMD(16) {
          const unsigned int x = item.get_global_id(1); if (x >= width ) return;
          const unsigned int y = item.get_global_id(0); if (y >= height) return;
          const RTCFeatureFlags feature_mask = RTC_FEATURE_FLAG_ALL;
          GBuffer gb;
          gb.normal = fp_convert(Vec3f(1,0,0));
          gb.albedo = fp_convert(Vec3f(0,0,0));
          const Vec3f c = renderPixelPathTracer(ldata,x,y,pixels,width,height,camera,gb,feature_mask);
          if (!denoise)
          {
            unsigned int r = (unsigned int) (255.01f * clamp(c.x,0.0f,1.0f));
            unsigned int g = (unsigned int) (255.01f * clamp(c.y,0.0f,1.0f));
            unsigned int b = (unsigned int) (255.01f * clamp(c.z,0.0f,1.0f));
            pixels[y*width+x] = (b << 16) + (g << 8) + r;  
          }
          else
          {
            gb.color = fp_convert(c);
            gbuffer[y*width+x] = gb;            
          }
          
        });
      });
      global_gpu_queue->wait_and_throw();
      return event;
    }
#endif
  }

};
