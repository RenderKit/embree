//                                                                          //
// Unless required by applicable law or agreed to in writing, software      //
// distributed under the License is distributed on an "AS IS" BASIS,        //
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. //
// See the License for the specific language governing permissions and      //
// limitations under the License.                                           //
// ======================================================================== //

#pragma once

#include "bezier1i.h"
#include "common/ray16.h"
#include "geometry/filter.h"
#include "geometry/mailbox.h"

namespace embree
{
  typedef LinearSpace3<mic3f> LinearSpace_mic3f;
    
  /*! Intersector for a single ray from a ray packet with a bezier curve. */
  struct Bezier1iIntersector16
  {
    typedef Bezier1i Primitive;

    struct Precalculations 
    {
      __forceinline Precalculations (const Ray16& ray, const size_t k)
	: ray_space(frame(Vec3fa(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k])).transposed()) {} // FIXME: works only with normalized ray direction

      __forceinline Precalculations (const LinearSpace_mic3f& ls16, const size_t k)
	: ray_space(ls16.vx.x[k],ls16.vy.x[k],ls16.vz.x[k],
		    ls16.vx.y[k],ls16.vy.y[k],ls16.vz.y[k],
		    ls16.vx.z[k],ls16.vy.z[k],ls16.vz.z[k])
      {}
      LinearSpace3fa ray_space;
      Mailbox mbox;
    };

    static __forceinline bool intersect(const Precalculations& pre, Ray16& ray, const size_t k, const Bezier1i& curve_in, const void* geom)
    {
      STAT3(normal.trav_prims,1,1,1);

      /* load ray */
      const Vec3fa ray_org(ray.org.x[k],ray.org.y[k],ray.org.z[k]);
      const Vec3fa ray_dir(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
      const float ray_tnear = ray.tnear[k];
      const float ray_tfar  = ray.tfar [k];
      
      prefetch<PFHINT_L1>((char*)curve_in.p + 0);
      prefetch<PFHINT_L1>((char*)curve_in.p + 2);

      /* load bezier curve control points */
      const Vec3fa& v0 = curve_in.p[0];
      const Vec3fa& v1 = curve_in.p[1];
      const Vec3fa& v2 = curve_in.p[2];
      const Vec3fa& v3 = curve_in.p[3];

      /* transform control points into ray space */
      const Vec3fa v0_ray_org = v0-ray_org;
      const Vec3fa v1_ray_org = v1-ray_org;
      const Vec3fa v2_ray_org = v2-ray_org;
      const Vec3fa v3_ray_org = v3-ray_org;

      Vec3fa w0 = xfmVector(pre.ray_space,v0_ray_org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(pre.ray_space,v1_ray_org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(pre.ray_space,v2_ray_org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(pre.ray_space,v3_ray_org); w3.w = v3.w;
      BezierCurve3D curve2D(w0,w1,w2,w3,0.0f,1.0f,4);


      /* subdivide 3 levels at once */ 
      const mic4f p0 = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const mic4f p1 = curve2D.eval(coeff1[0],coeff1[1],coeff1[2],coeff1[3]); // FIXME: can be calculated from p0 by shifting, use just 15 segments

      /* approximative intersection with cone */
      const mic4f v = p1-p0;
      const mic4f w = -p0;
      const mic_f d0 = w.x*v.x + w.y*v.y;
      const mic_f d1 = v.x*v.x + v.y*v.y;
      const mic_f u = clamp(d0*rcp(d1),mic_f(zero),mic_f(one));
      const mic4f p = p0 + u*v;
      const mic_f t = p.z;
      const mic_f d2 = p.x*p.x + p.y*p.y; 
      const mic_f r = p.w;
      const mic_f r2 = r*r;
      mic_m valid = d2 <= r2 & mic_f(ray_tnear) < t & t < mic_f(ray_tfar);
      const float one_over_width = 1.0f/16.0f;


      if (unlikely(none(valid))) return false;
      STAT3(normal.trav_prim_hits,1,1,1);
      size_t i = select_min(valid,t);

      /* ray masking test */
#if defined(__USE_RAY_MASK__)
      BezierCurves* g = ((Scene*)geom)->getBezierCurves(curve_in.geomID);
      if (unlikely(g->mask & ray.mask[k]) == 0) return false;
#endif  

      /* intersection filter test */

      /* update hit information */
      const float uu = (float(i)+u[i])*one_over_width; // FIXME: correct u range for subdivided segments
      const BezierCurve3D curve3D(v0,v1,v2,v3,0.0f,1.0f,0);
      Vec3fa P,T; curve3D.eval(uu,P,T);
      assert( T != Vec3fa(zero) );
      //if (T == Vec3fa(zero)) { valid ^= (1 << i); PING; goto retry; } // ignore denormalized curves
      ray.u[k] = uu;
      ray.v[k] = 0.0f;
      ray.tfar[k] = t[i];
      ray.Ng.x[k] = T.x;
      ray.Ng.y[k] = T.y;
      ray.Ng.z[k] = T.z;
      ray.geomID[k] = curve_in.geomID;
      ray.primID[k] = curve_in.primID;

      return true;
    }

    static __forceinline bool occluded(const Precalculations& pre, const Ray16& ray, const size_t k, const Bezier1i& curve_in, const void* geom) 
    {
      STAT3(shadow.trav_prims,1,1,1);

      /* load ray */
      const Vec3fa ray_org(ray.org.x[k],ray.org.y[k],ray.org.z[k]);
      const Vec3fa ray_dir(ray.dir.x[k],ray.dir.y[k],ray.dir.z[k]);
      const float ray_tnear = ray.tnear[k];
      const float ray_tfar  = ray.tfar [k];

      /* load bezier curve control points */
      const Vec3fa v0 = curve_in.p[0];
      const Vec3fa v1 = curve_in.p[1];
      const Vec3fa v2 = curve_in.p[2];
      const Vec3fa v3 = curve_in.p[3];

      const Vec3fa v0_ray_org = v0-ray_org;
      const Vec3fa v1_ray_org = v1-ray_org;
      const Vec3fa v2_ray_org = v2-ray_org;
      const Vec3fa v3_ray_org = v3-ray_org;

      /* transform control points into ray space */
      Vec3fa w0 = xfmVector(pre.ray_space,v0_ray_org); w0.w = v0.w;
      Vec3fa w1 = xfmVector(pre.ray_space,v1_ray_org); w1.w = v1.w;
      Vec3fa w2 = xfmVector(pre.ray_space,v2_ray_org); w2.w = v2.w;
      Vec3fa w3 = xfmVector(pre.ray_space,v3_ray_org); w3.w = v3.w;
      BezierCurve3D curve2D(w0,w1,w2,w3,0.0f,1.0f,4);

      /* subdivide 3 levels at once */ 
      const mic4f p0 = curve2D.eval(coeff0[0],coeff0[1],coeff0[2],coeff0[3]);
      const mic4f p1 = curve2D.eval(coeff1[0],coeff1[1],coeff1[2],coeff1[3]);

      /* approximative intersection with cone */
      const mic4f v = p1-p0;
      const mic4f w = -p0;
      const mic_f d0 = w.x*v.x + w.y*v.y;
      const mic_f d1 = v.x*v.x + v.y*v.y;
      const mic_f u = clamp(d0*rcp(d1),mic_f(zero),mic_f(one));
      const mic4f p = p0 + u*v;
      const mic_f t = p.z;
      const mic_f d2 = p.x*p.x + p.y*p.y; 
      const mic_f r = p.w;
      const mic_f r2 = r*r;
      mic_m valid = d2 <= r2 & mic_f(ray_tnear) < t & t < mic_f(ray_tfar);
      const float one_over_width = 1.0f/16.0f;

      if (none(valid)) return false;
      STAT3(shadow.trav_prim_hits,1,1,1);

      /* ray masking test */
#if defined(__USE_RAY_MASK__)
      BezierCurves* g = ((Scene*)geom)->getBezierCurves(curve_in.geomID);
      if (unlikely(g->mask & ray.mask[k]) == 0) return false;
#endif  

      /* intersection filter test */
      return true;
    }

  };
}
