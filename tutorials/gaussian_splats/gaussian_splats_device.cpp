// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include "gaussian_splats_device.h"
#include <algorithm>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace embree {

#define FEATURE_MASK \
  RTC_FEATURE_FLAG_TRIANGLE | \
  RTC_FEATURE_FLAG_USER_GEOMETRY_CALLBACK_IN_GEOMETRY

RTCScene g_scene = nullptr;
TutorialData data;
std::string g_plyFilePath;

void gaussian_splats_set_ply_file(const std::string& filePath)
{
  g_plyFilePath = filePath;
}

namespace
{
  enum class PlyFormat { ASCII, BINARY_BIG_ENDIAN, BINARY_LITTLE_ENDIAN };

  enum class PlyType
  {
    CHAR,
    UCHAR,
    SHORT,
    USHORT,
    INT,
    UINT,
    FLOAT,
    DOUBLE
  };

  struct PlyProperty
  {
    std::string name;
    bool isList;
    PlyType type;
    PlyType listCountType;
    PlyType listDataType;
  };

  struct PlyElement
  {
    std::string name;
    size_t count;
    std::vector<PlyProperty> properties;
  };

  static PlyType plyTypeFromString(const std::string& typeName)
  {
    if (typeName == "char" || typeName == "int8") return PlyType::CHAR;
    if (typeName == "uchar" || typeName == "uint8") return PlyType::UCHAR;
    if (typeName == "short" || typeName == "int16") return PlyType::SHORT;
    if (typeName == "ushort" || typeName == "uint16") return PlyType::USHORT;
    if (typeName == "int" || typeName == "int32") return PlyType::INT;
    if (typeName == "uint" || typeName == "uint32") return PlyType::UINT;
    if (typeName == "float" || typeName == "float32") return PlyType::FLOAT;
    if (typeName == "double" || typeName == "float64") return PlyType::DOUBLE;
    throw std::runtime_error("unsupported PLY property type: " + typeName);
  }

  template<typename T>
  static T readBinaryValue(std::ifstream& stream, const PlyFormat format)
  {
    T value {};
    stream.read((char*) &value, sizeof(T));
    if (!stream) throw std::runtime_error("failed reading PLY binary data");

#if defined(__BYTE_ORDER__) && __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
    const bool needsSwap = format == PlyFormat::BINARY_LITTLE_ENDIAN;
#else
    const bool needsSwap = format == PlyFormat::BINARY_BIG_ENDIAN;
#endif

    if (needsSwap) {
      unsigned char* bytes = (unsigned char*) &value;
      for (size_t i = 0; i < sizeof(T) / 2; ++i)
        std::swap(bytes[i], bytes[sizeof(T) - 1 - i]);
    }

    return value;
  }

  static double readPlyScalar(std::ifstream& stream, const PlyFormat format, const PlyType type)
  {
    if (format == PlyFormat::ASCII) {
      std::string token;
      stream >> token;
      if (!stream) throw std::runtime_error("failed reading PLY ASCII data");
      return std::stod(token);
    }

    switch (type) {
    case PlyType::CHAR:   return (double) readBinaryValue<int8_t>(stream, format);
    case PlyType::UCHAR:  return (double) readBinaryValue<uint8_t>(stream, format);
    case PlyType::SHORT:  return (double) readBinaryValue<int16_t>(stream, format);
    case PlyType::USHORT: return (double) readBinaryValue<uint16_t>(stream, format);
    case PlyType::INT:    return (double) readBinaryValue<int32_t>(stream, format);
    case PlyType::UINT:   return (double) readBinaryValue<uint32_t>(stream, format);
    case PlyType::FLOAT:  return (double) readBinaryValue<float>(stream, format);
    case PlyType::DOUBLE: return (double) readBinaryValue<double>(stream, format);
    }

    return 0.0;
  }

  static bool hasProperty(const std::unordered_map<std::string, float>& values, const std::string& name)
  {
    return values.find(name) != values.end();
  }

  static float getProperty(const std::unordered_map<std::string, float>& values, const std::string& name, float defaultValue)
  {
    const auto it = values.find(name);
    return it == values.end() ? defaultValue : it->second;
  }

  static Vec4f normalizeQuaternion(const Vec4f& q)
  {
    const float lengthSquared = dot(q, q);
    if (!(lengthSquared >= 1.0e-20f) || !std::isfinite(lengthSquared))
      return Vec4f(1.0f, 0.0f, 0.0f, 0.0f);
    return q * rsqrt(lengthSquared);
  }

  static Vec3fa rotateVector(const Vec4f& q, const Vec3fa& v)
  {
    const Vec3fa qv(q.y, q.z, q.w);
    const Vec3fa t = 2.0f * cross(qv, v);
    return v + q.x * t + cross(qv, t);
  }

  static Vec3fa inverseRotateVector(const Vec4f& q, const Vec3fa& v)
  {
    return rotateVector(Vec4f(q.x, -q.y, -q.z, -q.w), v);
  }

  static float decodeOpacity(float opacity, bool encoded)
  {
    if (encoded)
      opacity = 1.0f / (1.0f + exp(-opacity));
    return clamp(opacity, 0.01f, 1.0f);
  }

  static Vec3fa decodeColor(const std::unordered_map<std::string, float>& values)
  {
    if (hasProperty(values, "r") && hasProperty(values, "g") && hasProperty(values, "b")) {
      float r = getProperty(values, "r", 0.0f);
      float g = getProperty(values, "g", 0.0f);
      float b = getProperty(values, "b", 0.0f);
      const float maxValue = max(r, max(g, b));
      if (maxValue > 1.0f) {
        r *= 1.0f / 255.0f;
        g *= 1.0f / 255.0f;
        b *= 1.0f / 255.0f;
      }
      return Vec3fa(clamp(r, 0.0f, 1.0f), clamp(g, 0.0f, 1.0f), clamp(b, 0.0f, 1.0f));
    }

    if (hasProperty(values, "red") && hasProperty(values, "green") && hasProperty(values, "blue")) {
      const float r = getProperty(values, "red", 0.0f) * (1.0f / 255.0f);
      const float g = getProperty(values, "green", 0.0f) * (1.0f / 255.0f);
      const float b = getProperty(values, "blue", 0.0f) * (1.0f / 255.0f);
      return Vec3fa(clamp(r, 0.0f, 1.0f), clamp(g, 0.0f, 1.0f), clamp(b, 0.0f, 1.0f));
    }

    if (hasProperty(values, "f_dc_0") && hasProperty(values, "f_dc_1") && hasProperty(values, "f_dc_2")) {
      const float c0 = 0.28209479177387814f;
      const float r = 0.5f + c0 * getProperty(values, "f_dc_0", 0.0f);
      const float g = 0.5f + c0 * getProperty(values, "f_dc_1", 0.0f);
      const float b = 0.5f + c0 * getProperty(values, "f_dc_2", 0.0f);
      return Vec3fa(clamp(r, 0.0f, 1.0f), clamp(g, 0.0f, 1.0f), clamp(b, 0.0f, 1.0f));
    }

    return Vec3fa(0.8f, 0.8f, 0.8f);
  }

  static Vec3fa decodeScale(const std::unordered_map<std::string, float>& values)
  {
    if (hasProperty(values, "scale_0")) {
      const float scaleX = exp(getProperty(values, "scale_0", -2.0f));
      const float scaleY = exp(getProperty(values, "scale_1", getProperty(values, "scale_0", -2.0f)));
      const float scaleZ = exp(getProperty(values, "scale_2", getProperty(values, "scale_0", -2.0f)));
      return Vec3fa(max(1.0e-6f, scaleX), max(1.0e-6f, scaleY), max(1.0e-6f, scaleZ));
    }

    const float sx = hasProperty(values, "scale_x") ? abs(getProperty(values, "scale_x", 0.1f)) : 0.1f;
    const float sy = hasProperty(values, "scale_y") ? abs(getProperty(values, "scale_y", sx)) : sx;
    const float sz = hasProperty(values, "scale_z") ? abs(getProperty(values, "scale_z", sx)) : sx;
    return Vec3fa(max(1.0e-6f, sx), max(1.0e-6f, sy), max(1.0e-6f, sz));
  }

  static Vec4f decodeRotation(const std::unordered_map<std::string, float>& values)
  {
    if (!hasProperty(values, "rot_0"))
      return Vec4f(1.0f, 0.0f, 0.0f, 0.0f);

    return normalizeQuaternion(Vec4f(getProperty(values, "rot_0", 1.0f),
                                     getProperty(values, "rot_1", 0.0f),
                                     getProperty(values, "rot_2", 0.0f),
                                     getProperty(values, "rot_3", 0.0f)));
  }

  static Vec3fa clampGaussianScale(const Vec3fa& scale)
  {
    const float minimumScale = 1.0e-6f;
    const Vec3fa finiteScale(
      std::isfinite(scale.x) && scale.x > 0.0f ? scale.x : minimumScale,
      std::isfinite(scale.y) && scale.y > 0.0f ? scale.y : minimumScale,
      std::isfinite(scale.z) && scale.z > 0.0f ? scale.z : minimumScale);
    const float maxScale = max(finiteScale.x, max(finiteScale.y, finiteScale.z));
    return max(finiteScale, Vec3fa(maxScale * 1.0e-3f));
  }

  static bool evaluateGaussian(const GaussianSplat& splat,
                               const Ray& ray,
                               float& t,
                               float& particleOpacity)
  {
    if (!is_finite(splat.center) || !std::isfinite(splat.opacity))
      return false;

    const Vec3fa scale = clampGaussianScale(splat.scale);
    const Vec4f rotation = normalizeQuaternion(splat.rotation);

    const Vec3fa oRotated = inverseRotateVector(rotation, ray.org - splat.center);
    const Vec3fa dRotated = inverseRotateVector(rotation, ray.dir);
    const Vec3fa oGaussian(oRotated.x / scale.x, oRotated.y / scale.y, oRotated.z / scale.z);
    const Vec3fa dGaussian(dRotated.x / scale.x, dRotated.y / scale.y, dRotated.z / scale.z);

    const float denominator = dot(dGaussian, dGaussian);
    if (!(denominator > 1.0e-20f) || !std::isfinite(denominator))
      return false;

    t = -dot(oGaussian, dGaussian) / denominator;
    if (!std::isfinite(t) || t < ray.tnear() || t > ray.tfar)
      return false;

    const Vec3fa xGaussian = oGaussian + t * dGaussian;
    particleOpacity = splat.opacity * exp(-0.5f * dot(xGaussian, xGaussian));
    return std::isfinite(particleOpacity) && particleOpacity >= 0.01f;
  }

  static void loadGaussianSplatsFromPly(const std::string& filePath)
  {
    std::ifstream stream(filePath.c_str(), std::ios::in | std::ios::binary);
    if (!stream.is_open())
      throw std::runtime_error("cannot open PLY file: " + filePath);

    std::string line;
    std::getline(stream, line);
    if (line != "ply")
      throw std::runtime_error("invalid PLY signature in file: " + filePath);

    PlyFormat format = PlyFormat::ASCII;
    std::vector<PlyElement> elements;
    PlyElement* currentElement = nullptr;

    while (std::getline(stream, line))
    {
      if (line == "end_header")
        break;
      if (line.empty() || line[0] == '#')
        continue;

      std::stringstream headerLine(line);
      std::string tag;
      headerLine >> tag;

      if (tag == "comment") {
        continue;
      } else if (tag == "format") {
        std::string formatName;
        std::string version;
        headerLine >> formatName >> version;
        if (version != "1.0")
          throw std::runtime_error("unsupported PLY version: " + version);
        if (formatName == "ascii") format = PlyFormat::ASCII;
        else if (formatName == "binary_big_endian") format = PlyFormat::BINARY_BIG_ENDIAN;
        else if (formatName == "binary_little_endian") format = PlyFormat::BINARY_LITTLE_ENDIAN;
        else throw std::runtime_error("unsupported PLY format: " + formatName);
      } else if (tag == "element") {
        PlyElement element;
        headerLine >> element.name >> element.count;
        elements.push_back(element);
        currentElement = &elements.back();
      } else if (tag == "property") {
        if (!currentElement)
          throw std::runtime_error("PLY property declared before any element");
        std::string typeName;
        headerLine >> typeName;
        PlyProperty property {};
        property.isList = false;
        if (typeName == "list") {
          std::string countTypeName, dataTypeName;
          headerLine >> countTypeName >> dataTypeName >> property.name;
          property.isList = true;
          property.listCountType = plyTypeFromString(countTypeName);
          property.listDataType = plyTypeFromString(dataTypeName);
          property.type = property.listDataType;
        } else {
          headerLine >> property.name;
          property.type = plyTypeFromString(typeName);
          property.listCountType = PlyType::UCHAR;
          property.listDataType = PlyType::UCHAR;
        }
        currentElement->properties.push_back(property);
      }
    }

    std::vector<GaussianSplat> splats;
    std::vector<Vec3fa> colors;

    for (const PlyElement& element : elements)
    {
      const bool isVertex = element.name == "vertex";
      if (isVertex) {
        splats.reserve(element.count);
        colors.reserve(element.count);
      }

      for (size_t i = 0; i < element.count; ++i)
      {
        std::unordered_map<std::string, float> values;

        for (const PlyProperty& property : element.properties)
        {
          if (property.isList) {
            const size_t count = (size_t) readPlyScalar(stream, format, property.listCountType);
            for (size_t j = 0; j < count; ++j)
              (void) readPlyScalar(stream, format, property.listDataType);
          } else {
            const float value = (float) readPlyScalar(stream, format, property.type);
            if (isVertex)
              values[property.name] = value;
          }
        }

        if (!isVertex)
          continue;

        if (!hasProperty(values, "x") || !hasProperty(values, "y") || !hasProperty(values, "z"))
          throw std::runtime_error("PLY vertex element must provide x, y, z properties");

        GaussianSplat splat {};
        splat.center = Vec3fa(getProperty(values, "x", 0.0f),
                              getProperty(values, "y", 0.0f),
                              getProperty(values, "z", 0.0f));

        splat.scale = decodeScale(values);
        splat.rotation = decodeRotation(values);
        splat.opacity = decodeOpacity(getProperty(values, "opacity", 1.0f),
                                      hasProperty(values, "scale_0"));
        splat.colorID = (unsigned int) splats.size();
        splats.push_back(splat);
        colors.push_back(decodeColor(values));
      }
    }

    if (splats.empty())
      throw std::runtime_error("PLY file does not contain any vertex data: " + filePath);

    TutorialData_ResizeSplats(&data, (unsigned int) splats.size());
    for (unsigned int i = 0; i < data.splatCount; ++i) {
      data.splats[i] = splats[i];
      data.colors[i] = colors[i];
    }
  }

  static void generateRandomGaussianSplats()
  {
    TutorialData_ResizeSplats(&data, DEFAULT_NUM_SPLATS);

    RandomSampler rng;
    RandomSampler_init(rng, 1337);

    for (unsigned int i = 0; i < data.splatCount; ++i)
    {
      const float px = 8.0f * RandomSampler_get1D(rng) - 4.0f;
      const float py = 2.5f * RandomSampler_get1D(rng) - 0.5f;
      const float pz = 8.0f * RandomSampler_get1D(rng) - 4.0f;

      data.splats[i].center = Vec3fa(px, py, pz);
      data.splats[i].scale = Vec3fa(0.06f + 0.18f * RandomSampler_get1D(rng),
                                    0.06f + 0.18f * RandomSampler_get1D(rng),
                                    0.06f + 0.18f * RandomSampler_get1D(rng));
      data.splats[i].rotation = Vec4f(1.0f, 0.0f, 0.0f, 0.0f);
      data.splats[i].opacity = 0.25f + 0.75f * RandomSampler_get1D(rng);
      data.splats[i].colorID = i;

      const float cr = 0.2f + 0.8f * RandomSampler_get1D(rng);
      const float cg = 0.2f + 0.8f * RandomSampler_get1D(rng);
      const float cb = 0.2f + 0.8f * RandomSampler_get1D(rng);
      data.colors[i] = Vec3fa(cr, cg, cb);
    }
  }
}

void splatBoundsFunc(const RTCOrientedBoundsFunctionArguments* args)
{
  const GaussianSplat* splats = (const GaussianSplat*) args->geometryUserPtr;
  const GaussianSplat& s = splats[args->primID];
  RTCOrientedBounds* bounds = args->bounds_o;

  const Vec3fa scale = clampGaussianScale(s.scale);
  const Vec4f rotation = normalizeQuaternion(s.rotation);
  const Vec3fa axis0 = 3.0f * rotateVector(rotation, Vec3fa(scale.x, 0.0f, 0.0f));
  const Vec3fa axis1 = 3.0f * rotateVector(rotation, Vec3fa(0.0f, scale.y, 0.0f));
  const Vec3fa axis2 = 3.0f * rotateVector(rotation, Vec3fa(0.0f, 0.0f, scale.z));

  bounds->center_x = s.center.x;
  bounds->center_y = s.center.y;
  bounds->center_z = s.center.z;
  bounds->axis0_x = axis0.x;
  bounds->axis0_y = axis0.y;
  bounds->axis0_z = axis0.z;
  bounds->axis1_x = axis1.x;
  bounds->axis1_y = axis1.y;
  bounds->axis1_z = axis1.z;
  bounds->axis2_x = axis2.x;
  bounds->axis2_y = axis2.y;
  bounds->axis2_z = axis2.z;
}

RTC_SYCL_INDIRECTLY_CALLABLE void splatIntersectFunc(const RTCIntersectFunctionNArguments* args)
{
  int* valid = args->valid;
  Ray* ray = (Ray*) args->rayhit;
  RTCHit* hit = (RTCHit*) &ray->Ng.x;

  if (args->N != 1 || !valid[0])
    return;

  const GaussianSplat* splats = (const GaussianSplat*) args->geometryUserPtr;
  const GaussianSplat& s = splats[args->primID];

  float t;
  float particleOpacity;
  if (!evaluateGaussian(s, *ray, t, particleOpacity))
    return;

  ray->tfar = t;
  hit->geomID = args->geomID;
  hit->primID = args->primID;
  hit->Ng_x = -ray->dir.x;
  hit->Ng_y = -ray->dir.y;
  hit->Ng_z = -ray->dir.z;
  hit->u = particleOpacity;
  hit->v = 0.0f;
  valid[0] = -1;
}

RTC_SYCL_INDIRECTLY_CALLABLE void splatOccludedFunc(const RTCOccludedFunctionNArguments* args)
{
  int* valid = args->valid;
  Ray* ray = (Ray*) args->ray;

  if (args->N != 1 || !valid[0])
    return;

  const GaussianSplat* splats = (const GaussianSplat*) args->geometryUserPtr;
  const GaussianSplat& s = splats[args->primID];

  float t;
  float particleOpacity;
  if (evaluateGaussian(s, *ray, t, particleOpacity) && particleOpacity >= 0.05f)
    ray->tfar = neg_inf;
}

unsigned int addGroundPlane(RTCScene scene)
{
  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_TRIANGLE);

  Vertex* vertices = (Vertex*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, sizeof(Vertex), 4);
  vertices[0].x = -12.0f; vertices[0].y = -2.0f; vertices[0].z = -12.0f;
  vertices[1].x = -12.0f; vertices[1].y = -2.0f; vertices[1].z = +12.0f;
  vertices[2].x = +12.0f; vertices[2].y = -2.0f; vertices[2].z = -12.0f;
  vertices[3].x = +12.0f; vertices[3].y = -2.0f; vertices[3].z = +12.0f;

  Triangle* triangles = (Triangle*) rtcSetNewGeometryBuffer(geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, sizeof(Triangle), 2);
  triangles[0].v0 = 0; triangles[0].v1 = 1; triangles[0].v2 = 2;
  triangles[1].v0 = 1; triangles[1].v1 = 3; triangles[1].v2 = 2;

  rtcCommitGeometry(geom);
  unsigned int geomID = rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);
  return geomID;
}

void addGaussianSplats(RTCScene scene)
{
  if (g_plyFilePath.empty()) {
    generateRandomGaussianSplats();
  } else {
    loadGaussianSplatsFromPly(g_plyFilePath);
  }

  RTCGeometry geom = rtcNewGeometry(g_device, RTC_GEOMETRY_TYPE_USER_ORIENTED);
  rtcSetGeometryUserPrimitiveCount(geom, data.splatCount);
  rtcSetGeometryUserData(geom, data.splats);
  rtcSetGeometryOrientedBoundsFunction(geom, splatBoundsFunc, nullptr);
  rtcSetGeometryIntersectFunction(geom, splatIntersectFunc);
  rtcSetGeometryOccludedFunction(geom, splatOccludedFunc);
  rtcCommitGeometry(geom);
  rtcAttachGeometry(scene, geom);
  rtcReleaseGeometry(geom);
}

void renderPixelStandard(const TutorialData& data,
                         int x, int y,
                         int* pixels,
                         const unsigned int width,
                         const unsigned int height,
                         const float time,
                         const ISPCCamera& camera,
                         RayStats& stats)
{
  Ray ray(Vec3fa(camera.xfm.p), Vec3fa(normalize(x * camera.xfm.l.vx + y * camera.xfm.l.vy + camera.xfm.l.vz)), 0.0f, inf);

  RTCIntersectArguments iargs;
  rtcInitIntersectArguments(&iargs);
  iargs.feature_mask = (RTCFeatureFlags)(FEATURE_MASK);
  rtcTraversableIntersect1(data.g_traversable, RTCRayHit_(ray), &iargs);
  RayStats_addRay(stats);

  Vec3fa color(0.02f, 0.03f, 0.05f);
  if (ray.geomID != RTC_INVALID_GEOMETRY_ID)
  {
    if (ray.geomID == 0) {
      const Vec3fa Ng = normalize(ray.Ng);
      const float checker = ((int)floor(0.5f * ray.org.x + ray.tfar * ray.dir.x) ^ (int)floor(0.5f * ray.org.z + ray.tfar * ray.dir.z)) & 1;
      const Vec3fa base = checker ? Vec3fa(0.15f, 0.15f, 0.18f) : Vec3fa(0.35f, 0.35f, 0.4f);
      const Vec3fa L = normalize(Vec3fa(-1.0f, -1.0f, -1.0f));
      const float d = clamp(-dot(L, Ng), 0.0f, 1.0f);
      color = base * (0.2f + 0.8f * d);
    } else {
      const float w = clamp(ray.u, 0.0f, 1.0f);
      const Vec3fa base = data.colors[ray.primID];
      const Vec3fa Ng = normalize(ray.Ng);
      const Vec3fa L = normalize(Vec3fa(-1.0f, -1.0f, -1.0f));
      const float d = clamp(-dot(L, Ng), 0.0f, 1.0f);
      color = base * (0.1f + 0.9f * d) * w;

      Ray shadow(ray.org + ray.tfar * ray.dir, neg(L), 0.001f, inf, 0.0f);
      RTCOccludedArguments sargs;
      rtcInitOccludedArguments(&sargs);
      sargs.feature_mask = (RTCFeatureFlags)(FEATURE_MASK);
      rtcTraversableOccluded1(data.g_traversable, RTCRay_(shadow), &sargs);
      RayStats_addShadowRay(stats);

      if (shadow.tfar < 0.0f)
        color *= 0.55f;
    }
  }

  const unsigned int r = (unsigned int)(255.0f * clamp(color.x, 0.0f, 1.0f));
  const unsigned int g = (unsigned int)(255.0f * clamp(color.y, 0.0f, 1.0f));
  const unsigned int b = (unsigned int)(255.0f * clamp(color.z, 0.0f, 1.0f));
  pixels[y * width + x] = (b << 16) + (g << 8) + r;
}

void renderTileTask(int taskIndex, int threadIndex, int* pixels,
                    const unsigned int width,
                    const unsigned int height,
                    const float time,
                    const ISPCCamera& camera,
                    const int numTilesX,
                    const int numTilesY)
{
  const unsigned int tileY = taskIndex / numTilesX;
  const unsigned int tileX = taskIndex - tileY * numTilesX;
  const unsigned int x0 = tileX * TILE_SIZE_X;
  const unsigned int x1 = min(x0 + TILE_SIZE_X, width);
  const unsigned int y0 = tileY * TILE_SIZE_Y;
  const unsigned int y1 = min(y0 + TILE_SIZE_Y, height);

  for (unsigned int y = y0; y < y1; y++) {
    for (unsigned int x = x0; x < x1; x++)
      renderPixelStandard(data, x, y, pixels, width, height, time, camera, g_stats[threadIndex]);
  }
}

extern "C" void renderFrameStandard(int* pixels,
                                     const unsigned int width,
                                     const unsigned int height,
                                     const float time,
                                     const ISPCCamera& camera)
{
  const int numTilesX = (width + TILE_SIZE_X - 1) / TILE_SIZE_X;
  const int numTilesY = (height + TILE_SIZE_Y - 1) / TILE_SIZE_Y;
  parallel_for(size_t(0), size_t(numTilesX * numTilesY), [&](const range<size_t>& range) {
    const int threadIndex = (int)TaskScheduler::threadIndex();
    for (size_t i = range.begin(); i < range.end(); i++)
      renderTileTask((int)i, threadIndex, pixels, width, height, time, camera, numTilesX, numTilesY);
  });
}

extern "C" void device_init(const char* cfg)
{
  _unused(cfg);
  TutorialData_Constructor(&data);
  g_scene = data.g_scene = rtcNewScene(g_device);

  addGroundPlane(g_scene);
  addGaussianSplats(g_scene);

  rtcCommitScene(g_scene);
  data.g_traversable = rtcGetSceneTraversable(data.g_scene);
}

extern "C" void device_render(int* pixels,
                               const unsigned int width,
                               const unsigned int height,
                               const float time,
                               const ISPCCamera& camera)
{
  renderFrameStandard(pixels, width, height, time, camera);
}

extern "C" void device_cleanup()
{
  TutorialData_Destructor(&data);
}

} // namespace embree
