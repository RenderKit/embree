// Copyright 2026 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <embree4/rtcore.h>
#include <embree4/rtcore_builder.h>

#include <cassert>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>

#if defined(RTC_NAMESPACE_USE)
RTC_NAMESPACE_USE
#endif

namespace
{
  constexpr unsigned int max_branching_factor = 8;

  struct CaseResult
  {
    bool pass;
    bool skip;
    std::string message;
  };

  struct Node
  {
    Node()
    {
      for (unsigned int i = 0; i < max_branching_factor; ++i)
        children[i] = nullptr;
    }

    virtual ~Node() = default;
    Node* children[max_branching_factor];
  };

  static CaseResult passResult(const char* msg)
  {
    return { true, false, msg };
  }

  static CaseResult failResult(const char* msg)
  {
    return { false, false, msg };
  }

  static bool buildProgress(void* /*userPtr*/, double /*f*/)
  {
    return true;
  }

  static void* createNode(RTCThreadLocalAllocator alloc, unsigned int childCount, void* /*userPtr*/)
  {
    assert(childCount <= max_branching_factor);
    if (childCount > max_branching_factor)
      return nullptr;

    Node* node = (Node*)rtcThreadLocalAlloc(alloc, sizeof(Node), 16);
    new (node) Node();
    return node;
  }

  static void setNodeChildren(void* nodePtr, void** children, unsigned int childCount, void* /*userPtr*/)
  {
    assert(childCount <= max_branching_factor);
    if (childCount > max_branching_factor)
      return;

    Node* node = (Node*)nodePtr;
    for (unsigned int i = 0; i < childCount; ++i)
      node->children[i] = (Node*)children[i];
  }

  static void setNodeBounds(void* /*nodePtr*/, const RTCBounds** /*bounds*/, unsigned int childCount, void* /*userPtr*/)
  {
    assert(childCount <= max_branching_factor);
  }

  static void* createLeaf(RTCThreadLocalAllocator alloc,
                          const RTCBuildPrimitive* /*prims*/,
                          size_t /*primCount*/,
                          void* /*userPtr*/)
  {
    Node* node = (Node*)rtcThreadLocalAlloc(alloc, sizeof(Node), 16);
    new (node) Node();
    return node;
  }

  static std::vector<RTCBuildPrimitive> makeGridPrimitives(size_t primitiveCount)
  {
    std::vector<RTCBuildPrimitive> prims(primitiveCount);
    for (size_t i = 0; i < primitiveCount; ++i)
    {
      const float x = float(i % 32);
      const float y = float((i / 32) % 32);

      RTCBuildPrimitive& p = prims[i];
      p = {};
      p.lower_x = x * 2.0f;
      p.lower_y = y * 2.0f;
      p.lower_z = 0.0f;
      p.upper_x = p.lower_x + 0.5f;
      p.upper_y = p.lower_y + 0.5f;
      p.upper_z = 0.5f;
      p.geomID = 0;
      p.primID = (unsigned int)i;
    }
    return prims;
  }

  static bool morton_builder_rejects_oversized_branching_factor(RTCDevice device, unsigned int maxBranchingFactor)
  {
    RTCBVH bvh = rtcNewBVH(device);
    if (!bvh)
      return false;

    std::vector<RTCBuildPrimitive> prims = makeGridPrimitives(1024);

    RTCBuildArguments args = rtcDefaultBuildArguments();
    args.byteSize = sizeof(args);
    args.buildQuality = RTC_BUILD_QUALITY_LOW;
    args.maxBranchingFactor = maxBranchingFactor;
    args.maxDepth = 1024;
    args.minLeafSize = 1;
    args.maxLeafSize = 1;
    args.bvh = bvh;
    args.primitives = prims.data();
    args.primitiveCount = prims.size();
    args.primitiveArrayCapacity = prims.size();
    args.createNode = createNode;
    args.setNodeChildren = setNodeChildren;
    args.setNodeBounds = setNodeBounds;
    args.createLeaf = createLeaf;
    args.buildProgress = buildProgress;

    rtcGetDeviceError(device);
    void* root = rtcBuildBVH(&args);
    const RTCError error = rtcGetDeviceError(device);

    rtcReleaseBVH(bvh);
    return root == nullptr && error == RTC_ERROR_INVALID_ARGUMENT;
  }

  static CaseResult morton_builder_clamp(RTCDevice device)
  {
    if (!morton_builder_rejects_oversized_branching_factor(device, 64))
      return failResult("maxBranchingFactor=64 was not rejected");

    if (!morton_builder_rejects_oversized_branching_factor(device, std::numeric_limits<unsigned int>::max()))
      return failResult("maxBranchingFactor=UINT_MAX was not rejected");

    return passResult("oversized maxBranchingFactor values are rejected");
  }

  struct TestCase
  {
    const char* name;
    CaseResult (*fn)(RTCDevice);
  };
}

int main()
{
  RTCDevice device = rtcNewDevice(nullptr);
  if (!device)
  {
    std::printf("FAIL create_device\n");
    return 1;
  }

  const TestCase tests[] = {
    { "Morton-builder-clamp", morton_builder_clamp }
  };

  int failed = 0;
  for (const TestCase& tc : tests)
  {
    const CaseResult result = tc.fn(device);
    if (result.pass)
      std::printf("PASS %s: %s\n", tc.name, result.message.c_str());
    else
    {
      ++failed;
      std::printf("FAIL %s: %s\n", tc.name, result.message.c_str());
    }
  }

  rtcReleaseDevice(device);

  std::printf("SUMMARY total=%u passed=%u failed=%d\n",
              (unsigned)(sizeof(tests) / sizeof(tests[0])),
              (unsigned)(sizeof(tests) / sizeof(tests[0])) - (unsigned)failed,
              failed);
  return failed == 0 ? 0 : 1;
}
