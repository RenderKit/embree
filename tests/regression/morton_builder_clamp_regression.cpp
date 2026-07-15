// Copyright 2026 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <embree4/rtcore.h>
#include <embree4/rtcore_builder.h>

#include <cassert>
#include <cstring>
#include <iostream>
#include <limits>
#include <vector>

constexpr unsigned int max_branching_factor = 8;

struct Node
{
  Node()
  {
    for (unsigned int i = 0; i < max_branching_factor; ++i)
      children[i] = nullptr;
  }
  virtual ~Node() = default;
  Node *children[max_branching_factor];
};

static bool buildProgress(void * /*userPtr*/, double /*f*/)
{
  return true;
}

bool memoryMonitor(void * /*userPtr*/, ssize_t /*bytes*/, bool /*post*/)
{
  return true;
}

static void *createNode(RTCThreadLocalAllocator alloc, unsigned int childCount, void * /*userPtr*/)
{
  assert(childCount <= max_branching_factor);
  if (childCount > max_branching_factor)
    return nullptr;

  Node *node = (Node *)rtcThreadLocalAlloc(alloc, sizeof(Node), 16);
  new (node) Node();
  return node;
}

static void setNodeChildren(void *nodePtr, void **children, unsigned int childCount, void * /*userPtr*/)
{
  assert(childCount <= max_branching_factor);
  if (childCount > max_branching_factor)
    return;
  Node *node = (Node *)nodePtr;
  for (unsigned int i = 0; i < childCount; ++i)
    node->children[i] = (Node *)children[i];
}

static void setNodeBounds(void *nodePtr, const RTCBounds **bounds, unsigned int childCount, void * /*userPtr*/)
{
  assert(childCount <= max_branching_factor);
  /* deliberately empty in regression test */
}

static void *createLeaf(RTCThreadLocalAllocator alloc,
                        const RTCBuildPrimitive *prims,
                        size_t primCount,
                        void * /*userPtr*/)
{

  Node *node = (Node *)rtcThreadLocalAlloc(alloc, sizeof(Node), 16);
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

    RTCBuildPrimitive p{};
    p.lower_x = x * 2.0f;
    p.lower_y = y * 2.0f;
    p.lower_z = 0.0f;
    p.upper_x = p.lower_x + 0.5f;
    p.upper_y = p.lower_y + 0.5f;
    p.upper_z = 0.5f;
    p.geomID = 0;
    p.primID = (unsigned int)i;
    prims[i] = p;
  }
  return prims;
}

static bool runCase(unsigned int maxBranchingFactor)
{
  RTCDevice device = rtcNewDevice(nullptr);
  if (device == nullptr)
    return false;

  RTCBVH bvh = rtcNewBVH(device);
  if (bvh == nullptr)
  {
    rtcReleaseDevice(device);
    return false;
  }

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

  void *root = rtcBuildBVH(&args);

  rtcReleaseBVH(bvh);
  rtcReleaseDevice(device);
  return root != nullptr;
}

int main()
{
  /* In the failure case, this test should assert or result in a segfault from stack overflow. */

  bool okOversized = runCase(64);
  bool okExtreme = runCase(std::numeric_limits<unsigned int>::max());

  if (!okOversized)
    std::cerr << "Morton clamp regression failed for maxBranchingFactor=64\n";
  if (!okExtreme)
    std::cerr << "Morton clamp regression failed for maxBranchingFactor=UINT_MAX\n";

  std::cout << "Morton clamp regression test completed.\n";
  return (okOversized && okExtreme) ? 0 : 1;
}
