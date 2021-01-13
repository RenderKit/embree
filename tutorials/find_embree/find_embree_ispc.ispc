// Copyright 2009-2021 Intel Corporation
// SPDX-License-Identifier: Apache-2.0

#include <embree3/rtcore.isph>

export void ispcEntry()
{
  /* create new Embree device */
  RTCDevice device = rtcNewDevice(NULL);

  /* delete Embree device again */
  rtcReleaseDevice(device);
}
