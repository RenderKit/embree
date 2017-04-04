#!/usr/bin/python

## ======================================================================== ##
## Copyright 2009-2012 Intel Corporation                                    ##
##                                                                          ##
## Licensed under the Apache License, Version 2.0 (the "License");          ##
## you may not use this file except in compliance with the License.         ##
## You may obtain a copy of the License at                                  ##
##                                                                          ##
##     http://www.apache.org/licenses/LICENSE-2.0                           ##
##                                                                          ##
## Unless required by applicable law or agreed to in writing, software      ##
## distributed under the License is distributed on an "AS IS" BASIS,        ##
## WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. ##
## See the License for the specific language governing permissions and      ##
## limitations under the License.                                           ##
## ======================================================================== ##

import sys
import os
import re
import subprocess

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' libembree.so\n')
  sys.exit(1)

if len(sys.argv) < 2:
  printUsage()
  sys.exit(1)

(cout,sterr) = subprocess.Popen(['nm','-a','--demangle','--print-size','--size-sort','--reverse-sort','-t','d',sys.argv[1] ], stdout=subprocess.PIPE).communicate()
symbols = cout.split('\n') 

def parse_line(s):
  try:
    i0=s.find(' ')
    pos=int(s[0:i0]);
    i1=s.find(' ',i0+1)
    bytes=int(s[i0+1:i1])
    i2=s.find(' ',i1+1)
    ty=s[i1+1:i2]
    sym=s[i2+1:]
    return[bytes,sym]
  except ValueError:
    return []    


symbols = map (parse_line, symbols)
symbols=filter(lambda x: len(x) == 2,symbols)

c=0
def count_feature(f,symbols):
  global c

  c=0
  def count(l):
    global c
    if (l[1].find(f) == -1):
      return True
    c = c+l[0]
    return False
    
  symbols = filter(count,symbols)
  return (c,symbols)

def count_feature2(f,components,symbols):
  r=map(lambda x: count_feature(f,x),symbols)
  b=map(lambda (x,y): x,r)
  l=map(lambda (x,y): y,r)
  components.append([f,b])
  return (components,l)

def split_list(l,s):
  a=filter(lambda x: x[1].find(s) != -1,l)
  b=filter(lambda x: x[1].find(s) == -1,l)
  return (a,b)

(symbols_avx512skx, symbols) = split_list(symbols,"::avx512skx::")
(symbols_avx512knl, symbols) = split_list(symbols,"::avx512knl::")
(symbols_avx2, symbols) = split_list(symbols,"::avx2::")
(symbols_avx, symbols) = split_list(symbols,"::avx::")
(symbols_sse42, symbols) = split_list(symbols,"::sse42::")
(symbols_sse2, symbols) = split_list(symbols,"::sse2::")
isa_symbols = (symbols,symbols_sse2,symbols_sse42,symbols_avx,symbols_avx2,symbols_avx512knl,symbols_avx512skx)

components=[]

component_names=[
  "::BVHNIntersector1",
  "::BVHNIntersectorKSingle",
  "::BVHNIntersectorKHybrid",
  "::BVHNIntersectorStream",
  "::BVHNHairMBBuilderSAH",
  "::BVHBuilderHair",
  "::BVHNHairBuilderSAH",
  "::BVHNMeshBuilderMorton",
  "::BVHNBuilderInstancing",
  "::BVHNBuilderTwoLevel",
  "::BVHNBuilderMSMBlurSAH",
  "::BVHNBuilderSAH",
  "::BVHNBuilderFastSpatialSAH",
  "::BVHNSubdivPatch1EagerBuilderSAH",
  "::BVHNSubdivPatch1CachedBuilderSAH",
  "::BVHBuilderMorton",
  "::createPrimRefArray",
  "::createBezierRefArrayMBlur",
  "::GeneralBVHBuilder",
  "::HeuristicArrayBinningSAH",
  "::HeuristicArraySpatialSAH",
  "::UnalignedHeuristicArrayBinningSAHOld",
  "::UnalignedHeuristicArrayBinningSAH",
  "::HeuristicStrandSplit",
  "::PatchEvalSimd",
  "::PatchEvalGrid",
  "::PatchEval",
  "::patchEval",
  "::evalGridBounds",
  "::evalGrid",
  "::FeatureAdaptiveEvalGrid",
  "::FeatureAdaptiveEval",
  "::intersect_bezier_recursive_jacobian",
  "::patchNormal",
  "::BVHNRotate",
  "tbb::interface9::internal::start_for",
  "::RayStream",
  ""
]
for n in component_names:
  (components,isa_symbols) = count_feature2(n,components,isa_symbols)

for c in components:
  print c

for sym in isa_symbols[1]:
  print sym

