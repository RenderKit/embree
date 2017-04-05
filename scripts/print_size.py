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

def count_feature2(f,symbols):
  r=map(lambda x: count_feature(f,x),symbols)
  b=map(lambda (x,y): x,r)
  l=map(lambda (x,y): y,r)
  return (b,l)

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

component_names=[
  ("Intersectors",
   ["::BVHNIntersector1",
    "::BVHNIntersectorKSingle",
    "::BVHNIntersectorKHybrid",
    "::BVHNIntersectorStream",
    "::RayStream"]),
  ("Builders",
   ["::BVHNRotate",
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
    "::HeuristicStrandSplit"]),
   ("Subdiv",
    ["::PatchEvalSimd",
     "::PatchEvalGrid",
     "::PatchEval",
     "::patchEval",
     "::evalGridBounds",
     "::evalGrid",
     "::FeatureAdaptiveEvalGrid",
     "::FeatureAdaptiveEval",
     "::patchNormal"]),
   ("Other",
    ["::intersect_bezier_recursive_jacobian",
    "tbb::interface9::internal::start_for",
    ""])
 ]

def eval_component(name):
  global isa_symbols
  if type(name) is tuple: 
    return eval_component_group(name)
  else: 
    (component,isa_symbols) = count_feature2(name,isa_symbols)
    if name == "": return ["remaining",component]
    else: return [name,component]
def eval_component_group((name,names)):
  return (name,eval_components(names))
def eval_components(names):
  return map(eval_component,names)

components = eval_components(component_names)

def add7((a0,a1,a2,a3,a4,a5,a6),(b0,b1,b2,b3,b4,b5,b6)):
  return (a0+b0,a1+b1,a2+b2,a3+b3,a4+b4,a5+b5,a6+b6)

def print_header():
   sys.stdout.write(' ' + '{0:<40}'.format("Component"))
   sys.stdout.write('        NONE        SSE2      SSE4.2         AVX        AVX2   AVX512knl   AVX512skx         SUM\n')

def sum_component(c):
  if type(c) is tuple:
    return sum_component_group(c)
  else:
    return c[1]
def sum_component_group((name,components)):
  return sum_components(components)
def sum_components(components):
  sum=(0,0,0,0,0,0,0)
  for c in components:
    sum = add7(sum,sum_component(c))
  return sum

total_by_isa=sum_components(components)
total=0
for x in total_by_isa:
  total = total + x
if total == 0:
  total=1

def print_component(c):
  if type(c) is tuple:
    print_component_group(c)
  else:
    sys.stdout.write(' ' + '{0:<40}'.format(c[0]))
    sum=0;
    for s in c[1]:
      sys.stdout.write((' %#8.3f MB' %  (1E-6*s)))  
      sum = sum + s
    sys.stdout.write((' %#8.3f MB' %  (1E-6*sum)))
    sys.stdout.write((' %#7.2f %%' %  (100.0*sum/total)))
    sys.stdout.write('\n')
def print_component_group((name,components)):
  sum=sum_components(components)
  sys.stdout.write('\n')
  print_component([name,sum])
  print_components(components)
def print_components(components):
  for c in components:
    print_component(c)

print_header()
print_components(components)
sys.stdout.write('\n')
print_component(["SUM",total_by_isa])

#for sym in isa_symbols[1]:
#  print sym

