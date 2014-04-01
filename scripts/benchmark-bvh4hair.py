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

########################## configuration ##########################

dash = '/'
model  = ''
statDir = ''

hair_builder_modes_uncompressed = [ 
    ('tri_accel=bvh4.triangle4  --tessellate-hair 0 4', 'bvh4.triangle4.P0aO'),
#    ('tri_accel=bvh4.triangle4  --tessellate-hair 1 4', 'bvh4.triangle4.P1aO'),
#    ('tri_accel=bvh4.triangle4  --tessellate-hair 2 4', 'bvh4.triangle4.P2aO'),
    ('tri_accel=bvh4.triangle4  --tessellate-hair 3 4', 'bvh4.triangle4.P3aO'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aO', 'bvh4hair.bezier1i.P0aO'),
#    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P1aO', 'bvh4hair.bezier1i.P1aO'),
#    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P2aO', 'bvh4hair.bezier1i.P2aO'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P3aO', 'bvh4hair.bezier1i.P3aO'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuO', 'bvh4hair.bezier1i.P0aOuO'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuST', 'bvh4hair.bezier1i.P0aOuOuST'),
#    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOaSP,hair_builder_replication_factor=1', 'bvh4hair.bezier1i.P0aOaSP.R1'),
#    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOaSP,hair_builder_replication_factor=3', 'bvh4hair.bezier1i.P0aOaSP.R3'),
#    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOaSP,hair_builder_replication_factor=7', 'bvh4hair.bezier1i.P0aOaSP.R7'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuSTaSPuSP,hair_builder_replication_factor=1', 'bvh4hair.bezier1i.P0aOuOuSTaSPuSP.R1'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuSTaSPuSP,hair_builder_replication_factor=3', 'bvh4hair.bezier1i.P0aOuOuSTaSPuSP.R3'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuSTaSPuSP,hair_builder_replication_factor=7', 'bvh4hair.bezier1i.P0aOuOuSTaSPuSP.R7')
    ];

hair_builder_modes_compressed_aligned = [
#  ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aO', 'cbvh4hair.bezier1i.P0aO'),
#  ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P1aO', 'cbvh4hair.bezier1i.P1aO'),
#  ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P2aO', 'cbvh4hair.bezier1i.P2aO'),
#  ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P3aO', 'cbvh4hair.bezier1i.P3aO'),
#  ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOaSP,hair_builder_replication_factor=7', 'cbvh4hair.bezier1i.P0aOaSP.R7'),
]

hair_builder_modes_compressed_unaligned = [
  ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuST', 'cbvh4hair.bezier1i.P0aOuOuST'),
  ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuSTaSPuSP,hair_builder_replication_factor=3', 'cbvh4hair.bezier1i.P0aOuOuSTaSPuSP.R3'),
  ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuSTaSPuSP,hair_builder_replication_factor=7', 'cbvh4hair.bezier1i.P0aOuOuSTaSPuSP.R7')
]

hair_builder_modes_measure = hair_builder_modes_uncompressed
#hair_builder_modes_measure = hair_builder_modes_compressed_aligned
#hair_builder_modes_measure = hair_builder_modes_compressed_unaligned 

keep_triangles = [
  'buddha',
  'hairdragon'  
]

hair_builder_ignore = [
  'sophie_bvh4.triangle4.P1aO',
  'sophie_bvh4.triangle4.P2aO',
  'sophie_bvh4.triangle4.P3aO',
  'buddha_bvh4.triangle4.P1aO',
  'buddha_bvh4.triangle4.P2aO',
  'buddha_bvh4.triangle4.P3aO',
  'hairdragon_bvh4.triangle4.P1aO',
  'hairdragon_bvh4.triangle4.P2aO',
  'hairdragon_bvh4.triangle4.P3aO'
]

hair_builder_modes_print =  [
  'bvh4.triangle4.P0aO',
  'bvh4.triangle4.P1aO',
  'bvh4.triangle4.P2aO',
  'bvh4.triangle4.P3aO',
  '',
  'bvh4hair.bezier1i.P0aO',
  'bvh4hair.bezier1i.P1aO',
  'bvh4hair.bezier1i.P2aO',
  'bvh4hair.bezier1i.P3aO',
  'cbvh4hair.bezier1i.P3aO',
  '',
  'bvh4hair.bezier1i.P0aOaSP.R1',
  'bvh4hair.bezier1i.P0aOaSP.R3',
  'bvh4hair.bezier1i.P0aOaSP.R7',
  'cbvh4hair.bezier1i.P0aOaSP.R7',
  '',
  'bvh4hair.bezier1i.P0aOuO',
  'bvh4hair.bezier1i.P0aOuOuST',
  'cbvh4hair.bezier1i.P0aOuOuST',
  '',
  'bvh4hair.bezier1i.P0aOuOuSTaSPuSP.R1',
  'bvh4hair.bezier1i.P0aOuOuSTaSPuSP.R3',
  'bvh4hair.bezier1i.P0aOuOuSTaSPuSP.R7',
  'cbvh4hair.bezier1i.P0aOuOuSTaSPuSP.R7',
  '',
  'bvh4.triangle4.P3aO',
  'bvh4hair.bezier1i.P3aO',
  'bvh4hair.bezier1i.P0aOaSP.R7',
  'bvh4hair.bezier1i.P0aOuOuST',
  'bvh4hair.bezier1i.P0aOuOuSTaSPuSP.R7',
  '',
  'cbvh4hair.bezier1i.P3aO',
  'cbvh4hair.bezier1i.P0aOaSP.R7',
  'cbvh4hair.bezier1i.P0aOuOuST',
  'cbvh4hair.bezier1i.P0aOuOuSTaSPuSP.R7'
]

def modelname(model):
  return os.path.splitext(os.path.basename(model))[0]
def name(model,mode):
  return modelname(model) + '_' + mode[1]
def name2(model,mode):
  return modelname(model) + '_' + mode
  
########################## compiling ##########################

def compile():
    command = 'mkdir -p build; cd build; cmake ..; make clean; make -j 8';
    os.system(command)

########################## rendering ##########################

def render(mode):
  #executable = 'build' + '/' + 'tutorial07'
  executable = './tutorial07'
  base = name(model,mode)
  os.system('mkdir -p ' + statDir)
  logFile = statDir + dash + base + '.log'
  imgFile = statDir + dash + base + '.tga'
  if not os.path.exists(logFile):
    command = executable
    command += ' -rtcore verbose=2,benchmark=1,' + mode[0]
    command += ' -c ' + model
    #if not modelname(model) in keep_triangles:
    #  command += ' -i none'  # disable triangle geometry
    command += ' -size 1024 1024 -frames 4 8'
    command += ' -o ' + imgFile + ' > ' + logFile
    os.system(command)

def renderLoop():
    for mode in hair_builder_modes_measure:
      base = name(model,mode)
      print(base)
      if (base in hair_builder_ignore):
        continue;
      render(mode)

########################## data extraction ##########################

tri_sah    = {}
tri_memory = {}
hair_sah    = {}
hair_memory = {}
fps   = {}

def extract(mode):
  base = name2(model,mode)
  logFileName = statDir + dash + base + '.log'
  tri_sah   [base] = 0
  tri_memory[base] = 0
  hair_sah   [base] = 0
  hair_memory[base] = 0
  fps  [base] = 0
  try:
    logFile = open(logFileName, 'r')
    for line in logFile:
      if line.count('BENCHMARK_HAIR_ACCEL ') == 1:
        numbers = map(float, line[21:].split(" "))
        hair_sah   [base] += numbers[0]
        hair_memory[base] += numbers[1]
      if line.count('BENCHMARK_TRIANGLE_ACCEL ') == 1:
        numbers = map(float, line[25:].split(" "))
        tri_sah   [base] += numbers[0]
        tri_memory[base] += numbers[1]
      if line.count('BENCHMARK_RENDER ') == 1:
        numbers = map(float, line[17:].split(" "))
        fps[base] += numbers[0]
  except IOError :
    print('cannot open ' + logFileName)

# Extract all data
def extractLoop():
  for mode in hair_builder_modes_print:
    if mode != '': extract(mode)

def printData(mode):
  base = name2(model,mode)
  line  = '  ' + '{0:<40}'.format(mode) + ' | '
  line += (' %#6.1f' %  tri_sah[base])
  line += ('   %#6.1f MB' %  (1E-6*tri_memory[base]))
  line += (' %#6.1f' %  hair_sah[base])
  line += ('   %#6.1f MB' %  (1E-6*hair_memory[base]))
  line += ('  %#6.3f' %  fps[base])
  line += ('  %#6.3f' %  (fps[base]/(1E-9*(tri_memory[base]+hair_memory[base]+0.0001))))
  print(line)

def printDataLoop():
  tableWidth = 40 + 60

  print('')
  
  title = os.path.splitext(os.path.basename(model))[0]
  line  = '  ' + '{0:<40}'.format(title) + ' |  TriSAH   TriMemory HairSAH HairMemory     Fps  Fps/GB'
  print(line)

  line = ''
  while (len(line) < tableWidth): line = line + '-'
  print(line)

  for mode in hair_builder_modes_print:
    if mode == '': print('')
    else: printData(mode)

  print('')

########################## command line parsing ##########################

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' measure <statDir> <model0> <model1> ...\n')
  sys.stderr.write('       ' + sys.argv[0] + ' print   <statDir> <model0> <model1> ...\n')
  sys.exit(1)

if len(sys.argv) < 3:
  printUsage()
  sys.exit(1)

model = 'none' 
statDir = 'stat'

if sys.argv[1] == 'measure':
  statDir = sys.argv[2]
  for i in range(3, len(sys.argv)):
    model = sys.argv[i]
    renderLoop()
  for i in range(3, len(sys.argv)):
    model = sys.argv[i]
    extractLoop()
    printDataLoop()
  sys.exit(1)

if sys.argv[1] == 'print':
  print(os.path.splitext(os.path.basename(model))[0])
  statDir = sys.argv[2]
  for i in range(3, len(sys.argv)):
    model = sys.argv[i]
    extractLoop()
    printDataLoop()
  sys.exit(1)

printUsage()
sys.exit(1)
