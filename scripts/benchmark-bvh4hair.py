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
hair_builder_modes = [ 
    ('tri_accel=bvh4.triangle4  --tessellate-hair 0 4', 'bvh4.triangle4.P0aO'),
    ('tri_accel=bvh4.triangle4  --tessellate-hair 1 4', 'bvh4.triangle4.P1aO'),
    ('tri_accel=bvh4.triangle4  --tessellate-hair 2 4', 'bvh4.triangle4.P2aO'),
    ('tri_accel=bvh4.triangle4  --tessellate-hair 3 4', 'bvh4.triangle4.P3aO'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aO', 'bvh4hair.bezier1i.P0aO'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P1aO', 'bvh4hair.bezier1i.P1aO'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P2aO', 'bvh4hair.bezier1i.P2aO'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P3aO', 'bvh4hair.bezier1i.P3aO'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuO', 'bvh4hair.bezier1i.P0aOuO'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuST', 'bvh4hair.bezier1i.P0aOuOuST'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOaSP,hair_builder_replication_factor=1', 'bvh4hair.bezier1i.P0aOaSP.R1'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOaSP,hair_builder_replication_factor=3', 'bvh4hair.bezier1i.P0aOaSP.R3'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOaSP,hair_builder_replication_factor=7', 'bvh4hair.bezier1i.P0aOaSP.R7'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuSTaSPuSP,hair_builder_replication_factor=1', 'bvh4hair.bezier1i.P0aOuOuSTaSPuSP.R1'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuSTaSPuSP,hair_builder_replication_factor=3', 'bvh4hair.bezier1i.P0aOuOuSTaSPuSP.R3'),
    ('hair_accel=bvh4hair.bezier1i,hair_builder_mode=P0aOuOuSTaSPuSP,hair_builder_replication_factor=7', 'bvh4hair.bezier1i.P0aOuOuSTaSPuSP.R7')
    ];

def name(model,mode):
  return os.path.splitext(os.path.basename(model))[0] + '_' + mode[1]
  
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
    command += ' -i none'  # disable triangle geometry
    command += ' -size 1024 1024 -frames 4 32'
    command += ' -o ' + imgFile + ' > ' + logFile
    os.system(command)

def renderLoop():
    for mode in hair_builder_modes:
      print(name(model,mode))
      render(mode)

########################## data extraction ##########################

sah    = {}
memory = {}
fps   = {}

def extract(mode):
  base = name(model,mode)
  logFileName = statDir + dash + base + '.log'
  sah   [base] = 0
  memory[base] = 0
  fps  [base] = 0
  try:
    logFile = open(logFileName, 'r')
    for line in logFile:
      if line.count('BENCHMARK_HAIR_ACCEL ') == 1:
        numbers = map(float, line[21:].split(" "))
        sah   [base] += numbers[0]
        memory[base] += numbers[1]
      if line.count('BENCHMARK_TRIANGLE_ACCEL ') == 1:
        numbers = map(float, line[25:].split(" "))
        sah   [base] += numbers[0]
        memory[base] += numbers[1]
      if line.count('BENCHMARK_RENDER ') == 1:
        numbers = map(float, line[17:].split(" "))
        fps[base] += numbers[0]
  except IOError :
    print('cannot open ' + logFileName)

# Extract all data
def extractLoop():
  for mode in hair_builder_modes:
    extract(mode)

def printData(mode):
  base = name(model,mode)
  line  = '  ' + '{0:<35}'.format(mode[1]) + ' | '
  line += (' %#6.1f' %  sah[base])
  line += ('   %#6.1f MB' %  (1E-6*memory[base]))
  line += ('  %#6.1f' %  fps[base])
  print(line)

def printDataLoop():
  tableWidth = 35 + 32

  print('')
  
  line  = '  ' + '{0:<35}'.format(statDir) + ' |     SAH      Memory     Fps'
  print(line)

  line = ''
  while (len(line) < tableWidth): line = line + '-'
  print(line)

  for mode in hair_builder_modes:
    printData(mode)

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
