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
#models = [ 'conference' ]
#models = [ 'conference', 'sponza', 'headlight', 'crown']
models = [ 'conference', 'sponza', 'headlight', 'crown', 'bentley', 'xyz_dragon', 'powerplant' ]
arg = ''
modelDir  = ''
tutorial = 'tutorial03'
statDir = 'stat'
name = ''
modelDir = '~/models/embree'

########################## rendering ##########################

def baseName(name,model):
  return name + '_' + model

def render(name,model):
  executable = './' + tutorial
  base = baseName(name,model)
  os.system('mkdir -p ' + statDir)
  logFile = statDir + dash + base + '.log'
  imgFile = statDir + dash + base + '.tga'
  if not os.path.exists(logFile):
    command = executable
    command += ' -c ' + modelDir + dash + model + dash + model + '_regression.ecs'
    command += ' -rtcore verbose=2,benchmark=1,' + arg
    command += ' -size 1024 1024 -frames 4 8'
    command += ' -o ' + imgFile + ' > ' + logFile
    print(command)
    os.system(command)

def renderLoop():
    for model in models:
      print(name + '_' + model)
      render(name,model)

########################## data extraction ##########################

tri_sah    = {}
tri_memory = {}
hair_sah    = {}
hair_memory = {}
fps   = {}

def extract(name,model):
  base = baseName(name,model)
  logFileName = statDir + dash + base + '.log'
  tri_sah   [base] = 0
  tri_memory[base] = 0
  hair_sah   [base] = 0
  hair_memory[base] = 0
  fps        [base] = 0
  try:
    logFile = open(logFileName, 'r')
    for line in logFile:
      if line.count('BENCHMARK_HAIR_ACCEL ') == 1:
        numbers = map(float, line[21:].split(" "))
        hair_sah   [base] = numbers[0]
        hair_memory[base] = numbers[1]
      if line.count('BENCHMARK_TRIANGLE_ACCEL ') == 1:
        numbers = map(float, line[25:].split(" "))
        tri_sah   [base] = numbers[0]
        tri_memory[base] = numbers[1]
      if line.count('BENCHMARK_RENDER ') == 1:
        numbers = map(float, line[17:].split(" "))
        fps[base] = numbers[0]
  except IOError :
    print('cannot open ' + logFileName)

# Extract all data
def extractLoop():
  for name in names:
    for model in models:
      extract(name,model)

def printData(name,model):
  base = baseName(name,model)
  line  = '  ' + '{0:<40}'.format(name) + ' | '
  line += (' %#6.1f' %  tri_sah[base])
  line += ('   %#6.1f MB' %  (1E-6*tri_memory[base]))
  line += (' %#6.1f' %  hair_sah[base])
  line += ('   %#6.1f MB' %  (1E-6*hair_memory[base]))
  line += ('  %#6.3f' %  fps[base])
  print(line)

def printDataLoop():
  tableWidth = 40 + 60

  print('')
  
  title = ''
  line  = '  ' + '{0:<40}'.format(title) + ' |  TriSAH   TriMemory HairSAH HairMemory     Fps'
  print(line)

  line = ''
  while (len(line) < tableWidth): line = line + '-'
  print(line)

  for model in models:
    print(model)
    for name in names:
      printData(name,model)

  print('')

########################## command line parsing ##########################

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' tutorial?? name args\n')
  sys.stderr.write('       ' + sys.argv[0] + ' print    name1 name2 ...\n')
  sys.exit(1)

if len(sys.argv) < 3:
  printUsage()
  sys.exit(1)

if sys.argv[1][0:8] == 'tutorial':
  tutorial = sys.argv[1]
  name = sys.argv[2]
  arg = sys.argv[3]
  renderLoop()
  sys.exit(1)

if sys.argv[1] == 'print':
  names = sys.argv[2:]
  extractLoop()
  printDataLoop()
  sys.exit(1)

printUsage()
sys.exit(1)
