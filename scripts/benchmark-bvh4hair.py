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
  'aO',
  'aOuO',
  'aOuOuST',
  'aOuOuSTaSP',
  'aOuOuSTaSPuSP',
  'aOuOuSTaSD',
  'aOuOuSTaSDuSD',
  'aOuOuSTaSPuSPaSDuSD',
  'P3aO',
  'P3aOuO',
  'P3aOuOuST'
  ];

########################## compiling ##########################

def compile():
    command = 'mkdir -p build; cd build; cmake ..; make clean; make -j 8';
    os.system(command)

########################## rendering ##########################

def render(mode):
  #executable = 'build' + '/' + 'tutorial07'
  executable = './tutorial07'
  base = os.path.basename(model) + '_' + mode
  os.system('mkdir -p ' + statDir)
  logFile = statDir + dash + base + '.log'
  imgFile = statDir + dash + base + '.tga'
  if not os.path.exists(logFile):
    command = executable
    command += ' -rtcore verbose=2,benchmark=1,hairaccel=bvh4hair.bezier1i,hairaccelmode=' + mode
    if model != 'none':
      command += ' -c ' + model
    command += ' -size 1024 1024 -frames 4 32'
    command += ' -o ' + imgFile + ' > ' + logFile
    os.system(command)

def renderLoop():
    for mode in hair_builder_modes:
      print(mode)
      render(mode)

########################## data extraction ##########################

sah    = {}
memory = {}
fps   = {}

def extract(mode):
  base = os.path.basename(model) + '_' + mode
  logFileName = statDir + dash + base + '.log'
  sah   [base] = 0
  memory[base] = 0
  fps  [base] = 0
  try:
    logFile = open(logFileName, 'r')
    for line in logFile:
      if line.count('BENCHMARK_HAIR_ACCEL ') == 1:
        numbers = map(float, line[21:].split(" "))
        sah   [base] = numbers[0]
        memory[base] = numbers[1]
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
  base = os.path.basename(model) + '_' + mode
  line  = '  ' + '{0:<27}'.format(mode) + ' | '
  line += (' %#6.1f' %  sah[base])
  line += ('   %#6.1f MB' %  (1E-6*memory[base]))
  line += ('  %#6.1f' %  fps[base])
  print(line)

def printDataLoop():
  tableWidth = 33 + 28

  print('')
  
  line  = '  ' + '{0:<27}'.format('Mode') + ' |     SAH      Memory     Fps'
  print(line)

  line = ''
  while (len(line) < tableWidth): line = line + '-'
  print(line)

  for mode in hair_builder_modes:
    printData(mode)

  print('')

########################## command line parsing ##########################

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' measure <model> <statDir>\n')
  sys.stderr.write('       ' + sys.argv[0] + ' print   <model> <statDir>\n')
  sys.exit(1)

if len(sys.argv) != 3 and len(sys.argv) != 4:
  printUsage()
  sys.exit(1)

model = 'none';  
if len(sys.argv) >= 3:
  model = sys.argv[2]

statDir = 'stat';  
if len(sys.argv) >= 4:
  statDir = sys.argv[3]

if sys.argv[1] == 'measure':
  renderLoop()
  extractLoop()
  printDataLoop()
  sys.exit(1)

if sys.argv[1] == 'print':
  extractLoop()
  printDataLoop()
  sys.exit(1)

printUsage()
sys.exit(1)
