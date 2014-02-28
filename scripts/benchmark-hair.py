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
  'aOB',
  'aOB-uOB',
  'aOB-uOB-uST',
  'aOB-uOB-uST-aSP',
  'aOB-uOB-uST-aSP-uSP',
  'aOB-uOB-uST-aSD',
  'aOB-uOB-uST-aSD-uSD',
  'aOB-uOB-uST-aSP-uSP-aSD-uSD',
  'P3-aOB',
  'P3-aOB-uOB'
  'P3-aOB-uOB-uST'
  ];

########################## compiling ##########################

def compile():
    command = 'mkdir build; cd build; cmake ..; make clean; make -j 8';
    os.system(command)

########################## rendering ##########################

def render(mode):
  #executable = 'build' + '/' + 'tutorial07'
  executable = './tutorial07' 
  base = mode
  logFile = statDir + dash + base + '.log'
  imgFile = statDir + dash + base + '.tga'
  if not os.path.exists(logFile):
    command = executable
    command += ' -rtcore verbose=2,benchmark=1,hairaccel=bvh4hair.bezier1,hairaccelmode=' + mode
    #command += ' -c ' + model
    command += ' -frames 16'
    command += ' -o ' + imgFile + ' > ' + logFile
    os.system(command)

def renderLoop():
    for mode in hair_builder_modes:
      print(mode)
      render(mode)

########################## data extraction ##########################

sah    = {}
memory = {}
mrps   = {}

def extract(mode):
  base = mode
  logFileName = statDir + dash + base + '.log'
  sah   [base] = 0
  memory[base] = 0
  mrps  [base] = 0
  try:
    logFile = open(logFileName, 'r')
    for line in logFile:
      if line.count('BENCHMARK_HAIR_ACCEL ') == 1:
        numbers = map(float, line[21:].split(" "))
        sah   [base] = numbers[0]
        memory[base] = numbers[1]
      if line.count('BENCHMARK_RENDER ') == 1:
        numbers = map(float, line[17:].split(" "))
        mrps[base] += numbers[0]
  except IOError :
    print('cannot open ' + logFileName)

# Extract all data
def extractLoop():
  for mode in hair_builder_modes:
    extract(mode)

def printData(mode):
  base = mode
  line  = '  ' + '{0:<27}'.format(mode) + ' | '
  line += (' %#6.1f' %  sah[base])
  line += ('   %#6.1f' %  memory[base])
  line += ('  %#6.1f' %  mrps[base])
  print(line)
 
def printDataLoop():
  tableWidth = 33 + 24
  line = ''
  while (len(line) < tableWidth): line = line + '='
  print(line)
  
  line = ''
  while (len(line) < 30): line = line + ' '
  line += '|     sah   memory    mrps'
  print(line)

  for mode in hair_builder_modes:
    printData(mode)

  line = '';
  while (len(line) < tableWidth): line = line + "="
  print(line + '\n\n')

########################## command line parsing ##########################

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' <statDir> <model>\n')
  sys.stderr.write('       ' + sys.argv[0] + ' <statDir>\n')
  sys.exit(1)

if len(sys.argv) == 2:
  statDir = sys.argv[1]
  extractLoop()
  printDataLoop()
  sys.exit(1)
    
if len(sys.argv) == 3:
  statDir = sys.argv[1]
  model   = sys.argv[2]
  #compile()
  renderLoop()
  extractLoop()
  printDataLoop()
  sys.exit(1)

printUsage()
sys.exit(1)
