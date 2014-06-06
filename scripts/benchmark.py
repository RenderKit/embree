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
  executable = tutorial
  base = baseName(name,model)
  os.system('mkdir -p ' + statDir)
  logFile = statDir + dash + base + '.log'
  if not os.path.exists(logFile):
    command = executable
    command += ' -c ' + modelDir + dash + model + dash + model + '_regression.ecs'
    for arg in args:
      command += ' ' + arg
    command += ' -size 1024 1024 -benchmark 4 8 > ' + logFile
    os.system(command)

def renderLoop():
    printHeader()
    for model in models:
      sys.stdout.write('  ' + '{0:<20}'.format(model) + ' | ')
      render(name,model)
      extract(name,model)
      printData(name,model,'')

########################## data extraction ##########################

memory = {}
buildperf = {}
sah    = {}
fps   = {}

def extract(name,model):
  base = baseName(name,model)
  logFileName = statDir + dash + base + '.log'
  memory[base] = 0
  buildperf[base] = 0
  sah   [base] = 0
  fps   [base] = 0
  try:
    logFile = open(logFileName, 'r')
    for line in logFile:
      if line.count('BENCHMARK_BUILD ') == 1:
        numbers = map(float, line[16:].split(" "))
        buildperf[base] = numbers[1]
        sah   [base] = numbers[2]
        memory[base] = numbers[3]
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

def printData(name,model,prevname):
  base = baseName(name,model)
  line = (' %#6.1f MB' %  (1E-6*memory[base]))
  line += (' %#6.1f M/s' %  (1E-6*buildperf[base]))
  line += (' %#6.1f ' %  sah[base])
  line += (' %#6.3f fps' %  fps[base])
  if (prevname != ''):
    prevbase = baseName(prevname,model)
    line += (' (%#+5.1f%%)' %  (100.0*fps[base]/fps[prevbase]-100.0))
  line += '\n'
  sys.stdout.write(line)

def printHeader():
  tableWidth = 40 + 60
  line  = '  ' + '{0:<20}'.format('') + ' |     Memory      Build    SAH      Render'
  print(line)
  line = ''
  while (len(line) < tableWidth): line = line + '-'
  print(line)

def printDataLoop():
  print('')
  printHeader()
  for model in models:
    print(model)
    prevname = ''
    for name in names:
      sys.stdout.write('  ' + '{0:<20}'.format(name) + ' | ')
      printData(name,model,prevname)
      prevname = name

  print('')

########################## command line parsing ##########################

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' run name tutorialXX args\n')
  sys.stderr.write('       ' + sys.argv[0] + ' print name1 name2 ...\n')
  sys.exit(1)

if len(sys.argv) < 3:
  printUsage()
  sys.exit(1)

if sys.argv[1] == 'run':
  name = sys.argv[2]
  tutorial = sys.argv[3]
  args = sys.argv[4:]
  renderLoop()
  sys.exit(1)

if sys.argv[1] == 'print':
  names = sys.argv[2:]
  extractLoop()
  printDataLoop()
  sys.exit(1)

printUsage()
sys.exit(1)
