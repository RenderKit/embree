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
models = [ 'conference', 'sponza', 'bentley', 'xyz_dragon', 'powerplant']
#models = [ 'conference', 'sponza', 'headlight', 'crown', 'bentley', 'xyz_dragon', 'powerplant' ]
#models = [ 'sophie', 'sophie_mblur']

arg = ''
modelDir  = ''
tutorial = 'tutorial03'
statDir = 'stat'
name = ''
if sys.platform == 'win32':
  modelDir = '%HOMEPATH%/models/embree/benchmarking'
else:
  modelDir = '~/models/embree/benchmarking'

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
    command += ' -c ' + modelDir + dash + model + '_tutorial.ecs'
    for arg in args:
      command += ' ' + arg
    command += ' -rtcore verbose=2 -benchmark 4 16 > ' + logFile
    os.system(command)

def renderLoop():
    avgBase = baseName(name,'average')
    memory   [avgBase] = 0
    buildperf[avgBase] = 0
    sah      [avgBase] = 0
    fps_avg  [avgBase] = 0
    fps_sigma[avgBase] = 0
    fps_gain [avgBase] = 0
    printHeader()
    for model in models:
      sys.stdout.write('  ' + '{0:<55}'.format(model) + ' | ')
      render(name,model)
      extract(name,model,'')
      printData(name,model)

########################## data extraction ##########################

memory = {}
buildperf = {}
sah      = {}
fps_avg  = {}
fps_sigma  = {}
fps_gain = {}
fps_davg = {}

def extract(name,model,prevname):
  base = baseName(name,model)
  prevBase = baseName(prevname,model)
  avgBase = baseName(name,'average')
  logFileName = statDir + dash + base + '.log'
  memory   [base] = 0
  buildperf[base] = 0
  sah      [base] = 0
  fps_avg  [base] = 0
  fps_sigma[base] = 0
  fps_gain [base] = 0
  fps_davg [base] = 0
  try:
    logFile = open(logFileName, 'r')
    for line in logFile:
      if line.count('BENCHMARK_BUILD ') == 1:
        numbers = map(float, line[(line.index('BENCHMARK_BUILD ')+16):].split(" "))
        buildperf[base] = numbers[1]
        sah   [base] = numbers[2]
        memory[base] = numbers[3]
      if line.count('BENCHMARK_RENDER_AVG ') == 1:
        numbers = map(float, line[21:].split(" "))
        fps_avg[base] = numbers[0]
        if (prevname != ''):
          fps_gain[base] = 100.0*fps_avg[base]/fps_avg[prevBase]-100.0
          fps_davg[base] = fps_avg[base]-fps_avg[prevBase]
      if line.count('BENCHMARK_RENDER_AVG_SIGMA ') == 1:
        numbers = map(float, line[27:].split(" "))
        fps_sigma[base] = numbers[0]
  except IOError :
    print('cannot open ' + logFileName)

  memory   [avgBase] += memory   [base] / len(models)
  buildperf[avgBase] += buildperf[base] / len(models)
  sah      [avgBase] += sah      [base] / len(models)
  fps_avg  [avgBase] += fps_avg  [base] / len(models)
  fps_sigma[avgBase] += fps_sigma[base] / len(models)
  if (prevname != ''):
    fps_gain  [avgBase] += fps_gain  [base] / len(models)
    fps_davg  [avgBase] += fps_davg  [base] / len(models)

# Extract all data
def extractLoop():
  prevname = ''
  for name in names:
    avgBase = baseName(name,'average')
    memory   [avgBase] = 0
    buildperf[avgBase] = 0
    sah      [avgBase] = 0
    fps_avg  [avgBase] = 0
    fps_sigma[avgBase] = 0
    fps_gain [avgBase] = 0
    fps_davg [avgBase] = 0
    for model in models:
      extract(name,model,prevname)
    prevname = name

def printData(name,model):
  base = baseName(name,model)
  line = (' %#6.1f MB' %  (1E-6*memory[base]))
  line += (' %#6.1f M/s' %  (1E-6*buildperf[base]))
  line += (' %#6.1f ' %  sah[base])
  line += (' %#6.3f fps' %  fps_avg[base])
  line += (' +/-%#6.3f ' %  fps_sigma[base])
  line += (' (%#+3.3f, ' %  fps_davg[base])
  line += (' %#+2.1f%%)' %  fps_gain[base])
  line += '\n'
  sys.stdout.write(line)

def printHeader():
  tableWidth = 55 + 60
  line  = '  ' + '{0:<55}'.format('') + ' |     Memory      Build    SAH      Render'
  print(line)
  line = ''
  while (len(line) < tableWidth): line = line + '-'
  print(line)

def printDataLoop():
  print('')
  printHeader()
  for model in models:
    print(model)
    for name in names:
      sys.stdout.write('  ' + '{0:<55}'.format(name) + ' | ')
      printData(name,model)
  if len(models) > 1:
    print('average')
    for name in names:
      sys.stdout.write('  ' + '{0:<55}'.format(name) + ' | ')
      printData(name,'average')

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
