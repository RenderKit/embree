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
statDir = 'stat'
name = ''
models = []

########################## rendering ##########################

def baseName(name,model):
  return name + '_' + model

def render(name,modelname,model):
  executable = tutorial
  base = baseName(name,modelname)
  os.system('mkdir -p ' + statDir)
  logFile = statDir + dash + base + '.log'
  if not os.path.exists(logFile):
    command = executable
    command += ' -c ' + model
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
    for (modelname,model) in models:
      sys.stdout.write('  ' + '{0:<55}'.format(modelname) + ' | ')
      render(name,modelname,model)
      extract(name,modelname,'')
      printData(name,modelname)

########################## data extraction ##########################

memory = {}
buildperf = {}
sah      = {}
fps_avg  = {}
fps_sigma  = {}
fps_gain = {}
fps_davg = {}

def extract(name,modelname,prevname):
  base = baseName(name,modelname)
  prevBase = baseName(prevname,modelname)
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
    for (modelname,model) in models:
      extract(name,modelname,prevname)
    prevname = name

def printData(name,modelname):
  base = baseName(name,modelname)
  line = (' %#6.1f MB' %  (1E-6*memory[base]))
  line += (' %#6.2f M/s' %  (1E-6*buildperf[base]))
  line += (' %#6.1f ' %  sah[base])
  line += (' %#6.3f fps' %  fps_avg[base])
  line += (' +/-%#6.3f ' %  fps_sigma[base])
  line += (' (%#+3.3f, ' %  fps_davg[base])
  line += (' %#+2.1f%%)' %  fps_gain[base])
  line += '\n'
  sys.stdout.write(line)

def printHeader():
  tableWidth = 55 + 74
  line  = '  ' + '{0:<55}'.format('') + ' |     Memory      Build    SAH       Render'
  print(line)
  line = ''
  while (len(line) < tableWidth): line = line + '-'
  print(line)

def printDataLoop():
  print('')
  printHeader()
  for (modelname,model) in models:
    print(modelname)
    for name in names:
      sys.stdout.write('  ' + '{0:<55}'.format(name) + ' | ')
      printData(name,modelname)
  if len(models) > 1:
    print('average')
    for name in names:
      sys.stdout.write('  ' + '{0:<55}'.format(name) + ' | ')
      printData(name,'average')

  print('')

########################## command line parsing ##########################

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' run models name tutorialXX args\n')
  sys.stderr.write('       ' + sys.argv[0] + ' print models name1 name2 ...\n')
  sys.exit(1)

def readModelsFile(models_file):
  global models
  path, basename = os.path.split(models_file)
  with open(models_file, 'r') as f:
    lines = f.readlines()
    for line in lines:
      line = line.strip('\n')
      if line == "": continue;
      if line.startswith("#"): continue;
      (name,args) = line.split(' ',1);
      args2 = os.path.join(path,args.lstrip(' '))
      models += [(name,args2)]

if len(sys.argv) < 2:
  printUsage()
  sys.exit(1)

if sys.argv[1] == 'run':
  if len(sys.argv) < 4:
    printUsage()
    sys.exit(1)
  models_file = sys.argv[2]
  readModelsFile(models_file)
  name = sys.argv[3]
  tutorial = sys.argv[4]
  args = sys.argv[5:]
  renderLoop()
  sys.exit(1)

if sys.argv[1] == 'print':
  if len(sys.argv) < 5:
    printUsage()
    sys.exit(1)
  models_file = sys.argv[2]
  readModelsFile(models_file)
  names = sys.argv[3:]
  extractLoop()
  printDataLoop()
  sys.exit(1)

printUsage()
sys.exit(1)
