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

# Embree Test Scripts
# ===================

# Windows
# -------

# Prerequisites:
#   Install Python 3.2+
#   Install Visual Studio 2010 or 2008
#   Install Intel C++ Compiler
#   Check out Embree into <embree_dir>
#   Download Embree test scenes from shared drive /Repo/ModelRepository/Embree/*.tgz and unpack in <model_dir>

# Instructions:
#   Open the "Visual Studio x64 Cross Tools Command Prompt (2010)"
#   cd <embree_dir>
#   mkdir TEST
#   <python_dir>\python.exe <embree_dir>\scripts\run.py compile windows
#   <python_dir>\python.exe <embree_dir>\scripts\run.py render windows <model_dir> TEST
#   <python_dir>\python.exe <embree_dir>\scripts\run.py extract windows TEST
#   Check the generated output for "!" (Warning or worse), "Crash", "Error" and unusually low performance.

# Linux and OS X
# --------------

# Prerequisites:
#   Install Python 2.6+
#   Install Intel C++ Compiler
#   Check out Embree into <embree_dir>
#   Download Embree test scenes from shared drive /Repo/ModelRepository/Embree/*.tgz and unpack in <model_dir>

# Instructions:
#   Open a shell
#   cd <embree_dir>
#   mkdir TEST
#   ./scripts/run.py compile unix
#   ./scripts/run.py render unix <model_dir> TEST
#   ./scripts/run.py extract unix TEST
#   Check the generated output for red text (Warning or worse), "Crash", "Error" and unusually low performance.

import sys
import os
import re

dash = '/'

########################## configuration ##########################

oss = ['windows', 'unix']

compilers_win = ['V100']
#compilers_win = ['ICC']
#compilers_win  = ['V100', 'ICC']
compilers_unix = ['ICC']
#compilers_unix = ['GCC', 'ICC']
compilers      = []

#builds = ['Debug']
builds = ['Release']
#builds = ['Release', 'Debug']

platforms_win  = ['win32']
#platforms_win  = ['x64']
#platforms_win  = ['win32', 'x64']
platforms_unix = ['x64']
platforms      = []

#isas_win  = ['SSE4.2']
#isas_unix = ['SSSE3']
isas_unix = ['AVX']
#isas_unix = ['SSE4.2']
#isas_unix = ['SSSE3', 'SSE4.2'] #, 'AVX']
isas      = []

accels0 = [('singleray','bvh2.triangle4','default','objectsplit'), 
           ('singleray','bvh2.triangle4','default','spatialsplit'),
           ('singleray','bvh4.triangle4','default','objectsplit'),
           ('singleray','bvh4.triangle4','default','spatialsplit'),
           ('singleray','bvh4mb.triangle4i','default','objectsplit')
           ]

accels1 = [('singleray','bvh4.triangle1','default','objectsplit'),
           ('singleray','bvh4.triangle4','default','objectsplit'),
#           ('singleray','bvh4.triangle8','default','objectsplit')
           ]

accels2 = [('singleray','bvh4.triangle1i','default.moeller','objectsplit'),
           ('singleray','bvh4.triangle1i','default.pluecker','objectsplit'),
           ('singleray','bvh4.triangle1v','default.moeller','objectsplit'),
           ('singleray','bvh4.triangle1v','default.pluecker','objectsplit')]

accels3 = [('singleray','bvh4.triangle4i','default.moeller','objectsplit'),
           ('singleray','bvh4.triangle4i','default.pluecker','objectsplit'),
           ('singleray','bvh4.triangle4v','default.moeller','objectsplit'),
           ('singleray','bvh4.triangle4v','default.pluecker','objectsplit')]

accels4 = [
#           ('ispc','default','default','default'),
           ('ispc_sse','default','default','default'),
#           ('ispc_avx','default','default','default'),
           ('singleray_knc','default','default','default'),
           ('ispc_knc','default','default','default'),
           ]

accels5 = [('singleray','bvh4.triangle4','default','objectsplit'),
           ('singleray','bvh4.triangle4','default','spatialsplit'),
           ('ispc_avx','default','default','objectsplit'),
           ('ispc_avx','default','default','spatialsplit'),
           ]

accels = accels5
#accels = accels0 + accels1 + accels2 + accels3 + accels4

#scenes = ['conference']
scenes = ['conference', 'crown', 'headlight', 'nightgown', 'powerplant']
#scenes = ['conference', 'courtyard', 'crown', 'e87', 'e89', 'e89_engine', 'headlight', 'loftcube', 'nightgown', 'powerplant', 'stanford', 'xyz_dragon']

modelDir  = ''
testDir = ''

########################## compiling ##########################

def compile(OS,compiler,platform,isa,build):
  if OS == 'windows':
    command =  'msbuild embree_ispc.sln' + ' '
    command += '/p:Configuration=' + build + ';'
    command += 'Platform=' + platform + ';'
    command += 'PlatformToolset=';
#   if (compiler == 'ICC'): command += '"Intel C++ Compiler 12.1" '
    if (compiler == 'ICC'): command += '"Intel C++ Compiler XE 12.1" '
    elif (compiler == 'V100'): command += 'v100 '
    else: 
      sys.stderr.write('unkown compiler: ' + compiler + '\n')
      sys.exit(1)
    if (isa != 'SSE4.2'):
      sys.stderr.write('unkown ISA: ' + isa + '\n')
      sys.exit(1)
    command += '/t:rebuild /verbosity:n'
    print(command);
    os.system(command)
  else:
    command = 'mkdir build; cd build' + '; cmake '
    if (compiler == 'ICC'): command += '-D COMPILER=ICC '
    elif (compiler == 'GCC'): command += '-D COMPILER=GCC '
    else:
      sys.stderr.write('unkown compiler: ' + compiler + '\n')
      sys.exit(1)
    if (platform != 'x64'):
      sys.stderr.write('unkown platform: ' + platform + '\n')
      sys.exit(1)
    command += ' -D BUILD_ISPC_DEVICE_SSE=ON';
    command += ' -D BUILD_ISPC_DEVICE_AVX=ON';
    command += ' -D BUILD_ISPC_DEVICE_KNC=OFF';
    command += ' -D BUILD_SINGLERAY_DEVICE=ON';
    command += ' -D BUILD_SINGLERAY_DEVICE_KNC=OFF';
    command += ' -D BUILD_TUTORIALS_SSE=ON';
    command += ' -D BUILD_TUTORIALS_AVX=ON';
    command += ' -D BUILD_TUTORIALS_KNC=OFF';
    command += ' -D COMPILER_TARGET=' + isa + ' -D CMAKE_BUILD_TYPE=' + build + ' ..; make -j 8 ';
    os.system(command)

def compileLoop(OS):
    for compiler in compilers:
      for platform in platforms:
        for isa in isas:
          for build in builds:
            print(OS + ' ' + compiler + ' ' + platform + ' ' + isa + ' ' + build)
            compile(OS,compiler,platform,isa,build)


########################## rendering ##########################

def configName(OS, scene, compiler, platform, isa, build, device, accel, traverser, builder):
  return OS + '_' + compiler + '_' + platform + '_' + build + '_' + isa + '_' + scene + '_' + device + '_' + accel + '_' + traverser + '_' + builder

def render(OS, scene, compiler, platform, isa, build, device, accel, traverser, builder):
  if OS == 'windows': executable = platform + '\\' + build + '\\' + 'renderer.exe'
  else:               executable = 'build' + '/' + 'renderer' 
  base = configName(OS, scene, compiler, platform, isa, build, device, accel, traverser, builder)
  logFile = testDir + dash + base + '.log'
  if OS == 'windows': imageFile = testDir + dash + base + '.tga'
  else:               imageFile = testDir + dash + base + '.jpg'
  if not os.path.exists(logFile):
    command = executable
    command += ' -verbose -device ' + device
    command += ' -c ' + modelDir + dash + scene + dash + scene + '_test.ecs'
    command += ' -spp 16 '
    command += ' -accel ' + accel
    command += ' -traverser ' + traverser
    command += ' -builder ' + builder
    command += ' -o ' + imageFile + ' > ' + logFile
    os.system(command)

def renderLoop(OS):
    for compiler in compilers:
      for platform in platforms:
        for isa in isas:
          for build in builds:
            compile(OS,compiler,platform,isa,build)
            for scene in scenes:
              for accel in accels:    
                 print(compiler + ' ' + platform + ' ' + isa + ' ' + build + ' ' + scene + ' ' + accel[0] + ' '  + accel[1] + ' ' + accel[2] + ' ' + accel[3])
                 render(OS, scene, compiler, platform, isa, build, accel[0], accel[1], accel[2], accel[3])


########################## data extraction ##########################

buildTime = {}
memory = {}
mrps = {}
num = {}
warning = {}
error = {}
crash = {}

# Extract a floating point value from a string
def extractFloat(string, prefix, suffix):
  regexp = prefix + '[+-]? *(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?' + suffix
  values = re.findall(regexp, string)
  if (len(values) != 1):
    sys.stderr.write("Error: could not extract floating point value");
    sys.exit(1)
  return float(values[0].replace(prefix, '').replace(suffix, ''))

def initAverage():
  for OS in oss:
    if OS == 'windows':
      compilers = compilers_win
      platforms = platforms_win
      isas = isas_win
    else:
      compilers = compilers_unix
      platforms = platforms_unix
      isas = isas_unix
    for compiler in compilers:
      buildTime[compiler] = 0
      memory[compiler] = 0
      mrps[compiler] = 0
      num [compiler] = 0
    for platform in platforms:
      buildTime[platform] = 0
      memory[platform] = 0
      mrps[platform] = 0
      num [platform] = 0
    for isa in isas:
      buildTime[isa] = 0
      memory[isa] = 0
      mrps[isa] = 0
      num[isa] = 0
    for compiler in compilers:
      for platform in platforms:
        for isa in isas:
          for build in builds:
            for accel in accels:
              base = configName(OS, 'average', compiler, platform, isa, build, accel[0], accel[1], accel[2], accel[3])
              buildTime[base] = 0
              memory[base] = 0
              mrps[base] = 0
              num[base] = 0
              warning[base] = False
              error[base] = False
              crash[base] = False

# Extract data for on test configuration
def extract(OS, scene, compiler, platform, isa, build, device, accel, traverser, builder):
  base = configName(OS, scene, compiler, platform, isa, build, device, accel, traverser, builder)
  avg = configName(OS, 'average', compiler, platform, isa, build, device, accel, traverser, builder)
  logFileName = testDir + dash + base + '.log'
  warning[base] = False
  error[base] = False
  crash[base] = False
  buildTime[base] = 'N/A'
  memory[base] = 'N/A'
  mrps[base] = 'N/A'
  try:
    logFile = open(logFileName, 'r')
    crash[base] = True # assume a crash until mrps data was extracted successfully
    buildTime[base] = 'Crash'
    memory[base] = 'Crash'
    mrps[base] = 'Crash'
    for line in logFile:
      line = line.replace('mrps','Mrps')
      if line.count('Error') > 0:
        error[base] = True
        buildTime[base] = 'Error'
        memory[base] = 'Error'
        mrps[base] = 'Error'
        break
      if line.count('Warning') > 0:
        warning[base] = True
      if line.count('build time') > 0:
        buildTime[base] = extractFloat(line, 'build time = ', '')
      elif line.count('size') > 0:
        memory[base] = extractFloat(line, 'size = ', '')
      elif line.count('Mrps') > 0:
        mrps[base] = extractFloat(line, '', ' Mrps')
        crash[base] = False

      if mrps[base] != 'Crash':
        buildTime[compiler] += buildTime[base]
        memory[compiler] += memory[base]
        mrps[compiler] += mrps[base]
        num[compiler] += 1

        buildTime[platform] += buildTime[base]
        memory[platform] += memory[base]
        mrps[platform] += mrps[base]
        num[platform] += 1

        buildTime[isa] += buildTime[base]
        memory[isa] += memory[base]
        mrps[isa] += mrps[base]
        num[isa] += 1

        buildTime[avg] += buildTime[base]
        memory[avg] += memory[base]
        mrps[avg] += mrps[base]
        num[avg] += 1

  except IOError :
    print('cannot open ' + logFileName)

# Extract all data
def extractLoop():
  initAverage()
  for OS in oss:
    if OS == 'windows':
      compilers = compilers_win
      platforms = platforms_win
      isas = isas_win
    else:
      compilers = compilers_unix
      platforms = platforms_unix
      isas = isas_unix
    for compiler in compilers:
      for platform in platforms:
        for isa in isas:
          for build in builds:
            for scene in scenes:
              for accel in accels:    
                extract(OS, scene, compiler, platform, isa, build, accel[0], accel[1], accel[2], accel[3])

def printAccelRowForScene(OS,scene,compiler,platform,isa,build,accel):
  line  = '  ' + '{0:<8}'.format(isa) + ' | '
  line += '  ' + '{0:<13}'.format(accel[0]) + ' | '
  line += '  ' + '{0:<18}'.format(accel[1]) + ' | '
  line += '  ' + '{0:<10}'.format(accel[2].replace('default.','')) + ' | '
  line += '  ' + '{0:<12}'.format(accel[3]) + ' | '
  base = configName(OS, scene, compiler, platform, isa, build, accel[0], accel[1], accel[2], accel[3])
  if (num.has_key(base)): N = num[base] 
  else: N = 1
  if type(buildTime[base]) == float: line += ('%#6i' % (buildTime[base]/N)) + ' '
  else: line += '{0:>6}'.format(buildTime[base]) + ' '
  if type(memory[base]) == float: line += ('%#7.1f' %  (memory[base]/N)) + ' '
  else: line += '{0:>7}'.format(memory[base]) + ' '
  if type(mrps[base]) == float: line += ('%#6.1f' %  (mrps[base]/N))
  else: line += '{0:>6}'.format(mrps[base])
  line += ' | '
  print(line)
 
# Print data in formatted ASCII tables
def printScene(scene):
  tableWidth = 85 + 24
  line = scene + ' '
  while (len(line) < tableWidth+8): line = line + '='
  print(line)
  
  line = ''
  while (len(line) < 84): line = line + ' '
  line += '|  build  memory   mrps |'
  print(line)

  for OS in oss:
    if OS == 'windows':
      compilers = compilers_win
      platforms = platforms_win
      isas = isas_win
    else:
      compilers = compilers_unix
      platforms = platforms_unix
      isas = isas_unix

    for platform in platforms:
     for compiler in compilers:
      for build in builds:    
          line = compiler + ' ' + platform + ' ' + build + ' '
          while (len(line) < tableWidth): line = line + '.'
          print(line)
          for isa in isas:
            for accel in accels:
              printAccelRowForScene(OS,scene,compiler,platform,isa,build,accel)

  line = '';
  while (len(line) < tableWidth): line = line + "="
  print(line + '\n\n')

########################## command line parsing ##########################

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' compile <os>\n')
  sys.stderr.write('       ' + sys.argv[0] + ' render  <os> <modelDir> <testDir>\n')
  sys.stderr.write('       ' + sys.argv[0] + ' extract <os> <testDir>\n')
  sys.exit(1)

if len(sys.argv) < 3: printUsage()
mode = sys.argv[1]
OS = sys.argv[2]

if OS == 'windows':
  dash = '\\'
  compilers = compilers_win
  platforms = platforms_win
  isas = isas_win
else:
  dash = '/'
  compilers = compilers_unix
  platforms = platforms_unix
  isas = isas_unix

if mode == 'compile':
  compileLoop(OS)
  sys.exit(1)

if mode == 'render':
  if len(sys.argv) < 5: printUsage()
  modelDir  = sys.argv[3]
  testDir = sys.argv[4]
  renderLoop(OS)
  sys.exit(1)

if mode == 'extract':
  dash = '/'
  if len(sys.argv) < 4: printUsage()
  testDir = sys.argv[3]
  if OS != 'all': oss = [OS]
  extractLoop()

  # Print scenes
  for scene in scenes:
    printScene(scene)

  printScene('average')

  print('render average by compiler............................')
  for compiler in compilers:
    average = mrps[compiler]/num[compiler]
    print('  '+'{0:<18}'.format(compiler) + ': ' + ('%#7.1f' % average) + ' Mrps')
  print('')

  print('render average by instruction set ....................')
  for isa in isas:
    average = mrps[isa]/num[isa]
    print('  '+'{0:<18}'.format(isa) + ': ' + ('%#7.1f' % average) + ' Mrps')
  print('')

  sys.exit(1)
