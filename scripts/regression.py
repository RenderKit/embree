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

# Embree Regression Test Script
# ===============================

# Windows
# -------

# Prerequisites:
#   Install Python 3.2+
#   Install Visual Studio 2010 or 2008
#   Install Intel C++ Compiler
#   Check out Embree into <embree_dir>

# Instructions:
#   Open the "Visual Studio x64 Cross Tools Command Prompt (2010)"
#   cd <embree_dir>
#   mkdir TEST
#   <python_dir>\python.exe <embree_dir>\scripts\benchmark.py render windows TEST
#   <python_dir>\python.exe <embree_dir>\scripts\benchmark.py extract windows TEST

# Linux and OS X
# --------------

# Prerequisites:
#   Install Python 2.6+
#   Install Intel C++ Compiler
#   Check out Embree into <embree_dir>

# Instructions:
#   Open a shell
#   cd <embree_dir>
#   mkdir TEST
#   ./scripts/benchmark.py render linux <model_dir> TEST
#   ./scripts/benchmark.py extract linux TEST

import sys
import os
import re

dash = '/'

########################## configuration ##########################

#compilers_win = ['V100']
#compilers_win = ['ICC']
compilers_win  = ['V100', 'V110', 'V120', 'ICC']
#compilers_unix = ['ICC']
compilers_unix = ['GCC', 'ICC']
#compilers_unix = ['GCC', 'CLANG', 'ICC']
compilers      = []

supported_configurations = [
  'V100_x64_Debug', 'V100_x64_Release',
  'V110_x64_Debug', 'V110_x64_Release', 'V110_x64_ReleaseAVX', 
  'V120_x64_Debug', 'V120_x64_Release', 'V120_x64_ReleaseAVX', 'V120_x64_ReleaseAVX2', 
  'ICC_x64_Debug', 'ICC_x64_Release', 'ICC_x64_ReleaseAVX', 'ICC_x64_ReleaseAVX2', 
  'GCC_x64_Debug', 'GCC_x64_Release', 'GCC_x64_ReleaseAVX', 'GCC_x64_ReleaseAVX2', 
  'CLANG_x64_Debug', 'CLANG_x64_Release', 'CLANG_x64_ReleaseAVX', 'CLANG_x64_ReleaseAVX2', 
  #'V120_win32_Debug',
  'V120_win32_Release', 'V120_win32_ReleaseAVX', 'V120_win32_ReleaseAVX2'
  ]

#builds_win = ['Debug']
#builds_win = ['Release']
#builds_win = ['Release', 'Debug', 'ReleaseAVX']
builds_win = ['Release', 'Debug', 'ReleaseAVX', 'ReleaseAVX2']
#builds_unix = ['Debug']
#builds_unix = ['Release']
#builds_unix = ['Release', 'Debug', 'ReleaseAVX']
builds_unix = ['Release', 'Debug', 'ReleaseAVX', 'ReleaseAVX2']
builds = []

platforms_win  = ['win32']
#platforms_win  = ['x64']
#platforms_win  = ['win32', 'x64']
platforms_unix = ['x64']
platforms      = []

models = {}
models['win32'] = [ 'conference', 'sponza', 'headlight', 'crown', 'bentley' ]
models['x64'  ] = [ 'conference', 'sponza', 'headlight', 'crown', 'bentley', 'xyz_dragon', 'powerplant' ]

modelDir  = ''
testDir = ''

def configName(OS, compiler, platform, build, tutorial, scene, flags):
  cfg = OS + '_' + compiler + '_' + platform + '_' + build
  if tutorial != '':
    cfg += '_' + tutorial
  if scene != '':
    cfg += '_' + scene
  if flags != '':
    cfg += '_' + flags
  return cfg

########################## compiling ##########################

def compile(OS,compiler,platform,build):

  base = configName(OS, compiler, platform, build, 'build', '', '')
  logFile = testDir + dash + base + '.log'

  if OS == 'windows':
  
    cfg = '/p:Configuration=' + build + ';'
    cfg += 'Platform=' + platform + ';'
    cfg += 'PlatformToolset=';
#   if (compiler == 'ICC'): cfg += '"Intel C++ Compiler 12.1" '
#   if (compiler == 'ICC'): cfg += '"Intel C++ Compiler XE 12.1" '
    if (compiler == 'ICC'): cfg += '"Intel C++ Compiler XE 14.0" '
    elif (compiler == 'V90'): cfg += 'v90 '
    elif (compiler == 'V100'): cfg += 'v100 '
    elif (compiler == 'V110'): cfg += 'v110 '
    elif (compiler == 'V120'): cfg += 'v120 '
    else: 
      sys.stderr.write('unknown compiler: ' + compiler + '\n')
      sys.exit(1)

    # compile Embree
    command =  'msbuild embree.sln' + ' /nologo ' + cfg + ' /t:rebuild /verbosity:q > ' + logFile
    return os.system(command)
  
  else:

    if (platform != 'x64'):
      sys.stderr.write('unknown platform: ' + platform + '\n')
      sys.exit(1)

    if   (compiler == 'ICC'  ): compilerOption = ' -D COMPILER=ICC'
    elif (compiler == 'GCC'  ): compilerOption = ' -D COMPILER=GCC'
    elif (compiler == 'CLANG'): compilerOption = ' -D COMPILER=CLANG'
    else:
      sys.stderr.write('unknown compiler: ' + compiler + '\n')
      sys.exit(1)

    # first compile Embree
    command = 'mkdir -p build && cd build && cmake > /dev/null'
    command += compilerOption
    command += ' -D RTCORE_RAY_MASK=OFF'
    command += ' -D RTCORE_BACKFACE_CULLING=OFF'
    command += ' -D RTCORE_INTERSECTION_FILTER=ON'
    command += ' -D RTCORE_BUFFER_STRIDE=ON'
    command += ' -D RTCORE_STAT_COUNTERS=OFF'

    if build == 'ReleaseAVX' or build == 'ReleaseAVX2':
      command += ' -D XEON_ISA=AVX'
    elif build == 'ReleaseAVX2':
      command += ' -D XEON_ISA=AVX2'
    else:
      command += ' -D XEON_ISA=SSE4.2'

    if build == 'Debug':
      command += ' -D CMAKE_BUILD_TYPE=Debug'
    else:
      command += ' -D CMAKE_BUILD_TYPE=Release'
    
    command += ' .. && make clean && make -j 8'
    command += ' &> ../' + logFile
    return os.system(command)

def compileLoop(OS):
    for compiler in compilers:
      for platform in platforms:
        for build in builds:
          sys.stdout.write(OS + ' ' + compiler + ' ' + platform + ' ' + build)
          compile(OS,compiler,platform,isas,build)

########################## rendering ##########################

def render(OS, compiler, platform, build, tutorial, scene, flags):
  sys.stdout.write("  "+tutorial)
  if scene != '': sys.stdout.write(' '+scene)
  if flags != '': sys.stdout.write(' '+flags)
  sys.stdout.flush()
  base = configName(OS, compiler, platform, build, tutorial, scene, flags)
  logFile = testDir + dash + base + '.log'
  imageFile = testDir + dash + base + '.tga'
  if os.path.exists(logFile):
    sys.stdout.write(" [skipped]\n")
  else:
    if OS == 'windows': command = platform + '\\' + build + '\\' + tutorial + ' '
    else:               command = 'build' + '/' + tutorial + ' '
    if scene != '':
      command += '-c ' + modelDir + dash + scene + dash + scene + '_regression.ecs '
    if tutorial == 'regression':
      command += '-regressions 2000 '
    if tutorial[0:8] == 'tutorial':
      command += '-rtcore verbose=2 -size 1024 1024 -o ' + imageFile
    command += ' > ' + logFile
    ret = os.system(command)
    if ret == 0: sys.stdout.write(" [passed]\n")
    else       : sys.stdout.write(" [failed]\n")

def processConfiguration(OS, compiler, platform, build, models):
  sys.stdout.write('compiling configuration ' + compiler + ' ' + platform + ' ' + build)
  sys.stdout.flush()
  ret = compile(OS,compiler,platform,build)
  if ret != 0: sys.stdout.write(" [failed]\n")
  else: 
    sys.stdout.write(" [passed]\n")
                      
    render(OS, compiler, platform, build, 'verify', '', '')
    render(OS, compiler, platform, build, 'benchmark', '', '')

    render(OS, compiler, platform, build, 'tutorial00', '', '')
    render(OS, compiler, platform, build, 'tutorial01', '', '')
    render(OS, compiler, platform, build, 'tutorial02', '', '')
    for model in models:
      render(OS, compiler, platform, build, 'tutorial03', model, 'static')
      render(OS, compiler, platform, build, 'tutorial03', model, 'dynamic')
      render(OS, compiler, platform, build, 'tutorial03', model, 'high_quality')
      render(OS, compiler, platform, build, 'tutorial03', model, 'robust')
      render(OS, compiler, platform, build, 'tutorial03', model, 'compact')

    render(OS, compiler, platform, build, 'tutorial04', '', '')
    render(OS, compiler, platform, build, 'tutorial05', '', '')
    for model in models:
      render(OS, compiler, platform, build, 'tutorial06', model, '')
    render(OS, compiler, platform, build, 'tutorial07', '', '')
			    
    render(OS, compiler, platform, build, 'tutorial00_ispc', '', '')
    render(OS, compiler, platform, build, 'tutorial01_ispc', '', '')
    render(OS, compiler, platform, build, 'tutorial02_ispc', '', '')
    for model in models:
      render(OS, compiler, platform, build, 'tutorial03_ispc', model, '')
    render(OS, compiler, platform, build, 'tutorial04_ispc', '', '')
    render(OS, compiler, platform, build, 'tutorial05_ispc', '', '')
    for model in models:
      render(OS, compiler, platform, build, 'tutorial06_ispc', model, '')
    render(OS, compiler, platform, build, 'tutorial07_ispc', '', 'static')
    render(OS, compiler, platform, build, 'tutorial07_ispc', '', 'dynamic')
    render(OS, compiler, platform, build, 'tutorial07_ispc', '', 'high_quality')

def renderLoop(OS):
    for compiler in compilers:
      for platform in platforms:
        for build in builds:
          if compiler + '_' + platform + '_' + build in supported_configurations:
            processConfiguration(OS, compiler, platform, build, models[platform])

########################## command line parsing ##########################

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' render  <os> <testDir> <modelDir>\n')
  sys.stderr.write('       ' + sys.argv[0] + ' extract <os> <testDir>\n')
  sys.exit(1)

if len(sys.argv) < 3: printUsage()
mode = sys.argv[1]
OS = sys.argv[2]

if OS == 'windows':
  dash = '\\'
  compilers = compilers_win
  platforms = platforms_win
  builds = builds_win
  modelDir = '%HOMEPATH%\\models\\embree'

else:
  dash = '/'
  compilers = compilers_unix
  platforms = platforms_unix
  builds = builds_unix
  modelDir = '~/models/embree'

if mode == 'render':
  if len(sys.argv) < 4: printUsage()
  testDir = sys.argv[3]
  os.system('mkdir '+testDir)
  if len(sys.argv) > 4: 
    modelDir = sys.argv[4]
  renderLoop(OS)
  sys.exit(1)

