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
#   Install Visual Studio 2013
#   Install Intel C++ Compiler
#   Check out Embree into <embree_dir>

# Instructions:
#   Open the "Visual Studio x64 Cross Tools Command Prompt (2013)"
#   cd <embree_dir>
#   <python_dir>\python.exe <embree_dir>\scripts\benchmark.py run     windows test_dir
#   <python_dir>\python.exe <embree_dir>\scripts\benchmark.py compile windows test_dir

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
#   ./scripts/benchmark.py run     linux <model_dir> test_dir
#   ./scripts/benchmark.py compile linux <model_dir> test_dir

import sys
import os
import re

dash = '/'

########################## configuration ##########################

#compilers_win = ['V120']
#compilers_win = ['ICC']
#compilers_win  = ['V120', 'ICC']
compilers_win  = ['ICC', 'V110', 'V120']
#compilers_win  = ['V110', 'V120']
#compilers_unix = ['ICC']
#compilers_unix = ['GCC', 'CLANG']
compilers_unix = ['GCC', 'CLANG', 'ICC']
compilers      = []

#platforms_win  = ['Win32']
#platforms_win  = ['x64']
platforms_win  = ['Win32', 'x64']
platforms_unix = ['x64']
platforms      = []

#builds_win = ['Debug']
builds_win = ['RelWithDebInfo']
#builds_win = ['RelWithDebInfo', 'Debug']
#builds_unix = ['Debug']
builds_unix = ['RelWithDebInfo']
#builds_unix = ['RelWithDebInfo', 'Debug']
builds = []

#ISAs_win  = ['AVX2']
ISAs_win  = ['SSE2', 'AVX', 'AVX512']
#ISAs_unix = ['AVX2']
ISAs_unix = ['SSE2', 'AVX', 'AVX512']
ISAs = []

supported_configurations = [
  'V100_Win32_RelWithDebInfo_SSE2', 'V100_Win32_RelWithDebInfo_SSE4.2',
  'V100_x64_RelWithDebInfo_SSE2',   'V100_x64_RelWithDebInfo_SSE4.2',
  'V110_Win32_RelWithDebInfo_SSE2', 'V120_Win32_RelWithDebInfo_SSE4.2', 'V120_Win32_RelWithDebInfo_AVX',
  'V110_x64_RelWithDebInfo_SSE2',   'V120_x64_RelWithDebInfo_SSE4.2',   'V120_x64_RelWithDebInfo_AVX',  
  'V120_Win32_RelWithDebInfo_SSE2', 'V120_Win32_RelWithDebInfo_SSE4.2', 'V120_Win32_RelWithDebInfo_AVX', 'V120_Win32_RelWithDebInfo_AVX2', 
  'V120_x64_RelWithDebInfo_SSE2',   'V120_x64_RelWithDebInfo_SSE4.2',   'V120_x64_RelWithDebInfo_AVX',   'V120_x64_RelWithDebInfo_AVX2', 
  'ICC_Win32_RelWithDebInfo_SSE2',  'ICC_Win32_RelWithDebInfo_SSE4.2',  'ICC_Win32_RelWithDebInfo_AVX',  'ICC_Win32_RelWithDebInfo_AVX2', 
  'ICC_x64_RelWithDebInfo_SSE2',    'ICC_x64_RelWithDebInfo_SSE4.2',    'ICC_x64_RelWithDebInfo_AVX',    'ICC_x64_RelWithDebInfo_AVX2', 
  'GCC_x64_RelWithDebInfo_SSE2',    'GCC_x64_RelWithDebInfo_SSE4.2',    'GCC_x64_RelWithDebInfo_AVX',    'GCC_x64_RelWithDebInfo_AVX2', 
  'CLANG_x64_RelWithDebInfo_SSE2',  'CLANG_x64_RelWithDebInfo_SSE4.2',  'CLANG_x64_RelWithDebInfo_AVX',  'CLANG_x64_RelWithDebInfo_AVX2',  
  ]

models = {}
#models['Win32'] = [ 'conference' ]
#models['x64'  ] = [ 'crown' ]
models['Win32'] = [ 'conference', 'sponza' ]
models['x64'  ] = [ 'conference', 'sponza', 'headlight', 'crown', 'xyz_dragon', 'powerplant' ]

modelDir  = ''
testDir = ''

def configName(OS, compiler, platform, build, isa, tasking, tutorial, scene, flags):
  cfg = OS + '_' + compiler + '_' + platform + '_' + build + '_' + isa + '_' + tasking
  if tutorial != '':
    cfg += '_' + tutorial
  if scene != '':
    cfg += '_' + scene
  if flags != '':
    cfg += '_' + flags
  return cfg

########################## compiling ##########################

def compile(OS,compiler,platform,build,isa,tasking):

  base = configName(OS, compiler, platform, build, isa, tasking, 'build', '', '')
  logFile = testDir + dash + base + '.log'

  if OS == 'windows':

    # generate CMake generator name
    full_compiler = compiler
    if (compiler == 'V110'):
	  generator = 'Visual Studio 11 2012'
    elif (compiler == 'V120'):
      generator = 'Visual Studio 12 2013'
    elif (compiler == 'ICC'):
      generator = 'Visual Studio 12 2013'
      full_compiler = '"Intel C++ Compiler XE 15.0" '
    else:
      sys.stderr.write('unknown compiler: ' + compiler + '\n')
      sys.exit(1)

    if (platform == 'Win32'):
      generator += ''
    elif (platform == 'x64'):
	  generator += ' Win64'
    else:
      sys.stderr.write('unknown platform: ' + platform + '\n')
      sys.exit(1)

    # generate build directory
    if os.path.exists('build'):
      ret = os.system('rm -rf build && mkdir build')
      if ret != 0:
        sys.stdout.write("Cannot delete build folder!")
        return ret
    else:	
      os.system('mkdir build')	

    # generate solution files using cmake
    command = 'cmake -L '
    command += ' -G "' + generator + '"'
    command += ' -T ' + full_compiler
    command += ' -A ' + platform
    command += ' -D COMPILER=' + compiler
    command += ' -D XEON_ISA=' + isa
    command += ' -D RTCORE_RAY_MASK=OFF'
    command += ' -D RTCORE_BACKFACE_CULLING=OFF'
    command += ' -D RTCORE_INTERSECTION_FILTER=ON'
    command += ' -D RTCORE_BUFFER_STRIDE=ON'
    command += ' -D RTCORE_STAT_COUNTERS=OFF'
    if tasking == 'tbb':
      command += ' -D RTCORE_TASKING_SYSTEM=TBB'
    elif tasking == 'internal':
      command += ' -D RTCORE_TASKING_SYSTEM=INTERNAL'
    else:
      sys.stdout.write("invalid tasking system: "+tasking)
      return 1
    command += ' ..'
    os.system('echo ' + command + ' > ' + logFile)
    ret = os.system('cd build && ' + command + ' >> ../' + logFile)
    if ret != 0: return ret

    # compile Embree
    command =  'msbuild build\embree2.sln' + ' /m /nologo /p:Platform=' + platform + ' /p:Configuration=' + build + ' /t:rebuild /verbosity:n' 
    os.system('echo ' + command + ' >> ' + logFile)
    return os.system(command + ' >> ' + logFile)
  
  else:

    if (platform != 'x64'):
      sys.stderr.write('unknown platform: ' + platform + '\n')
      sys.exit(1)

    # compile Embree
    command = 'mkdir -p build && cd build && cmake > /dev/null'
    command += ' -D COMPILER=' + compiler
    command += ' -D CMAKE_BUILD_TYPE='+build
    command += ' -D XEON_ISA=' + isa
    command += ' -D RTCORE_RAY_MASK=OFF'
    command += ' -D RTCORE_BACKFACE_CULLING=OFF'
    command += ' -D RTCORE_INTERSECTION_FILTER=ON'
    command += ' -D RTCORE_BUFFER_STRIDE=ON'
    command += ' -D RTCORE_STAT_COUNTERS=OFF'
    if tasking == 'tbb':
      command += ' -D RTCORE_TASKING_SYSTEM=TBB'
    elif tasking == 'internal':
      command += ' -D RTCORE_TASKING_SYSTEM=INTERNAL'
    else:
      sys.stdout.write("invalid tasking system: "+tasking)
      return 1
    command += ' .. && make clean && make -j 8'
    command += ' &> ../' + logFile
    return os.system(command)

def compileLoop(OS):
    for compiler in compilers:
      for platform in platforms:
        for build in builds:
          for isa in ISAs:
            for tasking in ['tbb','internal']:
              if (compiler + '_' + platform + '_' + build + '_' + isa) in supported_configurations:
                sys.stdout.write(OS + ' ' + compiler + ' ' + platform + ' ' + build + ' ' + isa + ' ' + tasking)
                sys.stdout.flush()
                ret = compile(OS,compiler,platform,build,isa,tasking)
                if ret != 0: sys.stdout.write(" [failed]\n")
                else:        sys.stdout.write(" [passed]\n")

########################## rendering ##########################

def render(OS, compiler, platform, build, isa, tasking, tutorial, args, scene, flags):
  sys.stdout.write("  "+tutorial)
  if scene != '': sys.stdout.write(' '+scene)
  if flags != '' and flags != 'default': sys.stdout.write(' '+flags)
  sys.stdout.flush()
  base = configName(OS, compiler, platform, build, isa, tasking, tutorial, scene, flags)
  logFile = testDir + dash + base + '.log'
  imageFile = testDir + dash + base + '.tga'
  if os.path.exists(logFile):
    sys.stdout.write(" [skipped]\n")
  else:
    if OS == 'windows': command = 'build' + '\\' + build + '\\' + tutorial + ' ' + args + ' '
    else:               command = 'build' + '/' + tutorial + ' ' + args + ' '
    if tutorial != 'verify' and tutorial != 'benchmark':
      command += '-rtcore verbose=2'
    if flags != "": 
      if flags != "default":
        command += ",flags=" + flags
      command += ' -size 1024 1024 -o ' + imageFile
    command += ' > ' + logFile
    ret = os.system(command)
    if ret == 0: sys.stdout.write(" [passed]\n")
    else       : sys.stdout.write(" [failed]\n")

def render_viewer(OS, compiler, platform, build, isa, tasking, ty, scene, flags):
  render(OS,compiler,platform,build,isa,tasking,"viewer"+ty," -c " + modelDir + dash + scene + dash + scene + '_regression.ecs ',scene,flags)

def render_pathtracer(OS, compiler, platform, build, isa, tasking, ty, scene, flags):
  render(OS,compiler,platform,build,isa,tasking,"pathtracer"+ty," -c " + modelDir + dash + scene + dash + scene + '_regression.ecs ',scene,flags)

def render_hair_geometry(OS, compiler, platform, build, isa, tasking, ty, scene, flags):
  render(OS,compiler,platform,build,isa,tasking,"hair_geometry"+ty," -c " + modelDir + dash + scene + dash + scene + '_regression.ecs ',scene,flags)

def render_viewer(OS, compiler, platform, build, isa, tasking, ty, scene, flags):
  if scene[0:6] == 'subdiv':
    render(OS,compiler,platform,build,isa,tasking,"viewer"+ty," -i tutorials/models/" + scene + '.xml',scene,flags)
  else:
    render(OS,compiler,platform,build,isa,tasking,"viewer"+ty," -c " + modelDir + dash + scene + dash + scene + '_regression.ecs ',scene,flags)

def processConfiguration(OS, compiler, platform, build, isa, tasking, models):
  sys.stdout.write('compiling configuration ' + compiler + ' ' + platform + ' ' + build + ' ' + isa + ' ' + tasking)
  sys.stdout.flush()
  ret = compile(OS,compiler,platform,build,isa,tasking)
  if ret != 0: sys.stdout.write(" [failed]\n")
  else:        
    sys.stdout.write(" [passed]\n")

    render(OS, compiler, platform, build, isa, tasking, 'verify', '', '', '')
    render(OS, compiler, platform, build, isa, tasking, 'benchmark', '', '', '')

    render(OS, compiler, platform, build, isa, tasking, 'bvh_builder', '', '', '')
    for ty in ['','_ispc']:
      render(OS, compiler, platform, build, isa, tasking, 'triangle_geometry'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'dynamic_scene'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'user_geometry'+ty, '', '', 'default')
      for model in models:
        render_viewer(OS, compiler, platform, build, isa, tasking, ty, model, 'static')
        render_viewer(OS, compiler, platform, build, isa, tasking, ty, model, 'dynamic')
        render_viewer(OS, compiler, platform, build, isa, tasking, ty, model, 'high_quality')
        render_viewer(OS, compiler, platform, build, isa, tasking, ty, model, 'robust')
        render_viewer(OS, compiler, platform, build, isa, tasking, ty, model, 'compact')

      render(OS, compiler, platform, build, isa, tasking, 'instanced_geometry'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'intersection_filter'+ty, '', '', 'default')

      for model in models:
        render_pathtracer(OS, compiler, platform, build, isa, tasking, ty, model, 'default')

      render(OS, compiler, platform, build, isa, tasking, 'hair_geometry'+ty, '', '', 'default')
      render_hair_geometry(OS, compiler, platform, build, isa, tasking, ty, 'tighten', 'default')
      if platform == "x64" and OS != 'macosx': # not enough memory on MacOSX test machine:
        render_hair_geometry(OS, compiler, platform, build, isa, tasking, ty, 'sophie', 'default')
        render_hair_geometry(OS, compiler, platform, build, isa, tasking, ty, 'sophie_mblur', 'default')

      render(OS, compiler, platform, build, isa, tasking, 'subdivision_geometry'+ty, '', '', 'default')
      render(OS, compiler, platform, build, isa, tasking, 'displacement_geometry'+ty, '', '', 'default')
    
      render_viewer(OS, compiler, platform, build, isa, tasking, ty, 'subdiv0', 'default')
      render_viewer(OS, compiler, platform, build, isa, tasking, ty, 'subdiv1', 'default')
      render_viewer(OS, compiler, platform, build, isa, tasking, ty, 'subdiv2', 'default')
      render_viewer(OS, compiler, platform, build, isa, tasking, ty, 'subdiv3', 'default')
      render_viewer(OS, compiler, platform, build, isa, tasking, ty, 'subdiv4', 'default')
      render_viewer(OS, compiler, platform, build, isa, tasking, ty, 'subdiv5', 'default')
      render_viewer(OS, compiler, platform, build, isa, tasking, ty, 'subdiv6', 'default')
      render_viewer(OS, compiler, platform, build, isa, tasking, ty, 'bigguy', 'default')
      render_viewer(OS, compiler, platform, build, isa, tasking, ty, 'cupid', 'default')
			    
def renderLoop(OS):
    for compiler in compilers:
      for platform in platforms:
        for build in builds:
          for isa in ISAs:
            for tasking in ['tbb','internal']:
              if (compiler + '_' + platform + '_' + build + '_' + isa) in supported_configurations:
                processConfiguration(OS, compiler, platform, build, isa, tasking, models[platform])

########################## command line parsing ##########################

def printUsage():
  sys.stderr.write('Usage: ' + sys.argv[0] + ' compile <os> <testDir>\n')
  sys.stderr.write('       ' + sys.argv[0] + ' run     <os> <testDir> <modelDir>\n')
  sys.exit(1)

if len(sys.argv) < 3: printUsage()
mode = sys.argv[1]
OS = sys.argv[2]

if OS == 'windows':
  dash = '\\'
  compilers = compilers_win
  platforms = platforms_win
  builds = builds_win
  ISAs = ISAs_win
  modelDir = '%HOMEPATH%\\models\\embree'

else:
  dash = '/'
  compilers = compilers_unix
  platforms = platforms_unix
  builds = builds_unix
  ISAs = ISAs_unix
  modelDir = '~/models/embree'

if mode == 'run':
  if len(sys.argv) < 4: printUsage()
  testDir = sys.argv[3]
  if not os.path.exists(testDir):
    os.system('mkdir '+testDir)
  if len(sys.argv) > 4: 
    modelDir = sys.argv[4]
  renderLoop(OS)
  sys.exit(1)

if mode == 'compile':
  if len(sys.argv) < 4: printUsage()
  testDir = sys.argv[3]
  if not os.path.exists(testDir):
    os.system('mkdir '+testDir)
  compileLoop(OS)
  sys.exit(1)

