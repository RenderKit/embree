#!/usr/bin/python

import sys
import os
import ctypes
import time
import datetime
import json
import socket
import subprocess

g_cdash = ""
g_config = {}
g_mode = "Experimental"
g_intensive = False
g_debugMode = False
g_singleConfig = ""

def escape(str):
  str = str.replace("\\",r"\\")
  str = str.replace("\"",r"\"")
  return str

# detect platform
if sys.platform.startswith("win"):
  dash = '\\'
  SEM_FAILCRITICALERRORS = 0x0001
  SEM_NOGPFAULTERRORBOX  = 0x0002
  SEM_NOOPENFILEERRORBOX = 0x8000
  ctypes.windll.kernel32.SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
  OS = "windows"
elif sys.platform.startswith("cygwin"):
  dash = '/'
  SEM_FAILCRITICALERRORS = 0x0001
  SEM_NOGPFAULTERRORBOX  = 0x0002
  SEM_NOOPENFILEERRORBOX = 0x8000
  ctypes.cdll.kernel32.SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX | SEM_NOOPENFILEERRORBOX);
  OS = "windows"
elif sys.platform.startswith("linux"):
  dash = '/'
  OS = "linux"
elif sys.platform.startswith("darwin"):
  dash = '/'
  OS = "macosx"
else:
  print("unknown platform: "+ sys.platform);
  sys.exit(1)

def string_of_isa(isa):
  if type(isa) == str: return isa;
  else: return "ISAS-" + ("-".join(isa))

def createTest(config,OS):
  c = []  # CMake configuration
  e = []  # shell environment
  platform = config["platform"]
  build    = config["build"]
  compiler = config["compiler"]
  isa      = config["isa"]
  tasking  = config["tasking"]
  if "intensive" in config: intensive = config["intensive"]
  else                    : intensive = "OFF"
  if "memcheck" in config: memcheck = config["memcheck"]
  else                   : memcheck = "OFF"
  if "benchmark" in config: benchmark = config["benchmark"]
  else                    : benchmark = "OFF"
  if "sde" in config: sde = config["sde"]
  else              : sde = "OFF"
  if "test" in config: skip_testing = not config["test"]
  else               : skip_testing = False
  if tasking == "INT": tasking = "INTERNAL"
  if tasking == "PPL": tasking = "PPL"
  name = "gitlab-"+platform+"-"+build+"-"+compiler+"-"+string_of_isa(isa)+"-"+tasking
  if "package" in config:
    c.append("-D EMBREE_STACK_PROTECTOR=ON")
    if config["package"] == "ZIP": name = name + "-package-zip"
    else                         : name = name + "-package-installer"

  if "klocwork" in config: name = name + "-klocwork"
  ispc_ext = "-vs2013"
  if "package" in config and OS == 'linux': # we need up to date cmake for RPMs to work properly
    e.append("module load cmake")
  c.append("-D CMAKE_BUILD_TYPE="+build+"")
  if OS == "windows":
    ext = ""
    if platform == "x64":
      ext = " Win64"
    if (compiler == "V141"):
      c.append("-G \"Visual Studio 15 2017"+ext+"\"")
      c.append("-T \"V141\"")
      ispc_ext = "-vs2015"
    elif (compiler == "V140"):
      c.append("-G \"Visual Studio 14 2015"+ext+"\"")
      c.append("-T \"V140\"")
      ispc_ext = "-vs2015"
    elif (compiler == "V120"):
      c.append("-G \"Visual Studio 12 2013"+ext+"\"")
      c.append("-T \"V120\"")
    elif (compiler == "V110"):
      c.append("-G \"Visual Studio 11 2012"+ext+"\"")
      c.append("-T \"V110\"")
    elif (compiler == "ICC"):
      c.append("-G \"Visual Studio 12 2013"+ext+"\"")
      c.append("-T \"Intel C++ Compiler 17.0\"")
    elif (compiler == "ICC18"):
      c.append("-G \"Visual Studio 12 2013"+ext+"\"")
      c.append("-T \"Intel C++ Compiler 18.0\"")
    elif (compiler == "ICC17"):
      c.append("-G \"Visual Studio 12 2013"+ext+"\"")
      c.append("-T \"Intel C++ Compiler 17.0\"")
    elif (compiler == "ICC16"):
      c.append("-G \"Visual Studio 12 2013"+ext+"\"")
      c.append("-T \"Intel C++ Compiler 16.0\"")
    elif (compiler == "ICC15"):
      c.append("-G \"Visual Studio 12 2013"+ext+"\"")
      c.append("-T \"Intel C++ Compiler XE 15.0\"")
    elif (compiler == "CLANG"):
      c.append("-G \"Visual Studio 12 2013"+ext+"\"")
      c.append("-T \"LLVM-vs2013\"")
    else:
      raise ValueError('unknown compiler: ' + compiler + '')
  else:
    if (compiler == "ICC18"):
      c.append("-D CMAKE_CXX_COMPILER=icpc -D CMAKE_C_COMPILER=icc")
      e.append("module load intel/2018")
    elif (compiler == "ICC17"):
      c.append("-D CMAKE_CXX_COMPILER=icpc -D CMAKE_C_COMPILER=icc")
      e.append("module load intel/2017")
    elif (compiler == "ICC16"):
      c.append("-D CMAKE_CXX_COMPILER=icpc -D CMAKE_C_COMPILER=icc")
      e.append("module load intel/2016")
    elif (compiler == "ICC15"):
      c.append("-D CMAKE_CXX_COMPILER=icpc -D CMAKE_C_COMPILER=icc")
      e.append("module load intel/2015")
    elif (compiler == "ICC"):
      c.append("-D CMAKE_CXX_COMPILER=icpc -D CMAKE_C_COMPILER=icc")
      e.append("module load intel")
    elif (compiler == "GCC"):
      c.append("-D CMAKE_CXX_COMPILER=g++ -D CMAKE_C_COMPILER=gcc")
    elif (compiler == "CLANG4"):
      c.append("-D CMAKE_CXX_COMPILER=clang++ -D CMAKE_C_COMPILER=clang")
      e.append("module avail && module load clang/4")
    elif (compiler == "CLANG3.9"):
      c.append("-D CMAKE_CXX_COMPILER=clang++ -D CMAKE_C_COMPILER=clang")
      e.append("module load clang/3.9")
    elif (compiler == "CLANG3.8"):
      c.append("-D CMAKE_CXX_COMPILER=clang++ -D CMAKE_C_COMPILER=clang")
      e.append("module load clang/3.8")
    elif (compiler == "CLANG"):
      c.append("-D CMAKE_CXX_COMPILER=clang++ -D CMAKE_C_COMPILER=clang")
    else:
      raise ValueError('unknown compiler: ' + compiler + '')

  if type(isa) == str:
    c.append("-D EMBREE_MAX_ISA="+isa+"")
  else:
    c.append("-D EMBREE_MAX_ISA=NONE")
    if "SSE2"      in isa: c.append("-D EMBREE_ISA_SSE2=ON")
    else                 : c.append("-D EMBREE_ISA_SSE2=OFF")
    if "SSE42"     in isa: c.append("-D EMBREE_ISA_SSE42=ON")
    else                 : c.append("-D EMBREE_ISA_SSE42=OFF")
    if "AVX"       in isa: c.append("-D EMBREE_ISA_AVX=ON")
    else                 : c.append("-D EMBREE_ISA_AVX=OFF")
    if "AVX2"      in isa: c.append("-D EMBREE_ISA_AVX2=ON")
    else                 : c.append("-D EMBREE_ISA_AVX2=OFF")
    if "AVX512KNL" in isa: c.append("-D EMBREE_ISA_AVX512KNL=ON")
    else                 : c.append("-D EMBREE_ISA_AVX512KNL=OFF")
    if "AVX512SKX" in isa: c.append("-D EMBREE_ISA_AVX512SKX=ON")
    else                 : c.append("-D EMBREE_ISA_AVX512SKX=OFF")
    
  c.append("-D EMBREE_TASKING_SYSTEM="+tasking+"")
  c.append("-D EMBREE_TESTING_MEMCHECK="+memcheck+"")
  c.append("-D EMBREE_TESTING_INTENSIVE="+intensive+"")
  c.append("-D EMBREE_TESTING_SDE="+sde+"")

  ispc_enabled = True
  if "ISPC_SUPPORT" in config:
    c.append("-D EMBREE_ISPC_SUPPORT="+config["ISPC_SUPPORT"])
    if config["ISPC_SUPPORT"] == "OFF": name += "-noispc"
    ispc_enabled = False
  if "STATIC_LIB" in config:
    c.append("-D EMBREE_STATIC_LIB="+config["STATIC_LIB"])
    if config["STATIC_LIB"] == "ON": name += "-static"
  if "TUTORIALS" in config:
    c.append("-D EMBREE_TUTORIALS="+config["TUTORIALS"])
    if config["TUTORIALS"] == "OFF": name += "-notutorials"
  if "BACKFACE_CULLING" in config:
    c.append("-D EMBREE_BACKFACE_CULLING="+config["BACKFACE_CULLING"])
    if config["BACKFACE_CULLING"] == "ON": name += "-backfaceculling"
  if "IGNORE_INVALID_RAYS" in config:
    c.append("-D EMBREE_IGNORE_INVALID_RAYS="+config["IGNORE_INVALID_RAYS"])
    if config["IGNORE_INVALID_RAYS"] == "ON": name += "-noinvalid"
  if "INTERSECTION_FILTER" in config:
    c.append("-D EMBREE_INTERSECTION_FILTER="+config["INTERSECTION_FILTER"])
    if config["INTERSECTION_FILTER"] == "OFF": name += "-nofilter"
  if "RAY_MASK" in config:
    c.append("-D EMBREE_RAY_MASK="+config["RAY_MASK"])
    if config["RAY_MASK"] == "ON": name += "-raymask"
  if "RAY_PACKETS" in config:
    c.append("-D EMBREE_RAY_PACKETS="+config["RAY_PACKETS"])
    if config["RAY_PACKETS"] == "OFF": name += "-nopackets"
  if "STAT_COUNTERS" in config:
    c.append("-D EMBREE_STAT_COUNTERS="+config["STAT_COUNTERS"])
    if config["STAT_COUNTERS"] == "ON": name += "-stats"
  if "TRIS" in config:
    c.append("-D EMBREE_GEOMETRY_TRIANGLES="+config["TRIS"])
    if config["TRIS"] == "ON": name += "-tris"
  if "QUADS" in config:
    c.append("-D EMBREE_GEOMETRY_QUADS="+config["QUADS"])
    if config["QUADS"] == "ON": name += "-quads"
  if "LINES" in config:
    c.append("-D EMBREE_GEOMETRY_LINES="+config["LINES"])
    if config["LINES"] == "ON": name += "-lines"
  if "HAIR" in config:
    c.append("-D EMBREE_GEOMETRY_HAIR="+config["HAIR"])
    if config["HAIR"] == "ON": name += "-hair"
  if "SUBDIV" in config:
    c.append("-D EMBREE_GEOMETRY_SUBDIV="+config["SUBDIV"])
    if config["SUBDIV"] == "ON": name += "-subdiv"
  if "USERGEOM" in config:
    c.append("-D EMBREE_GEOMETRY_USER="+config["USERGEOM"])
    if config["USERGEOM"] == "ON": name += "-usergeom"

  if OS == "linux":
    if ispc_enabled:     c.append("-D EMBREE_ISPC_EXECUTABLE=/NAS/packages/apps/ispc/1.9.1/ispc")
    if tasking == "TBB": c.append("-D EMBREE_TBB_ROOT=/NAS/packages/apps/tbb/tbb-2017-linux")
  elif OS == "macosx":
    if ispc_enabled:     c.append("-D EMBREE_ISPC_EXECUTABLE=/Network/nfs/NAS/packages/apps/ispc/1.9.1-osx/ispc")
    if tasking == "TBB": c.append("-D EMBREE_TBB_ROOT=/Network/nfs/NAS/packages/apps/tbb/tbb-2017-osx")
  elif OS == "windows":
    if ispc_enabled:
#      c.append("-D EMBREE_ISPC_EXECUTABLE=N:\\packages\\apps\\ispc\\1.9.1-windows"+ispc_ext+"\\ispc.exe")
      c.append("-D EMBREE_ISPC_EXECUTABLE=C:\\ispc-v1.9.1-windows"+ispc_ext+"\\ispc.exe")

    if tasking == "TBB": 
#      tbb_path = "N:\\packages\\apps\\tbb\\tbb-2017-windows"
      tbb_path = "C:\\embree-testing\\dependencies\\tbb-2017-windows"
      c.append("-D EMBREE_TBB_ROOT="+tbb_path)

      if platform == "x64":
        e.append("set PATH="+tbb_path+"\\bin\\intel64\\vc12;%PATH%")
      else:
        e.append("set PATH="+tbb_path+"\\bin\\ia32\\vc12;%PATH%")
  else:
    sys.stderr.write("unknown operating system "+OS)
    sys.exit(1)

  if "klocwork" in config:
    c.append("-D EMBREE_TESTING_KLOCWORK="+config["klocwork"])

  if "package" in config:
    c.append("-D EMBREE_TESTING_PACKAGE=ON")
    c.append("-D EMBREE_TUTORIALS_IMAGE_MAGICK=OFF")
    c.append("-D EMBREE_TUTORIALS_LIBJPEG=OFF")
    c.append("-D EMBREE_TUTORIALS_LIBPNG=OFF")
    c.append("-D EMBREE_TUTORIALS_OPENEXR=OFF")
    if OS == "linux" and config["package"] == "ZIP":
      c.append("-D EMBREE_ZIP_MODE=ON")
      c.append("-D CMAKE_SKIP_INSTALL_RPATH=ON")
      c.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      c.append("-D CMAKE_INSTALL_LIBDIR=lib")
      c.append("-D CMAKE_INSTALL_DOCDIR=doc")
      c.append("-D CMAKE_INSTALL_BINDIR=bin")
    elif OS == "linux" and config["package"] == "RPM":
      c.append("-D EMBREE_ZIP_MODE=OFF")
      c.append("-D CMAKE_SKIP_INSTALL_RPATH=OFF")
      c.append("-D CMAKE_INSTALL_PREFIX=/usr")
      c.append("-D EMBREE_TBB_ROOT=/usr")
    elif OS == "macosx" and config["package"] == "ZIP":
      c.append("-D EMBREE_ZIP_MODE=ON")
      c.append("-D CMAKE_SKIP_INSTALL_RPATH=ON")
      c.append("-D CMAKE_MACOSX_RPATH=ON")
      c.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      c.append("-D CMAKE_INSTALL_LIBDIR=lib")
      c.append("-D CMAKE_INSTALL_DOCDIR=doc")
      c.append("-D CMAKE_INSTALL_BINDIR=bin")
    elif OS == "macosx" and config["package"] == "PKG":
      c.append("-D EMBREE_ZIP_MODE=OFF")
      c.append("-D CMAKE_SKIP_INSTALL_RPATH=OFF")
      c.append("-D CMAKE_INSTALL_PREFIX=/opt/local")
      c.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      c.append("-D CMAKE_INSTALL_LIBDIR=lib")
      c.append("-D CMAKE_INSTALL_DOCDIR=../../Applications/Embree2/doc")
      c.append("-D CMAKE_INSTALL_BINDIR=../../Applications/Embree2/bin")
      c.append("-D EMBREE_TBB_ROOT=/opt/local")
    elif OS == "windows" and config["package"] == "ZIP":
      c.append("-D EMBREE_ZIP_MODE=ON")
      c.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      c.append("-D CMAKE_INSTALL_LIBDIR=lib")
      c.append("-D CMAKE_INSTALL_DATAROOTDIR=")
      c.append("-D CMAKE_INSTALL_DOCDIR=doc")
      c.append("-D CMAKE_INSTALL_BINDIR=bin")
    elif OS == "windows" and config["package"] == "MSI":
      c.append("-D EMBREE_ZIP_MODE=OFF")
      c.append("-D CMAKE_INSTALL_INCLUDEDIR=include")
      c.append("-D CMAKE_INSTALL_LIBDIR=lib")
      c.append("-D CMAKE_INSTALL_DATAROOTDIR=")
      c.append("-D CMAKE_INSTALL_DOCDIR=doc")
      c.append("-D CMAKE_INSTALL_BINDIR=bin")
    else:
      sys.stderr.write("unknown package mode: "+OS+":"+config["package"])
      sys.exit(1)

  return [name,build,skip_testing,benchmark,c,e]

# runs all tests for specified host machine
def runConfig(cfg):
  [name,build,skip_testing,benchmark,conf,env] = createTest(cfg,OS)
  ctest = ""
  ctest +=  "ctest -VV -S scripts/test.cmake"
  if g_cdash != "": ctest += " -D CTEST_DROP_SITE="+g_cdash
  ctest += " -D TEST_NAME=\""+name+"\""
  if skip_testing: ctest += " -D CTEST_SKIP_TESTING=ON"
  else           : ctest += " -D CTEST_SKIP_TESTING=OFF"
  ctest += " -D CTEST_CONFIGURATION_TYPE=\""+build+"\""
  ctest += " -D CTEST_BUILD_OPTIONS=\"" + escape(" ".join(conf))+"\""
  if g_debugMode:
    print(name)
    for e in env: print('    '+e)
    print('    '+ctest+'\n')
  else:
    cmd = ""
    for e in env: cmd += e + " && "
    cmd += ctest+"\n"
    subprocess.check_call(cmd, shell=True) 

def parseCommandLine(argv):
  global g_cdash
  global g_config
  global g_mode
  global g_intensive
  global g_debugMode
  if len(argv) == 0:
    return;
  elif len(argv)>=2 and argv[0] == "--cdash":
    g_cdash = argv[1]
    parseCommandLine(argv[2:len(argv)])
  elif len(argv)>=2 and argv[0] == "--mode":
    g_mode = argv[1]
    parseCommandLine(argv[2:len(argv)])
  elif len(argv)>=1 and argv[0] == "--intensive":
    g_intensive = True
    parseCommandLine(argv[1:len(argv)])
  elif len(argv)>=1 and argv[0] == "--debug":
    g_debugMode = True
    parseCommandLine(argv[1:len(argv)])
  elif len(argv)>=1 and argv[0] == "--help":
    printUsage()
    return;
  elif ':' in argv[0]:
    p = argv[0].split(":")
    if p[1] == "False": g_config[p[0]] = False
    elif p[1] == "True": g_config[p[0]] = True
    elif p[1].startswith('[') and p[1].endswith(']'):
      list = p[1][1:-1].replace(' ','')
      g_config[p[0]] = list.split(',')
    else:
      g_config[p[0]] = p[1]
    parseCommandLine(argv[1:len(argv)])
  else:
    sys.stderr.write("unknown command line option: "+argv[0])
    sys.exit(1)
      
parseCommandLine(sys.argv[1:len(sys.argv)])
runConfig(g_config)




