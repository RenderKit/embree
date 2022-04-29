#!/usr/bin/python

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

import sys
import os
import ctypes
import json
import socket
import subprocess

def escape(str):
  str = str.replace("\\",r"\\")
  str = str.replace("\"",r"\"")
  return str

def parse_version(v):
  return tuple(map(int, v.split(".")))

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

debugMode = False
if len(sys.argv)>=1 and sys.argv[0] == "--debug":
  debugMode = True

with open(".ctest_env", "r") as file:
  env = json.load(file)
with open(".ctest_conf", "r") as file:
  conf = json.load(file)

##############
# build step #
##############
if OS == "windows":
  cmd = ""
  for e in env: cmd += e + " &&"
  cmd += " ctest -VV -S scripts/build.cmake" 
  cmd += " " + escape(" ".join(conf)) + "\n"
  
  if (debugMode):
    print(cmd)
  else:
    try:
      subprocess.check_call(cmd, stderr=subprocess.STDOUT, shell=True)
    except subprocess.CalledProcessError as e:
      sys.stderr.write("windows test invokation failed with return code "+str(e.returncode))
      sys.exit(1)
else:
  cmd = escape(" ".join(env))
  cmd += " ctest -VV -S scripts/build.cmake"
  cmd += " " + escape(" ".join(conf)) + "\n"

  if (debugMode):
    print(cmd)
  else:
    print("build", cmd) # TODO: remove
    # need to use bash shell as we configured environment modules only for bash
    process = subprocess.Popen(['bash','-l'], stdin=subprocess.PIPE)
    process.communicate(input=cmd.encode("utf-8"))
    if process.returncode != 0:
      sys.stderr.write("test invokation failed with return code "+str(process.returncode))
      sys.exit(1)