#!/usr/bin/env python3

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

import sys
import json
import glob

sys.argv
if len(sys.argv) < 3:
  print("usage: merge_json.py common_file_prefix output_filename")
  sys.exit(1)

common_file_prefix = sys.argv[1]
output_filename = sys.argv[2]

files = glob.glob(common_file_prefix+"*.json")
files.sort()

if (len(files) < 1):
  sys.exit(1)

hero_file = files[0] 
with open(hero_file, 'r') as f:
  hero_data = json.load(f)

for i in range(1, len(files)):
  with open(files[i], 'r') as f:
    data = json.load(f)
    hero_data['benchmarks'].extend(data['benchmarks'])

with open(output_filename, 'w') as f:
  f.write(json.dumps(hero_data, indent=2))