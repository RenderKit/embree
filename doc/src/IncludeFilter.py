#!/usr/bin/python

import sys
import json
import subprocess 

def include(key, val):
    if key == 'CodeBlock':
        [[id, classes, keyvals], code] = val
        for kv in keyvals:
            if kv[0] == 'include':
                src0 = subprocess.check_output(['pandoc', '-s', kv[1], '-t', 'json'])
                j = json.loads(src0)
                if type(j) is list: return recurse(j[1])
                else              : return recurse(j['blocks'])
            elif kv[0] == 'image':
                return {"t":"Para","c":[{"t":"Str","c":"![][" + kv[1] + "]"}]}
    return None

def recurse(x):
    if isinstance(x, list):
        lst = []
        for item in x:
            if isinstance(item, dict) and 't' in item and 'c' in item:
                res = include(item['t'], item['c'])
                if res is None:
                    lst.append(recurse(item))
                elif isinstance(res, list):
                    for z in res: lst.append(z)
                else:
                    lst.append(res)
            else:
                lst.append(recurse(item))
        return lst
    elif isinstance(x, dict):
        dct = {}
        for k in x:
            dct[k] = recurse(x[k])
        return dct
    else:
        return x

if __name__ == "__main__":
    doc = json.loads(sys.stdin.read())
    ret = recurse(doc)
    json.dump(ret, sys.stdout)
