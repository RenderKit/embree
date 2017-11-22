#!/usr/bin/python

import sys
import os
import copy

rule0 = (
  "ID geomID = rtcNewTriangleMesh ( EXPR e0, EXPR e1, EXPR e2, EXPR e3, EXPR e4 );",
  "RTCGeometry geom = rtcNewTriangleMesh (e0,e1,e2,e3,e4);\ngeomID = rtcAttachGeometry(e0,geom);",
  [("rtcMapBuffer(EXPR e0,ID geomID)","rtcNewBuffer(geom,e4)",())]
)

identifier_chars = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_";
number_chars = "0123456789";
delimiter_chars = " \t\n\r"

def parse_delimiter(chars,tokens):
  if not chars[0] in delimiter_chars: return False;
  id = ""
  while chars and chars[0] in delimiter_chars:
    id = id + chars.pop(0)
  tokens.append(id)
  return True
  
def parse_identifier(chars,tokens):
  if (not chars[0] in identifier_chars): return False;
  id = ""
  while chars and (chars[0] in identifier_chars or chars[0] in number_chars):
    id = id + chars.pop(0)
  tokens.append(id)
  return True

def parse_line_comment(chars,tokens):
  if chars[0:2] != ["/","/"]: return False
  id = ""
  while chars and chars[0] != "\n":
    id = id + chars.pop(0)
  tokens.append(id)
  if chars and chars[0] == "\n":
    tokens.append(chars.pop(0))
  return True

def parse_comment(chars,tokens):
  if chars[0:2] != ["/","*"]: return False
  id = ""
  while chars and chars[0:2] != ["*","/"]:
    id = id + chars.pop(0)
  if chars and chars[0:2] == ["*","/"]:
    id = id + chars.pop(0)
    id = id + chars.pop(0)
  tokens.append(id)
  return True   

def parse_number(chars,tokens):
  if (not chars[0] in number_chars): return False;
  id = ""
  while chars and chars[0] in number_chars:
    id = id + chars.pop(0)
  tokens.append(id)
  return True

def tokenize(chars):
  tokens = []
  while chars:
    if chars[0] == "\n": tokens.append(chars.pop(0)); continue;
    elif parse_delimiter(chars,tokens): continue;
    elif parse_identifier(chars,tokens): continue;
    elif parse_number(chars,tokens): continue;
    elif parse_line_comment(chars,tokens): continue;
    elif parse_comment(chars,tokens): continue;
    elif chars[0] == "(": tokens.append(chars.pop(0)); continue;
    elif chars[0] == ")": tokens.append(chars.pop(0)); continue;
    elif chars[0] == "[": tokens.append(chars.pop(0)); continue;
    elif chars[0] == "]": tokens.append(chars.pop(0)); continue;
    elif chars[0] == "{": tokens.append(chars.pop(0)); continue;
    elif chars[0] == "}": tokens.append(chars.pop(0)); continue;
    else: tokens.append(chars.pop(0))
  return tokens

def is_delimiter (c):
  return c in delimiter_chars
  
def is_delimiter_token (token):
  return token[0] in delimiter_chars or token[0].startswith("/*") or token[0].startswith("//")

def no_delimiter_token (token):
  return not is_delimiter_token (token)

def is_identifier_token (token):
  return token[0] in identifier_chars

def parse_expr(tokens,next):
  expr = []
  while (tokens):
    
    if tokens[0] == next:
      tokens.pop(0)
      return expr
    
    elif tokens[0] == "(":
      expr = expr + [tokens.pop(0)] + parse_expr(tokens,")") + [")"]
      
    elif tokens[0] == "[":
      expr = expr + [tokens.pop(0)] + parse_expr(tokens,"]") + ["]"]
      
    elif tokens[0] == "{":
      expr = expr + [tokens.pop(0)] + parse_expr(tokens,"}") + ["}"]
  
    else:
      expr = expr + [tokens.pop(0)]

  raise ValueError()

def match(pattern,tokens,env):

  if is_delimiter_token(tokens[0]):
    tokens.pop(0)
    return True
    
  elif pattern[0] == "ID":
    pattern.pop(0)
    var = pattern[0]
    pattern.pop(0)
    if (not is_identifier_token(tokens[0])):
      return False
    env[var] = [tokens[0]]
    tokens.pop(0)
    return True
    
  elif pattern[0] == "EXPR":
    pattern.pop(0)
    var = pattern[0]
    pattern.pop(0)
    next = pattern[0]
    pattern.pop(0)
    try: expr = parse_expr(tokens,next)
    except ValueError: return False
    if var in env:
      return env[var] == expr
    else:
      env[var] = expr
      return True
    
  elif pattern[0] == tokens[0]:
    pattern.pop(0)
    tokens.pop(0)
    return True

  else:
    return False

def substitute (env,tokens,ident):
  result = []
  for token in tokens:
    if token in env:
      result = result + env[token]
    else:
      result = result + [token]
    if token == "\n" and ident != 0:
      result = result + [" "*ident]
  return result

def print_token_list(list,f):
  for c in list:
    f.write(c)
      
def match_rule (pattern_in, tokens_in, env):
  pattern = copy.deepcopy(pattern_in)
  tokens = copy.deepcopy(tokens_in)
  while (tokens and pattern and match(pattern,tokens,env)):
    continue;
  if pattern:
    return (False,tokens_in)
  else:
    return (True,tokens)

def update_delimiter_ident(token,ident):
  for x in token:
    if x == "\n": ident = 0
    else: ident += 1
  return ident

def apply_rule (rule,env_in,tokens):
  (pattern,subst,follow_rules) = rule
  result = []
  depth = 0
  ident = 0
  while tokens:
    if is_delimiter_token(tokens[0]):
      ident = update_delimiter_ident(tokens[0],ident);
      result = result + [tokens.pop(0)]
    else:
      env = env_in
      (b,next_tokens) = match_rule (pattern,tokens,env)
      tokens = next_tokens
      if (b):
        result = result + substitute (env,subst,ident)
        for follow_rule in follow_rules:
          tokens = apply_rule(follow_rule,env,tokens)
      else:
        if tokens[0] == "{": depth = depth+1
        if tokens[0] == "}": depth = depth-1
        if (depth < 0):
          result = result + tokens
          return result
        result = result + [tokens.pop(0)]
  return result

def tokenize_rule (rule):
  if rule == (): return rule
  (pattern_str, subst_str, next_rule) = rule
  pattern = filter(no_delimiter_token,tokenize(list(pattern_str)))
  subst = tokenize(list(subst_str))
  return (pattern,subst,map (tokenize_rule,next_rule))

with open("test.cpp") as f:
  fstr = f.read()
  content = list(fstr)
  tokens = tokenize(content)
  rule = tokenize_rule(rule0)
#  print('tokens = {}'.format(tokens))
#  print('pattern = {}'.format(pattern))
#  print ('subst = {}'.format(subst))

  env = {}
  result = apply_rule(rule,env,tokens)

#  sys.stdout.write("Source:\n")
#  sys.stdout.write(fstr)
#  sys.stdout.write("\n\n")

  sys.stdout.write("Result:\n")
  print_token_list(result,sys.stdout)
  sys.stdout.write("\n\n")

rule_file = ""
cpp_file_in = ""
cpp_file_out = ""

def printUsage():
  sys.stdout.write("Usage: embree2_to_embree3.py --rules embree2_to_embree3.rules --in infile.cpp --out outfile.cpp\n")

def parseCommandLine(argv):
  global rule_file
  global cpp_file_in
  global cpp_file_out
  if len(argv) == 0:
    return;
  elif len(argv)>=2 and argv[0] == "--rules":
    rule_file = argv[1]
    parseCommandLine(argv[2:len(argv)])
  elif len(argv)>=2 and argv[0] == "--in":
    cpp_file_in = argv[1]
    parseCommandLine(argv[2:len(argv)])
  elif len(argv)>=2 and argv[0] == "--out":
    cpp_file_out = argv[1]
    parseCommandLine(argv[2:len(argv)])
  elif len(argv)>=1 and argv[0] == "--help":
    printUsage()
    sys.exit(1)
  else:
    sys.stderr.write("unknown command line option: "+argv[0])
    printUsage()
    sys.exit(1)
    
parseCommandLine(sys.argv[1:len(sys.argv)])
if (rule_file == ""):
  raise Exception("no rule file specified");
if (cpp_file_in == ""):
  raise Exception("no input file specified");

def parse_rules_file(rule_file):

  def empty_line(str):
    return all(map(is_delimiter,str))
  
  def parse_rule(lines):
    pattern_str = ""
    subst_str = ""
    next_rules = []
    
    while lines and not lines[0].startswith("=>") and not lines[0].startswith("}@@") and not lines[0].startswith("@@{"):
      pattern_str += lines.pop(0)
      
    if lines[0].startswith("=>"):
      lines.pop(0)
      while lines and not lines[0].startswith("=>") and not lines[0].startswith("}@@") and not lines[0].startswith("@@{"):
        subst_str += lines.pop(0)

    pattern_str = pattern_str.strip('\n')
    subst_str = subst_str.strip('\n')
    
    while lines[0].startswith("@@{"):
      lines.pop(0)
      next_rules.append(parse_rule(lines))
    if lines[0].startswith("}@@"):
      lines.pop(0)
      pattern = filter(no_delimiter_token,tokenize(list(pattern_str)))
      subst = tokenize(list(subst_str))
      return (pattern,subst,next_rules)
    print(lines[0])
    raise Exception("parse error in rules file")
  
  with open(rule_file,"r") as f:
    lines = f.readlines()
  
  rules = []
  while lines:
    if (lines[0].startswith("@@{")):
      lines.pop(0)
      rules.append(parse_rule(lines))
    elif (lines[0].startswith("//")):
      lines.pop(0)
    elif empty_line(lines[0]):
      lines.pop(0)
    else:
      raise Exception("parse error in rules file")
    
  return rules


rules = parse_rules_file(rule_file)
print(rules)
#rules = [tokenize_rule(rule0)]
#print(rules)
with open(cpp_file_in,"r") as f:
  tokens = tokenize(list(f.read()))
print(tokens)

for rule in rules:
  env = {}
  tokens = apply_rule(rule,env,tokens)

if (cpp_file_out == ""):
  print_token_list(tokens,sys.stdout)
else:
  with open(cpp_file_out,"w") as f:
    print_token_list(tokens,f)

  
