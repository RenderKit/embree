#!/usr/bin/python

import sys
import os
import copy
import re

rule_file = ""
cpp_file_in = ""
cpp_file_out = ""
ispc_mode = False

unique_id = 0
identifier_begin_chars = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ_";
identifier_cont_chars = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_";
number_chars = "0123456789";
delimiter_chars = " \t\n\r"

def parse_delimiter(chars,tokens):
  if not chars[0] in delimiter_chars: return False;
  id = ""
  while chars and chars[0] in delimiter_chars:
    id = id + chars.pop(0)
  tokens.append(id)
  return True
  
def parse_identifier(chars,tokens,parse_pattern):
  if (not chars[0] in identifier_begin_chars): return False;
  id = ""
  while chars and (chars[0] in identifier_cont_chars):
    id = id + chars.pop(0)
  if (id == "size_t" and ispc_mode): tokens.append("uintptr_t")
  else: tokens.append(id)
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

def parse_regexpr(chars,tokens):
  if chars[0] != "(": return False
  chars.pop(0)
  parse_identifier(chars,tokens,True)
  if chars.pop(0) != ",":
    return False
  id = ""
  while chars and chars[0] != ")":
    id = id + chars.pop(0)
  if chars: chars.pop(0)
  tokens.append(id)
  return True

def parse_number(chars,tokens):
  if (not chars[0] in number_chars): return False;
  id = ""
  while chars and chars[0] in number_chars:
    id = id + chars.pop(0)
  tokens.append(id)
  return True

def tokenize(chars,parse_pattern):
  tokens = []
  while chars:
    if chars[0] == "\n": tokens.append(chars.pop(0)); continue;
    elif parse_delimiter(chars,tokens): continue;
    elif parse_identifier(chars,tokens,parse_pattern): continue;
    elif parse_number(chars,tokens): continue;
    elif parse_line_comment(chars,tokens): continue;
    elif parse_comment(chars,tokens): continue;
    elif tokens and parse_pattern and tokens[-1] == "REGEXPR": parse_regexpr(chars,tokens); continue;
    elif len(chars) >= 2 and chars[0] == "-" and chars[1] == ">": chars.pop(0); chars.pop(0); tokens.append("->"); continue;
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
  return token[0] in delimiter_chars or token.startswith("/*") or token.startswith("//")

def no_delimiter_token (token):
  return not is_delimiter_token (token)

def is_identifier_token (token):
  return token[0] in identifier_begin_chars

def parse_expr_list(tokens,tpos,term_token):
  expr = []
  while (tpos < len(tokens)):
    
    if tokens[tpos] == term_token:
      return (expr,tpos)
    
    elif tokens[tpos] == "(":
      tpos+=1
      (e,tpos) = parse_expr_list(tokens,tpos,")")
      expr = expr + ["("] + e + [")"]
      tpos+=1
      
    elif tokens[tpos] == "{":
      tpos+=1
      (e,tpos) = parse_expr_list(tokens,tpos,"}")
      expr = expr + ["{"] + e + ["}"]
      tpos+=1
  
    else:
      expr.append(tokens[tpos])
      tpos+=1

  raise ValueError()
      
def parse_expr(tokens,tpos,term_token):
  expr = []
  while (tpos < len(tokens)):
    
    if tokens[tpos] == term_token or tokens[tpos] == ",":
      return (expr,tpos)
    
    elif tokens[tpos] == "(":
      tpos+=1
      (e,tpos) = parse_expr_list(tokens,tpos,")")
      expr = expr + ["("] + e + [")"]
      tpos+=1
      
    elif tokens[tpos] == "{":
      tpos+=1
      (e,tpos) = parse_expr_list(tokens,tpos,"}")
      expr = expr + ["{"] + e + ["}"]
      tpos+=1
  
    else:
      expr.append(tokens[tpos])
      tpos+=1

  raise ValueError()
     
def match(pattern,ppos,tokens,tpos,env,depth):

  #print("\npattern:"); print_token_list(pattern[ppos:],sys.stdout)
  #print("\ntokens:"); print_token_list(tokens[tpos:],sys.stdout)
  
  if is_delimiter_token(tokens[tpos]):
    tpos+=1
    return (ppos,tpos,depth,True)

  elif ispc_mode and tokens[tpos] == "uniform":
    tpos+=1
    return (ppos,tpos,depth,True)
  elif ispc_mode and tokens[tpos] == "varying":
    tpos+=1
    return (ppos,tpos,depth,True)
    
  elif pattern[ppos] == "ID":
    ppos+=1
    var = pattern[ppos]
    if (not is_identifier_token(tokens[tpos])):
      return (ppos,tpos,depth,False)
    b = True
    if var in env:
      b = env[var] == [tokens[tpos]]
    if b:
      ppos+=1
      env[var] = [tokens[tpos]]
      tpos+=1
    return (ppos,tpos,depth,True)

  elif pattern[ppos] == "LHS":
    ppos+=1
    var = pattern[ppos]
    if (not is_identifier_token(tokens[tpos])):
      return (ppos,tpos,depth,False)
    lhs = [tokens[tpos]]
    tpos+=1
    ppos+=1
    while is_identifier_token(tokens[tpos]) or tokens[tpos] == "." or tokens[tpos] == "->":
      lhs.append(tokens[tpos])
      tpos+=1
        
    env[var] = lhs
    return (ppos,tpos,depth,True)

  elif pattern[ppos] == "REGEXPR":
    ppos+=1
    name = pattern[ppos]
    ppos+=1
    pat = pattern[ppos]
    ppos+=1
    next = pattern[ppos]
    try: (expr,tpos) = parse_expr(tokens,tpos,next)
    except ValueError: return (ppos,tpos,depth,False)
    m = "".join(filter(no_delimiter_token,expr))
    if (re.match(pat,m) == None):
      return (ppos,tpos,depth,False)
    env[name] = [m]
    return (ppos,tpos,depth,True)
    
  elif pattern[ppos] == "EXPR":
    ppos+=1
    var = pattern[ppos]
    ppos+=1
    next = pattern[ppos]
    try: (expr,tpos) = parse_expr(tokens,tpos,next)
    except ValueError: return (ppos,tpos,depth,False)
    if (tokens[tpos] != next):
      return (ppos,tpos,depth,False)

    b = True
    if var in env:
      b = filter(no_delimiter_token,env[var]) == filter(no_delimiter_token,expr)

    if (b):
      tpos+=1
      ppos+=1
      env[var] = expr
      
    return (ppos,tpos,depth,b)

  elif pattern[ppos] == tokens[tpos]:

    if tokens[tpos] == "{": depth = depth+1
    if tokens[tpos] == "}": depth = depth-1

    # treat unsigned same as unsigned int
    if tokens[tpos] == "unsigned":
      if tpos+2 < len(tokens) and tokens[tpos+2] == "int":
        tpos+=2
      if ppos+1 < len(pattern) and pattern[ppos+1] ==  "int":
        ppos+=1
    
    ppos+=1
    tpos+=1
    return (ppos,tpos,depth,True)

  else:
    return (ppos,tpos,depth,False)

def substitute (env,tokens,ident):
  global unique_id
  result = []
  i = 0
  while i<len(tokens):
    if tokens[i] == "VAR":
      var = tokens[i+2]
      env[var] = [var+"_"+str(unique_id)]
      unique_id+=1
    elif tokens[i] == "COMMENT":
      result.append("//")

    elif ispc_mode and tokens[i] == "size_t":
      result.append("uintptr_t")
      
    elif not ispc_mode and tokens[i] == "uniform":
      i+=1 # also skip next space
      
    elif not ispc_mode and tokens[i] == "varying":
      i+=1 # also skip next space

    elif not ispc_mode and tokens[i] == "unmasked":
      i+=1 # also skip next space
    
    elif tokens[i] in env:
      result = result + env[tokens[i]]
    else:
      result.append(tokens[i])
      
    if tokens[i] == "\n" and ident != 0:
      result.append(" "*ident)

    i+=1
  return result

def print_token_list(list,f):
  for c in list:
    f.write(c)
      
def match_rule (pattern, tokens, tpos, env, depth):
  tpos_in = tpos
  depth_in = depth
  ppos = 0
  while (tpos < len(tokens) and ppos < len(pattern)):
    (ppos,tpos,depth,m) = match(pattern,ppos,tokens,tpos,env,depth)
    if (not m): break
  if ppos < len(pattern):
    return (False,tpos_in,depth_in)
  else:
    return (True,tpos,depth)

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
  tpos = 0
  while tpos < len(tokens):
    if is_delimiter_token(tokens[tpos]):
      ident = update_delimiter_ident(tokens[tpos],ident);
      result.append(tokens[tpos])
      tpos+=1
    else:
      env = dict(env_in)
      (b,tpos,depth) = match_rule (pattern,tokens,tpos,env,depth)
      if (b):
        result = result + substitute (env,subst,ident)
        for follow_rule in follow_rules:
          tokens = tokens[0:tpos] + apply_rule(follow_rule,env,tokens[tpos:])
      else:
        if tokens[tpos] == "{": depth = depth+1
        if tokens[tpos] == "}": depth = depth-1
        if (depth < 0):
          result = result + tokens[tpos:]
          return result
        result.append(tokens[tpos])
        tpos+=1
  return result

def printUsage():
  sys.stdout.write("Usage: cpp-patch.py [--ispc] --patch embree2_to_embree3.patch --in infile.cpp [--out outfile.cpp]\n")

def parseCommandLine(argv):
  global rule_file
  global cpp_file_in
  global cpp_file_out
  global ispc_mode
  if len(argv) == 0:
    return;
  elif len(argv)>=1 and argv[0] == "--ispc":
    ispc_mode = True
    parseCommandLine(argv[1:len(argv)])
  elif len(argv)>=2 and argv[0] == "--patch":
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
  printUsage()
  raise ValueError("no patch file specified");
if (cpp_file_in == ""):
  printUsage()
  raise ValueError("no input file specified");

line = 1
def parse_rules_file(rule_file):

  def pop_line(lines):
    global line
    line+=1
    return lines.pop(0)

  def empty_line(str):
    return all(map(is_delimiter,str))
  
  def parse_rule(lines):
    pattern_str = ""
    subst_str = ""
    next_rules = []

    while lines and not lines[0].startswith("=>") and not lines[0].startswith("}@@") and not lines[0].startswith("@@{"):
      pattern_str += pop_line(lines)
      
    if lines[0].startswith("=>"):
      pop_line(lines)
      while lines and not lines[0].startswith("=>") and not lines[0].startswith("}@@") and not lines[0].startswith("@@{"):
        subst_str += pop_line(lines)

    pattern_str = pattern_str.strip('\n')
    subst_str = subst_str.strip('\n')
    
    while lines[0].startswith("@@{"):
      l = pop_line(lines)
      rule = parse_rule(lines)
      if (l.startswith("@@{cpp")):
        if (not ispc_mode): next_rules.append(rule)
      elif (l.startswith("@@{ispc")):
        if (ispc_mode): next_rules.append(rule)
      else:
        next_rules.append(rule)

    if lines[0].startswith("}@@"):
      pop_line(lines)
      pattern = filter(no_delimiter_token,tokenize(list(pattern_str),True))
      subst = tokenize(list(subst_str),False)
      return (pattern,subst,next_rules)

    raise ValueError(rule_file+" line " + str(line) + ": parse error")
  
  with open(rule_file,"r") as f:
    lines = f.readlines()
  
  rules = []
  while lines:
    if (lines[0].startswith("@@{")):
      l = pop_line(lines)
      rule = parse_rule(lines)
      if (l.startswith("@@{cpp")):
        if (not ispc_mode): rules.append(rule)
      elif (l.startswith("@@{ispc")):
        if(ispc_mode): rules.append(rule)
      else:
        rules.append(rule)
    elif (lines[0].startswith("@@END")):
      return rules
    elif (lines[0].startswith("//")):
      pop_line(lines)
    elif empty_line(lines[0]):
      pop_line(lines)
    else:
      raise ValueError(rule_file+" line " + str(line) + ": parse error")
    
  return rules

rules = parse_rules_file(rule_file)
#rules = [rules[65]]
with open(cpp_file_in,"r") as f:
  tokens = tokenize(list(f.read()),False)

rule_id = 0
for rule in rules:
#  print("applying rule "+str(rule_id)+" of "+str(len(rules))+" rules")
  env = {}
  tokens = apply_rule(rule,env,tokens)
  rule_id+=1
#  print rule
#  print_token_list(tokens,sys.stdout)

if (cpp_file_out == ""):
  print_token_list(tokens,sys.stdout)
else:
  with open(cpp_file_out,"w") as f:
    print_token_list(tokens,f)

  
