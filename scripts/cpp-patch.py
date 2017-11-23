#!/usr/bin/python

import sys
import os
import copy

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
  return token[0] in delimiter_chars or token.startswith("/*") or token.startswith("//")

def no_delimiter_token (token):
  return not is_delimiter_token (token)

def is_identifier_token (token):
  return token[0] in identifier_chars

def parse_expr_list(tokens,tpos,term_token):
  expr = []
  while (tpos < len(tokens)):
    
    if tokens[tpos] == term_token:
      return expr
    
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
     
def match(pattern,ppos,tokens,tpos,env):

  if is_delimiter_token(tokens[tpos]):
    tpos+=1
    return (ppos,tpos,True)
    
  elif pattern[ppos] == "ID":
    ppos+=1
    var = pattern[ppos]
    ppos+=1
    if (not is_identifier_token(tokens[tpos])):
      return (ppos,tpos,False)
    env[var] = [tokens[tpos]]
    tpos+=1
    return (ppos,tpos,True)
    
  elif pattern[ppos] == "EXPR":
    ppos+=1
    var = pattern[ppos]
    ppos+=1
    next = pattern[ppos]
    try: (expr,tpos) = parse_expr(tokens,tpos,next)
    except ValueError: return (ppos,tpos,False)
    if (tokens[tpos] != next):
      return (ppos,tpos,False)
    tpos+=1
    ppos+=1
    if var in env:
      b = filter(no_delimiter_token,env[var]) == filter(no_delimiter_token,expr)
      return (ppos,tpos,b)
    else:
      env[var] = expr
      return (ppos,tpos,True)
    
  elif pattern[ppos] == tokens[tpos]:
    ppos+=1
    tpos+=1
    return (ppos,tpos,True)

  else:
    return (ppos,tpos,False)

def substitute (env,tokens,ident):
  result = []
  for token in tokens:
    if token in env:
      result = result + env[token]
    else:
      result.append(token)
    if token == "\n" and ident != 0:
      result.append(" "*ident)
  return result

def print_token_list(list,f):
  for c in list:
    f.write(c)
      
def match_rule (pattern, tokens, tpos, env):
  tpos_in = tpos
  ppos = 0
  while (tpos < len(tokens) and ppos < len(pattern)):
    (ppos,tpos,m) = match(pattern,ppos,tokens,tpos,env)
    if (not m): break
  if ppos < len(pattern):
    return (False,tpos_in)
  else:
    return (True,tpos)

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
      env = env_in
      (b,tpos) = match_rule (pattern,tokens,tpos,env)
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

def tokenize_rule (rule):
  if rule == (): return rule
  (pattern_str, subst_str, next_rule) = rule
  pattern = filter(no_delimiter_token,tokenize(list(pattern_str)))
  subst = tokenize(list(subst_str))
  return (pattern,subst,map (tokenize_rule,next_rule))

rule_file = ""
cpp_file_in = ""
cpp_file_out = ""

def printUsage():
  sys.stdout.write("Usage: cpp-patch.py --patch embree2_to_embree3.patch --in infile.cpp --out outfile.cpp\n")

def parseCommandLine(argv):
  global rule_file
  global cpp_file_in
  global cpp_file_out
  if len(argv) == 0:
    return;
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
  raise ValueError("no rule file specified");
if (cpp_file_in == ""):
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
      pop_line(lines)
      next_rules.append(parse_rule(lines))

    if lines[0].startswith("}@@"):
      pop_line(lines)
      pattern = filter(no_delimiter_token,tokenize(list(pattern_str)))
      subst = tokenize(list(subst_str))
      return (pattern,subst,next_rules)

    raise ValueError(rule_file+" line " + str(line) + ": parse error")
  
  with open(rule_file,"r") as f:
    lines = f.readlines()
  
  rules = []
  while lines:
    if (lines[0].startswith("@@{")):
      pop_line(lines)
      rules.append(parse_rule(lines))
    elif (lines[0].startswith("//")):
      pop_line(lines)
    elif empty_line(lines[0]):
      pop_line(lines)
    else:
      raise ValueError(rule_file+" line " + str(line) + ": parse error")
    
  return rules

rules = parse_rules_file(rule_file)
with open(cpp_file_in,"r") as f:
  tokens = tokenize(list(f.read()))

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

  
