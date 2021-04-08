#!/usr/bin/env python3

## Copyright 2009-2021 Intel Corporation
## SPDX-License-Identifier: Apache-2.0

import sympy as sp
import numpy as np
import math


################################################################################
#### Utils
################################################################################

def getTerms(map, key):
    if key in map.keys():
        return map[key]
    return 0

# simple linear interpolation wrapper
def lerp(v0,v1,t):
    return v0*(1-t)+v1*t

# custom quaternion to matrix conversion
def to_rotation_matrix(q):
    return sp.Matrix([[q.a*q.a + q.b*q.b - q.c*q.c - q.d*q.d, 2*(q.b*q.c - q.a*q.d),                 2*(q.b*q.d + q.a*q.c),                 0],
                      [2*(q.b*q.c + q.a*q.d),                 q.a*q.a - q.b*q.b + q.c*q.c - q.d*q.d, 2*(q.c*q.d - q.a*q.b),                 0],
                      [2*(q.b*q.d - q.a*q.c),                 2*(q.c*q.d + q.a*q.b),                 q.a*q.a - q.b*q.b - q.c*q.c + q.d*q.d, 0],
                      [0,                                     0,                                     0,                                     1]])


################################################################################
#### Set up symbolic objects
################################################################################

t, theta = sp.symbols("t, theta", real = True)

px0, py0, pz0 = sp.symbols("px0, py0, pz0", real=True) # vertex position at t=0
px1, py1, pz1 = sp.symbols("px1, py1, pz1", real=True) # vertex position at t=1

tx0, ty0, tz0 = sp.symbols("tx0, ty0, tz0", real=True) # translation at t=0
tx1, ty1, tz1 = sp.symbols("tx1, ty1, tz1", real=True) # translation at t=1

qx0, qy0, qz0, qw0 = sp.symbols("qx0, qy0, qz0, qw0", real=True) # quaternion at t=0 
qx1, qy1, qz1, qw1 = sp.symbols("qx1, qy1, qz1, qw1", real=True) # quaternion at t=1

# coefficients for upper triangular matrices
s000, s001, s002, s003, s011, s012, s013, s022, s023 = sp.symbols("s000, s001, s002, s003, s011, s012, s013, s022, s023", real=True)
s100, s101, s102, s103, s111, s112, s113, s122, s123 = sp.symbols("s100, s101, s102, s103, s111, s112, s113, s122, s123", real=True)

q0 = sp.Quaternion(qw0, qx0, qy0, qz0)
q1 = sp.Quaternion(qw1, qx1, qy1, qz1)

# assuming that q1 is qperp = normalize(q1-q0*cosTheta), where cosTheta=dot(q0, q1) and theta = acos(cosTheta).
# this simplifies the terms of the symbolic expressions later
qt = q0 * sp.cos(t*theta) + q1 * sp.sin(t*theta)

S0 = sp.Matrix([[s000, s001, s002, s003],
                [   0, s011, s012, s013],
                [   0,    0, s022, s023],
                [   0,    0,    0,    1]])
S1 = sp.Matrix([[s100, s101, s102, s103],
                [   0, s111, s112, s113],
                [   0,    0, s122, s123],
                [   0,    0,    0,    1]])
D0 = sp.Matrix([[1, 0, 0, tx0],
                [0, 1, 0, ty0],
                [0, 0, 1, tz0],
                [0, 0, 0,  1]])
D1 = sp.Matrix([[1, 0, 0, tx1],
                [0, 1, 0, ty1],
                [0, 0, 1, tz1],
                [0, 0, 0,  1]])
p0 = sp.Matrix([px0, py0, pz0, 1])
p1 = sp.Matrix([px1, py1, pz1, 1])

Gamma = lerp(D0, D1, t)*to_rotation_matrix(qt)*lerp(S0, S1, t)*lerp(p0, p1, t)
C = sp.Matrix(np.empty(8))   # 8 coefficients
K = sp.Matrix(np.empty(7))   # 7 inputs
A = sp.Matrix(np.empty(8*7*3)) # 8 coefficients, 7 inputs (1, px0, py0, pz0, px1, py1, pz1), 3 dimensions (x, y, z)
dGamma = sp.diff(Gamma, t)


################################################################################
#### Group the coefficients (this might time a couple of seconds)
################################################################################

# loop over dimensions (x, y, z)
for dim in range(3):
    dm = sp.expand(dGamma[dim])
    dm = dm.subs(sp.sin(t*theta)*sp.sin(t*theta),(1-sp.cos(2*t*theta))/2) # remove sin(t*theta)^2
    dm = dm.subs(sp.cos(t*theta)*sp.cos(t*theta),(1+sp.cos(2*t*theta))/2) # remove cos(t*theta)^2
    dm = dm.subs(sp.sin(t*theta)*sp.cos(t*theta),sp.sin(2*t*theta)/2)     # remove sin(t*theta)*cos(t*theta)
    dm = sp.expand(dm)

    # group all terms in the form a + b * cos(2*t*theta) + c * sin(2*t*theta)
    dm_cos_sin = sp.collect(dm, (sp.cos(2*t*theta), sp.sin(2*t*theta)), evaluate=False)

    # get the terms
    coeff_cos   = getTerms(dm_cos_sin, sp.cos(2*t*theta))
    coeff_sin   = getTerms(dm_cos_sin, sp.sin(2*t*theta))
    coeff_const = getTerms(dm_cos_sin, 1)

    # group the term in the form a + b * t 
    coeff_const_t = sp.collect(coeff_const, t, evaluate=False)
    C[0] = getTerms(coeff_const_t, 1)
    C[1] = getTerms(coeff_const_t, t)

    # group the term in the form a + b * t + c * t^2 
    coeff_cos_t = sp.collect(coeff_cos, t, evaluate=False)
    C[2] = getTerms(coeff_cos_t, 1)
    C[3] = getTerms(coeff_cos_t, t)
    C[4] = getTerms(coeff_cos_t, t*t)

    # group the term in the form a + b * t + c * t^2 
    coeff_sin_t = sp.collect(coeff_sin, t, evaluate=False)
    C[5] = getTerms(coeff_sin_t, 1)
    C[6] = getTerms(coeff_sin_t, t)
    C[7] = getTerms(coeff_sin_t, t*t)

    for c in range(8):
        kc = sp.collect(C[c], (px0, py0, pz0, px1, py1, pz1), evaluate=False)
        K[0] = getTerms(kc, 1)
        K[1] = getTerms(kc, px0)
        K[2] = getTerms(kc, py0)
        K[3] = getTerms(kc, pz0)
        K[4] = getTerms(kc, px1)
        K[5] = getTerms(kc, py1)
        K[6] = getTerms(kc, pz1)

        for k in range(7):
            K[k] = sp.expand(K[k])
            K[k] = K[k].subs(qw0*qw0, 1-qx0*qx0-qy0*qy0-qz0*qz0) # clean up substitutions
            K[k] = K[k].subs(qw1*qw1, 1-qx1*qx1-qy1*qy1-qz1*qz1) # clean up substitutions
            K[k] = sp.simplify(K[k])
            A[8*7*dim + c*7 + k] = K[k]


################################################################################
#### Write code to file
################################################################################

from sympy.utilities.codegen import codegen, default_datatypes
from sympy.codegen.ast import real, float32
from sympy.printing.ccode import C99CodePrinter
printer = C99CodePrinter()

# custom code printer that will not generate such nonesene as x^2 -> pow(x, 2)
class CustomCodePrinter(C99CodePrinter):
    def _print_Pow(self, expr):
        if expr.exp.is_integer and expr.exp > 0 and expr.exp < 5:
            return '*'.join([self._print(expr.base) for i in range(expr.exp)])
        else:
            return super()._print_Pow(expr)

customprinter = CustomCodePrinter()
customprinter.type_aliases[real] = float32 # cosf instead of cos
default_datatypes["float"].cname = "float" # float instead of double
params = [
    theta,
    tx0, ty0, tz0,
    tx1, ty1, tz1,
    qw0, qx0, qy0, qz0,
    qw1, qx1, qy1, qz1,
    s000, s001, s002, s003, s011, s012, s013, s022, s023,
    s100, s101, s102, s103, s111, s112, s113, s122, s123]
R = sp.MatrixSymbol("coeff", A.shape[0], A.shape[1])
P = sp.MatrixSymbol('p', len(params), 1)
param_map = dict(zip(params, P))
B = A.xreplace(param_map)
codegen(('motion_derivative_coefficients', sp.Eq(R,B)), language='c', printer=customprinter, prefix='motion_derivative_coefficients', to_files=True)
