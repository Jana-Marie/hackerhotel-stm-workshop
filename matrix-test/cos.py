#!/usr/bin/env python3

import math

S = 12
def FP(x):
    return int(x*math.pow(2, S))

k = 10
N = int(math.pow(2, k))
print("const int16_t cos_k = {:d};".format(k))
print("int16_t cos_table[{:d}]={{".format(N))
for i in range(0, N):
    theta = (math.pi/2) * i / (N-1)
    cs = FP(math.cos(theta))
    sn = FP(math.sin(theta))
    print("  {:d},".format(cs))
print("};")
