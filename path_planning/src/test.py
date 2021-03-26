#! /usr/bin/env python3

import cmath
import numpy as np
x = [1.0, 2.0]
y = [5.0, 3.0]
print(np.dot(x, y))

path = [(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (21.0, 1.5), (26.0, 4.25), (32.0, 2.8), (34.0, 2.0)] # (20.0, 1.0),
list2 = []
list2 = path * 2
print(list2)
ka = [3, 5, 6]
kaa = []
print(ka*2)

k = ka.index(6)
print(k)

coeff =[1, -2, 1]
a,b,c = (1, -6, 9)
dis = (b**2) - (4*a*c)
ans1 = (-b -cmath.sqrt(dis))/ (2*a)
ans2 = (-b +cmath.sqrt(dis))/ (2*a)
print("ans1", ans1)
print("ans2", ans2)
# print(ans1.type)
lala = np.roots(coeff)
print(lala)
print(lala[0].dtype)
tru = True
li = [4, 5, 8, 9]
if tru and (4 in li):
  print("4 in li")

lil = [(1,2), (4,5)]
print(lil[0])
node = lil[0]
print(node[0])

print("---------------------------------end -------------------------------")
x_axisd = [3.0, 1.0, 0.0]
y_axisd = [1.0, 0.0, 0.0]
cross = np.cross(x_axisd, y_axisd) 
print("cross", cross)