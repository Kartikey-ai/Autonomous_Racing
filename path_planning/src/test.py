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

'''
2
ans1 (3+0j)
ans2 (3+0j)
Traceback (most recent call last):
  File "/home/kartikey/catkin_ws/src/EUFS_SIM/path_planning/src/test.py", line 27, in <module>
    print(ans1.type)
AttributeError: 'complex' object has no attribute 'type'
kartikey@kartikey-ROG:~$ 


ans1 (3+0j)
ans2 (3+0j)
[1. 1.]
float64
4 in li
kartikey@kartikey-ROG:~$ /home/kartikey/catkin_ws/src/EUFS_SIM/path_planning/src/test.py
  File "/home/kartikey/catkin_ws/src/EUFS_SIM/path_planning/src/test.py", line 37
    print(lil[0])]
    ^
SyntaxError: invalid syntax
kartikey@kartikey-ROG:~$ 

  File "/home/kartikey/catkin_ws/src/EUFS_SIM/path_planning/src/test.py", line 37
    print(lil[0])]
                 ^
SyntaxError: unmatched ']'
kartikey@kartikey-ROG:~$ 

kartikey@kartikey-ROG:~$ /home/kartikey/catkin_ws/src/EUFS_SIM/path_planning/src/test.py
11.0
[(2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (21.0, 1.5), (26.0, 4.25), (32.0, 2.8), (34.0, 2.0), (2.0, 0.0), (4.0, 0.0), (6.0, 0.0), (8.0, 0.0), (10.0, 0.1), (13.0, 0.1), (15.0, 0.2), (17.0, 0.3), (18.0, 0.5), (21.0, 1.5), (26.0, 4.25), (32.0, 2.8), (34.0, 2.0)]
[3, 5, 6, 3, 5, 6]
2
ans1 (3+0j)
ans2 (3+0j)
[1. 1.]
float64
4 in li
(1, 2)
kartikey@kartikey-ROG:~$ 
'''