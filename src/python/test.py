import numpy as np 
import cv2

a = [1, 2]
b = [3, 4, 4]
c = [5, 6]
d = []
d.append([1, 2])
d.append([3, 4])
d.append([5, 6])
e = np.concatenate((a, b), axis = 0)
print(d)
print(d[-1][1])
print(e)
print(d[1][0])
mean = np.mean([d[0:3][0]])
print(mean)
mean = np.mean(np.array(d)[:,1])
print(mean)