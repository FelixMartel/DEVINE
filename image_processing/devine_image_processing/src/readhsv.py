

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


import re


f = open('HSVDump.txt', 'r')

data = f.read()

values = re.findall('\[(\d+) (\d+) (\d+)\]', data) 


hues = []
saturations = []
lightness = []

for value in values:
    [h, s, v] = value
    hues.append(int(h))
    saturations.append(int(s))
    lightness.append(int(v))





#print(images.flatten().shape)

fig = plt.figure()
#ax = fig.add_subplot(111, projection='3d')


#ax.scatter(hues, saturations, lightness) #, zdir='z', s=20, c=None, depthshade=True, *args, **kwargs)

plt.scatter(hues, saturations)#, extent=(x.min(), x.max(), y.max(), y.min()), interpolation='nearest', cmap=cm.gist_rainbow)


fig.show()

raw_input('Press Enter to exit')