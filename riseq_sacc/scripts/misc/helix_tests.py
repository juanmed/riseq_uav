import numpy as np
import matplotlib.pyplot as plt

tmax = 10
tstep = 0.01
t = np.arange(0,tmax,tstep)

# ladder position
lx = t
ly = t

r = 2

theta = 2*t

# drone position
dx = r*np.cos(theta) + lx
dy = r*np.sin(theta) + ly

fig = plt.figure()
ax = fig.add_subplot(1,1,1)
ax.plot(dx,dy, color = 'r')
ax.plot(lx,ly, color = 'b')


plt.show()