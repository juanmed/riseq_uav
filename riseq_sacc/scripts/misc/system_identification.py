import control as ctl
#print(ctl)
import numpy as np
import matplotlib.pyplot as plt

fig = plt.figure(figsize=(20,10))
fig.suptitle("Angular velocity dynamics modelling")
ax0 = fig.add_subplot(2,1,1)
ax1 = fig.add_subplot(2,1,2)


# define simulation time
t_max = 10
dt = 0.01
t = np.arange(0.0,t_max,dt)


# define step input
refx =  np.ones_like(t)

# px4 xpos dynamics
ts = 5	# settling time
tr = 2.5 	# rise time
mp = 0.99	# overshoot

sigma = 4.6/ts
wn = 1.8/tr
etta = 0.75

# define 2nd order system
num = np.array([wn**2])
den = np.array([1.0, 2*etta*wn, wn**2])
wx_tf = ctl.tf(num,den)

print(" Transfer Function \n{}".format(wx_tf))
print(" State Space: \n{}".format(ctl.tf2ss(num,den)))
# evaluate step response
tx, wx, s = ctl.forced_response(wx_tf, T=t, U=refx)

ax0.plot(t, wx,linestyle = '-',color ='r', label = "wx")
ax0.plot(t, refx,linestyle = '--', color = "k", label = 'x ref')
ax0.set_title("Wx dynamics", fontsize='small')
ax0.legend(loc='center right', shadow=True, fontsize='small')
ax0.set_xlabel("time {s}")
ax0.set_ylabel("wx {rad/s}")
ax0.set_xticks(np.arange(0,10,0.5))

plt.show()


"""
xpos
        0.4418
----------------------
s^2 + 0.997 s + 0.4418

 State Space: 
A = [[-0.99704579 -0.44182236]
 [ 1.          0.        ]]

B = [[-1.]
 [ 0.]]

C = [[ 0.         -0.44182236]]

D = [[0.]]

Translation dynamics PP K: 
[[ 5.003  17.5582]], Nu: 
[[1.]], Nx: 
[[0.        ]
 [2.26346763]]




ypos
        0.5184
---------------------
s^2 + 1.08 s + 0.5184

 State Space: 
A = [[-1.08   -0.5184]
 [ 1.      0.    ]]

B = [[-1.]
 [ 0.]]

C = [[ 0.     -0.5184]]

D = [[0.]]


Translation dynamics PP K: 
[[ 7.08   18.5184]], Nu: 
[[-1.]], Nx: 
[[0.        ]
 [1.92901235]]



"""

