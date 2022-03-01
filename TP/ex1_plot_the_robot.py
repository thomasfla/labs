from delta_utils import *
plot_box()
q = np.array([pi/2,pi/2]) #Joint configuration

#plot the robot in configuration q:
plot_delta(q)
plt.show()

#plot 

#plot the robot in two configurations:
plot_box()
plot_delta(q)
q = np.array([2.0,0.5]) 
plot_delta(q)
plt.show()

#plot the robot and a 2d point:
q = np.array([2.0,0.5])
p = np.array([0.05,0.10]) # x-y point
plot_box()
plot_delta(q)
plt.plot(p[0],p[1],"x")
plt.show()
