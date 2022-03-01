from delta_utils import *

q = np.array([2.0,0.5])
print (f" q ={q}")
p=fk_delta(q)
print (f" p = FK(q) = {p}")
q1=ik_delta(p)
print (f" q1=IK(p) = {q1}")

plot_box()
plot_delta(q)  
plt.plot(p[0],p[1],"x",markersize=20)
plot_delta(q1)  
plt.show()
embed()
