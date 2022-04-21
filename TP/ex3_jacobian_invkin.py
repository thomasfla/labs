from delta_utils import *

plot_box()    
q = np.array([pi/2,pi/2])
plot_delta(q)
p_goal = np.array([0.01,0.1])
plt.plot(p_goal[0],p_goal[1],"x",markersize=20)
for i in range(100):
  p=fk_delta(q)
  dq = 1e-1 * inv(J(q))@(p_goal-p)
  q += dq
  print(f"q={q}")
  print(f"p={p}")
  plot_delta(q)
print(f"p={p}")
plt.show()
embed()


