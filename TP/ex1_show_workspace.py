from delta_utils import *

plot_box()
#Show working space 
for q0 in np.linspace(-pi, pi  ,30):
  for q1 in np.linspace(-pi, pi  ,30):
    q=np.array([q0,q1])
    plot_delta(np.array(q),line="o")
plt.show()
