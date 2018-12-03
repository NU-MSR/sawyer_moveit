from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Pose
import tf.transformations as tr
import numpy as np
import copy

# our goal here is to just build a list of consecutive poses that we'll plan to
# have the EE pass through.


# let's plan for a small circle that holds the EE orientation constant:

# circle period:
T = 5
def circle_point(t):
    # circle center:
    P0 = np.array([0.45, 0.49, 0.365])
    # radius:
    R = 0.2
    # we'll move in the X-Z axis:
    pt_x = R*np.cos(2*np.pi*t/T)
    pt_z = R*np.sin(2*np.pi*t/T)
    pout = P0 + np.array([pt_x, 0, pt_z])
    return pout


# let's build a default orientation that we will use for all points:
q_arr = tr.quaternion_from_euler(-np.pi/2., 0, 0, 'sxyz')
quat = Quaternion(*q_arr)

# iterate:
tvec = np.linspace(0, 5, 30)
targets = []
for t in tvec:
    p = Pose()
    p.position = Point(*circle_point(t))
    p.orientation = copy.deepcopy(quat)
    targets.append(p)



