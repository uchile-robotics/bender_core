import tf
from math import pi

roll = 0
pitch = -pi/2
yaw = 0

q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
print '[{:.3f}, {:.3f}, {:.3f}, , {:.3f}]'.format(q[0],q[1],q[2],q[3])