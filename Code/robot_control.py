from visualization import ArmPlayer
from robot_arm import *

VISUAL_SCALE_FACTOR = 10

a_len = 0.1395 * VISUAL_SCALE_FACTOR
b_len = 0.01995 * VISUAL_SCALE_FACTOR
c_len = 0.06915 * VISUAL_SCALE_FACTOR
d_len = 0.06925 * VISUAL_SCALE_FACTOR
e_len = 0.1372 * VISUAL_SCALE_FACTOR
dh_parameters = [[np.pi/2.0, a_len, b_len, np.pi/2.0],
                 [0, 0, c_len, 0],
                 [0, 0, d_len, 0],
                 [0, 0, e_len, 0]]

# joints = ['r', 'p', 'r', 'p']

joint_limits = [[-180, 180],
                [-70, 90],
                [-90, 90],
                [-90, 90]]

arm = RobotArm(dh_parameters, joint_limits=joint_limits)
ArmPlayer(arm)