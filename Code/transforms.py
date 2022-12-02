"""
Transforms Module - Contains code for to learn about rotations
and eventually homogenous transforms. 

Empty outline derived from code written by John Morrell. 
"""

import numpy as np
from numpy import sin, cos, sqrt
from numpy.linalg import inv, norm
import sympy as sp

## 2D Rotations
def rot2(th):
    """
    R = rot2(theta)
    Parameters
        theta: float or int, angle of rotation
    Returns
        R: 2 x 2 numpy array representing rotation in 2D by theta
    """

    ## TODO - Fill this out
    R = np.array([[cos(th), -sin(th)],
                 [sin(th), cos(th)]])
    return R

## 3D Transformations
def rotx(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about x-axis by amount theta
    """
    ## TODO - Fill this out
    R = np.array([[1, 0, 0],
                  [0, cos(th), -sin(th)],
                  [0, sin(th), cos(th)]])

    return R

def roty(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about y-axis by amount theta
    """
    ## TODO - Fill this out
    R = np.array([[cos(th), 0, sin(th)],
                  [0, 1, 0],
                  [-sin(th), 0, cos(th)]])

    return R

def rotz(th):
    """
    R = rotx(th)
    Parameters
        th: float or int, angle of rotation
    Returns
        R: 3 x 3 numpy array representing rotation about z-axis by amount theta
    """

    ## TODO - Fill this out
    R = np.array([[cos(th), -sin(th), 0],
                  [sin(th), cos(th), 0],
                  [0, 0, 1]])

    return R

# Symbolic 3D transformations
def rotx_symbolic(th):
    R = sp.Matrix([ [1, 0, 0],
                [0, sp.cos(th), -sp.sin(th)],
                [0, sp.sin(th), sp.cos(th)]])
    return R

def roty_symbolic(th):
    R = sp.Matrix([ [sp.cos(th), 0, sp.sin(th)],
                    [0, 1, 0],
                    [-sp.sin(th), 0, sp.cos(th)]])
    return R

def rotz_symbolic(th):
    R = sp.Matrix([ [sp.cos(th), -sp.sin(th), 0],
                    [sp.sin(th), sp.cos(th), 0],
                    [0, 0, 1]])
    return R


# inverse of rotation matrix 
def rot_inv(R):
    '''
    R = rot_inv(R)
    Parameters
        R: 2x2 or 3x3 numpy array representing a proper rotation matrix
    Returns
        R: 2x2 or 3x3 inverse of the input rotation matrix
    '''
    ## TODO - Fill this out
    R_inv = np.transpose(R)
    return R_inv

def se3(R=np.eye(3), p=np.array([0, 0, 0])):
    """
        T = se3(R, p)
        Description:
            Given a numpy 3x3 array for R, and a 1x3 or 3x1 array for p, 
            this function constructs a 4x4 homogeneous transformation 
            matrix "T". 

        Parameters:
        R - 3x3 numpy array representing orientation, defaults to identity
        p = 3x1 numpy array representing position, defaults to [0, 0, 0]

        Returns:
        T - 4x4 numpy array
    """
    # TODO - fill out "T"
    # print(f"R: {R}")
    # print(f"p: {p}")
    top_rows = np.column_stack((R, np.transpose(p)))
    bottom_row = np.array([0, 0, 0, 1])
    T = np.vstack((top_rows, bottom_row))

    return T

def se3_symbolic(R=np.eye(3), p=np.array([0, 0, 0])):
    R = sp.Matrix(R)
    translation = sp.Matrix(p)
    # print(f"R ({R.shape}):")
    # sp.pprint(R)
    # print(f"translation ({translation.shape}):")
    # sp.pprint(translation)
    top_rows = sp.Matrix.hstack(R, translation)
    bottom_row = sp.Matrix([0, 0, 0, 1]).T
    # print(f"top_rows ({top_rows.shape}):")
    # sp.pprint(top_rows)
    # print(f"bottom_row ({bottom_row.shape}):")
    # sp.pprint(bottom_row)
    T = sp.Matrix.vstack(top_rows, bottom_row)

    return T

# a1 = sp.symbols("a1")
# a2 = sp.symbols("a2")
# my_matrix = se3_symbolic(rotx_symbolic("theta"), [a1, 0, 0])
# print("my_matrix:")
# sp.pprint(my_matrix)

def inv(T):
    """
        Tinv = inv(T)
        Description:
        Returns the inverse transform to T

        Parameters:
        T

        Returns:
        Tinv - 4x4 numpy array that is the inverse to T so that T @ Tinv = I
    """
    
    #TODO - fill this out 
    R = T[:3,:3]
    p = T[:3,3:]
    # print(f"R: {R}")
    # print(f"p: {p}")

    R_inv = np.transpose(R)
    p_inv = -R_inv @ p
    # print(f"R_inv: {R_inv}")
    # print(f"p_inv: {p_inv}")

    top_rows = np.column_stack((R_inv, p_inv))
    bottom_row = np.array([0, 0, 0, 1])
    T_inv = np.vstack((top_rows, bottom_row))

    return T_inv

#SO(3) conversion code to convert between different SO(3) representations.

def R2rpy(R):
    """
    rpy = R2rpy(R)
    Description:
    Returns the roll-pitch-yaw representation of the SO3 rotation matrix

    Parameters:
    R - 3 x 3 Numpy array for any rotation

    Returns:
    rpy - 1 x 3 Numpy Matrix, containing <roll pitch yaw> coordinates (in radians)
    """
    
    # follow formula in book, use functions like "np.atan2" 
    # for the arctangent and "**2" for squared terms. 
    # TODO - fill out this equation for rpy

    roll = np.arctan2(R[1][0], R[0][0]) # About Z axis (phi)
    pitch = np.arctan2(-R[2][0], sqrt(R[2][1]**2 + R[2][2]**2)) # About Y axis (theta)
    yaw = np.arctan2(R[2][1], R[2][2]) # About X axis (psi)

    return np.array([roll, pitch, yaw])

def R2axis(R):
    """
    axis_angle = R2axis(R)
    Description:
    Returns an axis angle representation of a SO(3) rotation matrix

    Parameters:
    R

    Returns:
    axis_angle - 1 x 4 numpy matrix, containing  the axis angle representation
    in the form: <angle, rx, ry, rz>
    """

    # see equation (2.27) and (2.28) on pg. 54, using functions like "np.acos," "np.sin," etc. 
    ang = np.arccos((R[0][0] + R[1][1] + R[2][2] - 1)/2) # TODO - fill out here. 
    coef = 1/(2*np.sin(ang))
    axis_angle = np.array([ang,
                            coef * (R[2][1] - R[1][2]), # TODO - fill out here, each row will be a function of "ang"
                            coef * (R[0][2] - R[2][0]),
                            coef * (R[1][0] - R[0][1])])

    return axis_angle

def axis2R(ang, v):
    """
    R = axis2R(angle, rx, ry, rz, radians=True)
    Description:
    Returns an SO3 object of the rotation specified by the axis-angle

    Parameters:
    angle - float, the angle to rotate about the axis in radians
    v = [rx, ry, rz] - components of the unit axis about which to rotate as 3x1 numpy array
    
    Returns:
    R - 3x3 numpy array
    """
    # TODO fill this out 
    r11 = (v[0]**2)*(1 - cos(ang)) + cos(ang)
    r12 = v[0]*v[1]*(1 - cos(ang)) - v[2]*sin(ang)
    r13 = v[0]*v[2]*(1 - cos(ang)) + v[1]*sin(ang)
    
    r21 = v[0]*v[1]*(1 - cos(ang)) + v[2]*sin(ang)
    r22 = (v[1]**2)*(1 - cos(ang)) + cos(ang)
    r23 = v[1]*v[2]*(1 - cos(ang)) - v[0]*sin(ang)
    
    r31 = v[0]*v[2]*(1 - cos(ang)) - v[1]*sin(ang)
    r32 = v[1]*v[2]*(1 - cos(ang)) + v[0]*sin(ang)
    r33 = (v[2]**2)*(1 - cos(ang)) + cos(ang)

    R = np.array([[r11, r12, r13],
                  [r21, r22, r23],
                  [r31, r32, r33]])
    return R

def R2q(R):
    """
    quaternion = R2q(R)
    Description:
    Returns a quaternion representation of pose

    Parameters:
    R

    Returns:
    quaternion - 1 x 4 numpy matrix, quaternion representation of pose in the 
    format [nu, ex, ey, ez]
    """
    # TODO, see equation (2.34) and (2.35) on pg. 55, using functions like "sp.sqrt," and "sp.sign"

    return np.array([0.5*sqrt(R[0][0] + R[1][1] + R[2][2] + 1),
                     0.5*np.sign(R[2][1] - R[1][2])*sqrt(R[0][0] - R[1][1] - R[2][2] + 1),
                     0.5*np.sign(R[0][2] - R[2][0])*sqrt(R[1][1] - R[2][2] - R[0][0] + 1),
                     0.5*np.sign(R[1][0] - R[0][1])*sqrt(R[2][2] - R[0][0] - R[1][1] + 1)])
  
def q2R(q):
    """
    R = q2R(q)
    Description:
    Returns a 3x3 rotation matrix

    Parameters:
    q - 4x1 numpy array, [nu, ex, ey, ez ] - defining the quaternion
    
    Returns:
    R - a 3x3 numpy array 
    """
    # TODO, extract the entries of q below, and then calculate R
    nu = q[0]
    ex = q[1]
    ey = q[2]
    ez = q[3]

    r11 = 2*(nu**2 + ex**2) - 1
    r12 = 2*(ex*ey - nu*ez)
    r13 = 2*(ex*ez + nu*ey)
    
    r21 = 2*(ex*ey + nu*ez)
    r22 = 2*(nu**2 + ey**2) - 1
    r23 = 2*(ey*ez - nu*ex)
    
    r31 = 2*(ex*ez - nu*ey)
    r32 = 2*(ey*ez + nu*ex)
    r33 = 2*(nu**2 + ez**2) - 1

    R = np.array([[r11, r12, r13],
                  [r21, r22, r23],
                  [r31, r32, r33]])
    return R

def euler2R(th1, th2, th3, order='xyz'):
    """
    R = euler2R(th1, th2, th3, order='xyz')
    Description:
    Returns a 3x3 rotation matrix as specified by the euler angles, we assume in all cases
    that these are defined about the "current axis," which is why there are only 12 versions 
    (instead of the 24 possiblities noted in the course slides). 

    Parameters:
    th1, th2, th3 - float, angles of rotation
    order - string, specifies the euler rotation to use, for example 'xyx', 'zyz', etc.
    
    Returns:
    R - 3x3 numpy matrix
    """

    # TODO - fill out each expression for R based on the condition 
    # (hint: use your rotx, roty, rotz functions)
    if order == 'xyx':
        R = rotx(th1) @ roty(th2) @ rotx(th3)
    elif order == 'xyz':
        R = rotx(th1) @ roty(th2) @ rotz(th3)
    elif order == 'xzx':
        R = rotx(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'xzy':
        R = rotx(th1) @ rotz(th2) @ roty(th3)
    elif order == 'yxy':
        R = roty(th1) @ rotx(th2) @ roty(th3)
    elif order == 'yxz':
        R = roty(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'yzx':
        R = roty(th1) @ rotz(th2) @ rotx(th3)
    elif order == 'yzy':
        R = roty(th1) @ rotz(th2) @ roty(th3)
    elif order == 'zxy':
        R = rotz(th1) @ rotx(th2) @ roty(th3)
    elif order == 'zxz':
        R = rotz(th1) @ rotx(th2) @ rotz(th3)
    elif order == 'zyx':
        R = rotz(th1) @ roty(th2) @ rotx(th3)
    elif order == 'zyz':
        R = rotz(th1) @ roty(th2) @ rotz(th3)
    else:
        print("Invalid Order!")
        return

    return R
