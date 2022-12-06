# %% [markdown]
# # Object Avoidance

import kinematics as kin  #this is your kinematics file that you've been developing all along
from visualization import VizScene # this is the newest visualization file updated on Oct 12
import time
import numpy as np
import pandas as pd
from scipy import signal


def compute_robot_path(q_init, goal, obst_location, obst_radius, joint_limits):
      # some initialization:
      q_s = []
      error = None
      count = 0
      maximum_reach = 0

      # Import constants:
      max_iter=1000
      tol=1e-4
      K=np.eye(3)
      kd=0.0001
      dt=0.01

      # Perform some checks:
      if isinstance(q_init, np.ndarray):
                  q = q_init
      elif q_init == None:
            q = np.zeros(1,4)
      else:
            q = np.array(q_init)

      for i in range(arm.n):  # Add max length of each link
            maximum_reach = maximum_reach + np.sqrt(arm.dh[i][1] ** 2 + arm.dh[i][2] ** 2)

      pt = goal  
      target_distance = np.sqrt(pt[0] ** 2 + pt[1] ** 2 + pt[2] ** 2) # Find distance to target

      if target_distance > maximum_reach:
            print("WARNING: Target outside of reachable workspace!")
            return q, error, count, False, "Failed: Out of workspace"
      else:
            if target_distance > maximum_reach:
                  print("Target out of workspace, but finding closest solution anyway")
            else:
                  print("Target passes naive reach test, distance is {:.1} and max reach is {:.1}".format(
                  float(target_distance), float(maximum_reach)))

      # Some functions for cleanliness:
      def get_error(q, point, index=None):
            cur_position = arm.fk(q=q, index=index)[0:3,3]
            e = point-cur_position
            return e

      def midpoint_error(q, point, index1=[0,0], index2=None):
            joint1 = arm.fk(q=q, index=index1)
            joint2 = arm.fk(q=q, index=index2)
            midpoint = (joint1[0:3,3] + joint2[0:3,3])/2
            e = point - midpoint
            print('join1', joint1)
            print('joint2', joint2)
            print('midpoint', midpoint)
            print('e', e)
            return e

      def get_fr(nu=0.2, envelope=0.2, kr=0.2, er_r=[]):
            if nu < envelope:
                  fr = (kr/nu**4) * np.gradient(er_r)
            else:
                  fr = er_r*0.0
            return fr

      # Find initial errors:
      er_a = get_error(q,pt) # distance from tip to goal
      er_r1 = get_error(q,obst_location) # distance from tip to object
      er_r2 = get_error(q,obst_location,index=[0,3]) # distance from last revolute joint to object
      er_r3 = midpoint_error(q, obst_location, index1=[0,2], index2=[0,3]) # distance from midpoint on 2nd link to object
      er_r4 = get_error(q, obst_location, index=[0,2]) # distance from first revolute joint to object
      er_r5 = get_error(q, [0, 0, 0]) # distance from tip to base1
      er_r6 = get_error(q, [0, 0, 0.1]) # distance from tip to base2

      # Generate q_s using Artificial Potential Field method:
      while np.linalg.norm(er_a) > tol and count < max_iter: 
            J = arm.jacob(q)[0:3,:]
            
            # Generate Attractive Field
            ka = 0.5
            fa = ka * er_a

            # Generate Repulsive Field for object
            nu1 = np.linalg.norm(er_r1) - obst_radius
            nu2 = np.linalg.norm(er_r2) - obst_radius
            nu3 = np.linalg.norm(er_r3) - obst_radius
            nu4 = np.linalg.norm(er_r4) - obst_radius
            nu5 = np.linalg.norm(er_r5) - obst_radius
            nu6 = np.linalg.norm(er_r6) - obst_radius


            if arm.fk(q_init)[2,3] < goal[2]:
                  fr1 = get_fr(nu=nu1, envelope=0.3, kr=0.6, er_r=er_r1)
                  fr2 = get_fr(nu=nu2, envelope=0.2, kr=0.6, er_r=er_r2)
                  fr3 = get_fr(nu=nu3, envelope=0.2, kr=0.3, er_r=er_r3)
                  fr4 = get_fr(nu=nu4, envelope=0.2, kr=0.6, er_r=er_r4)
                  fr5 = get_fr(nu=nu5, envelope=0.2, kr=0.6, er_r=er_r5)
                  fr6 = get_fr(nu=nu6, envelope=0.2, kr=0.6, er_r=er_r6)

            else:
                  fr1 = get_fr(nu=nu1, envelope=0.3, kr=0.6, er_r=er_r1)
                  fr2 = get_fr(nu=nu2, envelope=0.3, kr=0.6, er_r=er_r2)
                  fr3 = get_fr(nu=nu3, envelope=0.4, kr=0.3, er_r=er_r3)
                  fr4 = get_fr(nu=nu4, envelope=0.2, kr=0.6, er_r=er_r4)
                  fr5 = get_fr(nu=nu5, envelope=0.2, kr=0.6, er_r=er_r5)
                  fr6 = get_fr(nu=nu6, envelope=0.2, kr=0.6, er_r=er_r6)
            
            # Calculate next q
            error = fa + fr1 + fr2 + fr3 + fr4 # + fr5 + fr6  # sum of attractive and repulsive fields
            if (arm.fk(q=q)[2,3] + error[2]) < 0: # if the EE is about to go under the floor,
                  error[2] = 0 # set the z-velocity equal to zero
            Ker = K @ error
            qdot = J.T @ np.linalg.inv((J@J.T) + (kd**2*np.eye(3))) @ Ker  # use 'pinv' method
            q = q + qdot*dt

            # make sure q stays within joint limits
            for i in range(len(q)):
                  if q[i] <= (joint_limits[i][0]+5):
                        q[i] = joint_limits[i][0]+5
                  elif q[i] >= (joint_limits[i][1]-5):
                        q[i] = joint_limits[i][1]-5
            
            # Update errors
            er_a = get_error(q,pt)
            er_r1 = get_error(q,obst_location)
            er_r2 = get_error(q,obst_location,index=[0,3])
            er_r3 = midpoint_error(q, obst_location, index1=[0,2], index2=[0,3])
            er_r4 = get_error(q, obst_location, index=[0,2])
            er_r5 = get_error(q, [0, 0, 0]) # distance from tip to base1
            er_r6 = get_error(q, [0, 0, 0.1]) # distance from tip to base2

            count = count+1
            q_s.append(q)
            
      q_og = pd.DataFrame(q_s)
      q_smooth = np.zeros([1000,4])
      for i in range(len(q_smooth.T)):
            q_smooth[:,i] = signal.savgol_filter(q_og[i], 501, 6)
      q_out = np.array(q_smooth)
      
      return q_out     
    