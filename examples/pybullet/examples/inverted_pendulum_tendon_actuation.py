"""
https://valerolab.org/

PID control of an inverted pendulum actuated by strings.
"""

import pybullet as p
import time
import math as m
import numpy as np
import pybullet_data
import matplotlib.pyplot as plt


p.connect(p.GUI)
plane = p.loadURDF("plane.urdf")

"""_____________________________________________________________________________________________________________________________"""
"""Gains, motor forces, daq and timing parameters"""

""" Gains for pendulum angle"""
proportional_gain = 30000
integral_gain = 18000
derivative_gain = 22000

"""Motor force parameters"""
tension_force = 600

"""Control input parameters"""
u_factor = 1.5
u_lower_limit =  tension_force
u_upper_limit=9000

"""Data aquisition, timing and history"""
time_steps = 2000
history = np.array( [[1000,-1000,0]] )
time_history = np.array([[0]])
previous_pendulum_angle = 0
previous_cart_position = 0

"""_____________________________________________________________________________________________________________________________"""
"""Loading URDF files"""

cubeStartPos = [-2.15,0,.75]
cubeStartPos2 = [0,0,1.4]
cubeStartPos3 = [2.15,0,.75]

PulleyStartOrientation = p.getQuaternionFromEuler([1.570796, 0, 0])  
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0]) 
cubeStartOrientation2 = p.getQuaternionFromEuler([0,-1.570796,0])

base_1 = p.loadURDF("Base_1.urdf",cubeStartPos3, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION) #Base 1: magenta base and tendon
base_2 = p.loadURDF("Base_2.urdf",cubeStartPos, cubeStartOrientation, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION)  #Base 2: white base and tendon
pendulum = p.loadURDF("Pendulum_Tendon_1_Cart_Rail.urdf",cubeStartPos2, cubeStartOrientation2, useFixedBase=1, flags=p.URDF_USE_SELF_COLLISION) 


"""_____________________________________________________________________________________________________________________________"""
"""Getting access and information from specific joints in each body (each body has links and joint described in the URDF files):"""
nJoints = p.getNumJoints(base_1)  #Base 1: magenta base and tendon
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(base_1, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
Base_pulley_1 = jointNameToId['Base_pulley1']

nJoints = p.getNumJoints(pendulum)
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(pendulum, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
last_tendon_link_1 = jointNameToId['tendon1_13_tendon1_14']
cart_pendulumAxis = jointNameToId['cart_pendulumAxis']
cart = jointNameToId['slider_cart']

nJoints = p.getNumJoints(base_2)  #Base 2: white base and tendon
jointNameToId = {}
for i in range(nJoints):
  jointInfo = p.getJointInfo(base_2, i)
  jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
last_tendon_link_2 = jointNameToId['tendon1_13_tendon1_14']
Base_pulley_2 = jointNameToId['Base_pulley1']
"""_____________________________________________________________________________________________________________________________"""
"""Creating new contraints (joints), with the information obtained in the previous step"""

pulley_1_tendon_magenta = p.createConstraint(base_1, Base_pulley_1, pendulum, last_tendon_link_1, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [-.56, 0, 0])
tendon_white_cart = p.createConstraint(base_2, last_tendon_link_2, pendulum, cart, p.JOINT_FIXED, [0, 0, 1], [0, 0, 0], [0,-.55, 0])

"""_____________________________________________________________________________________________________________________________"""
"""Defining some motor conditions"""
p.setJointMotorControl2(pendulum, cart_pendulumAxis, p.VELOCITY_CONTROL, targetVelocity=0, force=0)
p.setJointMotorControl2(base_1, Base_pulley_1, p.VELOCITY_CONTROL, targetVelocity=10, force=1000) #Base 1: magenta base and tendon
p.setJointMotorControl2(base_2, Base_pulley_2, p.VELOCITY_CONTROL, targetVelocity=10, force=-1000)#Base 2: white base and tendon


"""_____________________________________________________________________________________________________________________________"""


p.setGravity(0,0,-10)


for i in range (time_steps):
    p.stepSimulation()
    pendulum_angle = p.getJointState(pendulum,cart_pendulumAxis)
    pendulum_angle = pendulum_angle[0]
    angle_delta_error = -pendulum_angle

    #PROPPORTIONAL
    p_correction = proportional_gain * pendulum_angle

    #INTEGRAL
    i_correction = integral_gain * (previous_pendulum_angle + pendulum_angle)
    previous_pendulum_angle = pendulum_angle

    #DERIVATIVE
    d_correction = derivative_gain * angle_delta_error

    u = p_correction + i_correction + d_correction + 10
    
    u = abs(u)
    if u<u_lower_limit:
      u=u_lower_limit
    elif u>u_upper_limit:
      u=u_upper_limit   
    
    if pendulum_angle > 0:
      u_pulley_1 = u * u_factor   #Base 1: magenta base and tendon
      u_pulley_2 = -tension_force #Base 2: white base and tendon
      #print(">0")
    else:
      u_pulley_1 = tension_force  #Base 1: magenta base and tendon
      u_pulley_2 = -u * u_factor  #Base 2: white base and tendon
      #print("<0")

    p.setJointMotorControl2(base_1, Base_pulley_1, p.VELOCITY_CONTROL, targetVelocity=100, force = u_pulley_1)  
    p.setJointMotorControl2(base_2, Base_pulley_2, p.VELOCITY_CONTROL, targetVelocity=100, force = u_pulley_2)  

    history = np.append(history , [[ u_pulley_1, u_pulley_2, pendulum_angle]] , axis = 0)    
        
    time.sleep(1./240.)

print("Done with simulation")

fig, ax1 = plt.subplots()
ax1.set_xlabel("Time Steps")
ax1.set_ylabel("Activation Values")
ax1.plot(history[:,0],label="u_pulley_1")
ax1.plot(history[:,1],label="u_pulley_2")
ax1.set_ylim((-12000,12000))

plt.legend(loc='best', bbox_to_anchor=(0.5, 0., 0.5, 0.5),
           ncol=1, mode=None, borderaxespad=0.)
plt.title("Ctrl Input and Angle History") 
plt.grid(True)
color = 'tab:red'
ax2 = ax1.twinx()
ax2.set_ylabel('Pendulum Angle', color=color)
ax2.plot(np.rad2deg(history[:,2]),label="Angle",color=color)
ax2.tick_params(axis='y', labelcolor=color)
ax2.set_ylim((-90,90))

fig.tight_layout()
plt.show()
    
p.disconnect()



