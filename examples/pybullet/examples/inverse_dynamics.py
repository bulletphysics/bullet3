import pybullet as bullet
plot = True
import time

if (plot):
	import matplotlib.pyplot as plt
import math
verbose = False

# Parameters:
robot_base = [0., 0., 0.]
robot_orientation = [0., 0., 0., 1.]
delta_t = 0.0001

# Initialize Bullet Simulator
id_simulator = bullet.connect(bullet.GUI)  # or bullet.DIRECT for non-graphical version
bullet.setTimeStep(delta_t)

# Switch between URDF with/without FIXED joints
with_fixed_joints = True


if with_fixed_joints:
    id_revolute_joints = [0, 3]
    id_robot = bullet.loadURDF("TwoJointRobot_w_fixedJoints.urdf",
                               robot_base, robot_orientation, useFixedBase=True)
else:
    id_revolute_joints = [0, 1]
    id_robot = bullet.loadURDF("TwoJointRobot_wo_fixedJoints.urdf",
                               robot_base, robot_orientation, useFixedBase=True)


bullet.changeDynamics(id_robot,-1,linearDamping=0, angularDamping=0)
bullet.changeDynamics(id_robot,0,linearDamping=0, angularDamping=0)
bullet.changeDynamics(id_robot,1,linearDamping=0, angularDamping=0)

jointTypeNames = ["JOINT_REVOLUTE", "JOINT_PRISMATIC","JOINT_SPHERICAL","JOINT_PLANAR","JOINT_FIXED","JOINT_POINT2POINT","JOINT_GEAR"]
    
# Disable the motors for torque control:
bullet.setJointMotorControlArray(id_robot, id_revolute_joints, bullet.VELOCITY_CONTROL, forces=[0.0, 0.0])

# Target Positions:
start = 0.0
end = 1.0

steps = int((end-start)/delta_t)
t = [0]*steps
q_pos_desired = [[0.]* steps,[0.]* steps]
q_vel_desired = [[0.]* steps,[0.]* steps]
q_acc_desired = [[0.]* steps,[0.]* steps]

for s in range(steps):
	t[s] = start+s*delta_t
	q_pos_desired[0][s] = 1./(2.*math.pi) * math.sin(2. * math.pi * t[s]) - t[s]
	q_pos_desired[1][s] = -1./(2.*math.pi) * (math.cos(2. * math.pi * t[s]) - 1.0)
	 
	q_vel_desired[0][s] = math.cos(2. * math.pi * t[s]) - 1.
	q_vel_desired[1][s] = math.sin(2. * math.pi * t[s])
	
	q_acc_desired[0][s] = -2. * math.pi * math.sin(2. * math.pi * t[s])
	q_acc_desired[1][s] =  2. * math.pi * math.cos(2. * math.pi * t[s])
	 

q_pos = [[0.]* steps,[0.]* steps]
q_vel = [[0.]* steps,[0.]* steps]
q_tor = [[0.]* steps,[0.]* steps]

# Do Torque Control:
for i in range(len(t)):

    # Read Sensor States:
    joint_states = bullet.getJointStates(id_robot, id_revolute_joints)
    
    q_pos[0][i] = joint_states[0][0]
    a = joint_states[1][0]
    if (verbose):
      print("joint_states[1][0]")
      print(joint_states[1][0])
    q_pos[1][i] = a
    
    q_vel[0][i] = joint_states[0][1]
    q_vel[1][i] = joint_states[1][1]

    # Computing the torque from inverse dynamics:
    obj_pos = [q_pos[0][i], q_pos[1][i]]
    obj_vel = [q_vel[0][i], q_vel[1][i]]
    obj_acc = [q_acc_desired[0][i], q_acc_desired[1][i]]

    if (verbose):
	    print("calculateInverseDynamics")
	    print("id_robot")
	    print(id_robot)
	    print("obj_pos")
	    print(obj_pos)
	    print("obj_vel")
	    print(obj_vel)
	    print("obj_acc")
	    print(obj_acc)
    
    torque = bullet.calculateInverseDynamics(id_robot, obj_pos, obj_vel, obj_acc)
    q_tor[0][i] = torque[0]
    q_tor[1][i] = torque[1]
    if (verbose):
    	print("torque=")
    	print(torque)

    # Set the Joint Torques:
    bullet.setJointMotorControlArray(id_robot, id_revolute_joints, bullet.TORQUE_CONTROL, forces=[torque[0], torque[1]])

    # Step Simulation
    bullet.stepSimulation()

# Plot the Position, Velocity and Acceleration:
if plot:
	figure = plt.figure(figsize=[15, 4.5])
	figure.subplots_adjust(left=0.05, bottom=0.11, right=0.97, top=0.9, wspace=0.4, hspace=0.55)
	
	ax_pos = figure.add_subplot(141)
	ax_pos.set_title("Joint Position")
	ax_pos.plot(t, q_pos_desired[0], '--r', lw=4, label='Desired q0')
	ax_pos.plot(t, q_pos_desired[1], '--b', lw=4, label='Desired q1')
	ax_pos.plot(t, q_pos[0], '-r', lw=1, label='Measured q0')
	ax_pos.plot(t, q_pos[1], '-b', lw=1, label='Measured q1')
	ax_pos.set_ylim(-1., 1.)
	ax_pos.legend()
	
	ax_vel = figure.add_subplot(142)
	ax_vel.set_title("Joint Velocity")
	ax_vel.plot(t, q_vel_desired[0], '--r', lw=4, label='Desired q0')
	ax_vel.plot(t, q_vel_desired[1], '--b', lw=4, label='Desired q1')
	ax_vel.plot(t, q_vel[0], '-r', lw=1, label='Measured q0')
	ax_vel.plot(t, q_vel[1], '-b', lw=1, label='Measured q1')
	ax_vel.set_ylim(-2., 2.)
	ax_vel.legend()
	
	ax_acc = figure.add_subplot(143)
	ax_acc.set_title("Joint Acceleration")
	ax_acc.plot(t, q_acc_desired[0], '--r', lw=4, label='Desired q0')
	ax_acc.plot(t, q_acc_desired[1], '--b', lw=4, label='Desired q1')
	ax_acc.set_ylim(-10., 10.)
	ax_acc.legend()
	
	ax_tor = figure.add_subplot(144)
	ax_tor.set_title("Executed Torque")
	ax_tor.plot(t, q_tor[0], '-r', lw=2, label='Torque q0')
	ax_tor.plot(t, q_tor[1], '-b', lw=2, label='Torque q1')
	ax_tor.set_ylim(-20., 20.)
	ax_tor.legend()

	plt.pause(0.01)	


while (1):
    bullet.stepSimulation()
    time.sleep(0.01)	
