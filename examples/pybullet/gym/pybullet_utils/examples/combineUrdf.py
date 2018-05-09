import pybullet_utils.bullet_client as bc
import pybullet_utils.urdfEditor as ed
import pybullet
import pybullet_data
import time

p0 = bc.BulletClient(connection_mode=pybullet.DIRECT)
p0.setAdditionalSearchPath(pybullet_data.getDataPath())

p1 = bc.BulletClient(connection_mode=pybullet.DIRECT)
p1.setAdditionalSearchPath(pybullet_data.getDataPath())

#can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER

husky = p1.loadURDF("husky/husky.urdf", flags=p0.URDF_USE_IMPLICIT_CYLINDER)
kuka = p0.loadURDF("kuka_iiwa/model_free_base.urdf", flags=p0.URDF_USE_IMPLICIT_CYLINDER)

ed0 = ed.UrdfEditor()
ed0.initializeFromBulletBody(husky, p1._client)
ed1 = ed.UrdfEditor()
ed1.initializeFromBulletBody(kuka, p0._client)
#ed1.saveUrdf("combined.urdf")


parentLinkIndex = 0
childLinkIndex = len(ed0.urdfLinks)
insertJointIndex = len(ed0.urdfJoints)

#combine all links, and add a joint
for link in ed1.urdfLinks:
	ed0.linkNameToIndex[link.link_name]=len(ed0.urdfLinks)
	ed0.urdfLinks.append(link)
for joint in ed1.urdfJoints:
	ed0.urdfJoints.append(joint)
#add a new joint between a particular


jointPivotXYZInParent = [0.1,0,0.1]
jointPivotRPYInParent = [0,0,0]

jointPivotXYZInChild = [0,0,0]
jointPivotRPYInChild = [0,0,0]
jointPivotQuatInChild = p0.getQuaternionFromEuler(jointPivotRPYInChild)
invJointPivotXYZInChild, invJointPivotQuatInChild = p0.invertTransform(jointPivotXYZInChild,jointPivotQuatInChild)



#apply this invJointPivot***InChild to all inertial, visual and collision element in the child link
#inertial
pos, orn = p0.multiplyTransforms(ed0.urdfLinks[childLinkIndex].urdf_inertial.origin_xyz,p0.getQuaternionFromEuler(ed0.urdfLinks[childLinkIndex].urdf_inertial.origin_rpy),invJointPivotXYZInChild,invJointPivotQuatInChild)
ed0.urdfLinks[childLinkIndex].urdf_inertial.origin_xyz = pos
ed0.urdfLinks[childLinkIndex].urdf_inertial.origin_rpy = p0.getEulerFromQuaternion(orn)
#all visual
for v in ed0.urdfLinks[childLinkIndex].urdf_visual_shapes:
	pos, orn = p0.multiplyTransforms(v.origin_xyz,p0.getQuaternionFromEuler(v.origin_rpy),invJointPivotXYZInChild,invJointPivotQuatInChild)
	v.origin_xyz = pos
	v.origin_rpy = p0.getEulerFromQuaternion(orn)
#all collision
for c in ed0.urdfLinks[childLinkIndex].urdf_collision_shapes:
	pos, orn = p0.multiplyTransforms(c.origin_xyz,p0.getQuaternionFromEuler(c.origin_rpy),invJointPivotXYZInChild,invJointPivotQuatInChild)
	c.origin_xyz = pos
	c.origin_rpy = p0.getEulerFromQuaternion(orn)


childLink = ed0.urdfLinks[childLinkIndex]
parentLink = ed0.urdfLinks[parentLinkIndex]


joint = ed.UrdfJoint()
joint.link = childLink
joint.joint_name = "joint_dummy1"
joint.joint_type = p0.JOINT_REVOLUTE
joint.joint_lower_limit = 0
joint.joint_upper_limit = -1
joint.parent_name = parentLink.link_name
joint.child_name = childLink.link_name
joint.joint_origin_xyz = jointPivotXYZInParent
joint.joint_origin_rpy = jointPivotRPYInParent
joint.joint_axis_xyz = [0,0,1]


#the following commented line would crash PyBullet, it messes up the joint indexing/ordering
#ed0.urdfJoints.append(joint)

#so make sure to insert the joint in the right place, to links/joints match
ed0.urdfJoints.insert(insertJointIndex,joint)

ed0.saveUrdf("combined.urdf")

print(p0._client)
print(p1._client)
print("p0.getNumBodies()=",p0.getNumBodies())
print("p1.getNumBodies()=",p1.getNumBodies())

pgui = bc.BulletClient(connection_mode=pybullet.GUI)

ed0.createMultiBody([0,0,0],pgui._client)
pgui.setRealTimeSimulation(1)

while (pgui.isConnected()):
	time.sleep(1./240.)
