import pybullet as p
import time

p.connect(p.GUI)

door = p.loadURDF("door.urdf")

class UrdfInertial(object):
	def __init__(self):
		self.mass = 1
		self.inertia_xxyyzz=[7,8,9]
		self.origin_rpy=[1,2,3]
		self.origin_xyz=[4,5,6]

class UrdfContact(object):
	def __init__(self):
		self.lateral_friction = 1
		self.rolling_friction = 0
		self.spinning_friction = 0

class UrdfLink(object):
	def __init__(self):
		self.link_name = "dummy"
		self.urdf_inertial = UrdfInertial()
		self.urdf_visual_shapes=[]
		self.urdf_collision_shapes=[]
		
class UrdfVisual(object):
	def __init__(self):
		self.origin_rpy = [1,2,3]
		self.origin_xyz = [4,5,6]
		self.geom_type = p.GEOM_BOX
		self.geom_radius = 1
		self.geom_extents = [7,8,9]
		self.geom_length=[10]
		self.geom_meshfile = "meshfile"
		self.material_rgba = [1,0,0,1]
		self.material_name = ""
		
class UrdfCollision(object):
	def __init__(self):
		self.origin_rpy = [1,2,3]
		self.origin_xyz = [4,5,6]
		self.geom_type = p.GEOM_BOX
		self.geom_radius = 1
		self.geom_extents = [7,8,9]
		self.geom_meshfile = "meshfile"
	
class UrdfJoint(object):
	def __init__(self):
		self.joint_name = "joint_dummy"
		self.joint_type = p.JOINT_REVOLUTE
		self.joint_lower_limit = 0
		self.joint_upper_limit = -1
		self.parent_name = "parentName"
		self.child_name = "childName"
		self.joint_origin_xyz = [1,2,3]
		self.joint_axis_xyz = [1,2,3]
		 
class UrdfEditor(object):
	def __init__(self):
		self.initialize()

	def initialize(self):
		self.urdfLinks=[]
		self.urdfJoints=[]
		self.robotName = ""
		
	#def addLink(...)

	#def createMultiBody(self):
		
	def convertLinkFromMultiBody(self, bodyUid, linkIndex, urdfLink):
		dyn = p.getDynamicsInfo(bodyUid,linkIndex)
		urdfLink.urdf_inertial.mass = dyn[0]
		urdfLink.urdf_inertial.inertia_xxyyzz = dyn[2]
		#todo
		urdfLink.urdf_inertial.origin_xyz = dyn[3]
		rpy = p.getEulerFromQuaternion(dyn[4])
		urdfLink.urdf_inertial.origin_rpy = rpy
		
		visualShapes = p.getVisualShapeData(bodyUid)
		matIndex = 0
		for v in visualShapes:
			if (v[1]==linkIndex):
				print("visualShape base:",v)
				urdfVisual = UrdfVisual()
				urdfVisual.geom_type = v[2]
				if (v[2]==p.GEOM_BOX):
					urdfVisual.geom_extents = v[3]
				if (v[2]==p.GEOM_SPHERE):
					urdfVisual.geom_radius = v[3][0]	
				if (v[2]==p.GEOM_MESH):
					urdfVisual.geom_meshfile = v[4].decode("utf-8")
				if (v[2]==p.GEOM_CYLINDER):
					urdfVisual.geom_radius=v[3][1]
					urdfVisual.geom_length=v[3][0]
					
				urdfVisual.origin_xyz = v[5]
				urdfVisual.origin_rpy = p.getEulerFromQuaternion(v[6])
				urdfVisual.material_rgba = v[7]
				name = 'mat_{}_{}'.format(linkIndex,matIndex)
				urdfVisual.material_name = name
				
				urdfLink.urdf_visual_shapes.append(urdfVisual)
				matIndex=matIndex+1
				
		collisionShapes = p.getCollisionShapeData(bodyUid, linkIndex)
		for v in collisionShapes:
			print("collisionShape base:",v)
			urdfCollision = UrdfCollision()
			print("geom type=",v[0])
			urdfCollision.geom_type = v[2]
			if (v[2]==p.GEOM_BOX):
				urdfCollision.geom_extents = v[3]
			if (v[2]==p.GEOM_SPHERE):
				urdfCollision.geom_radius = v[3][0]
			if (v[2]==p.GEOM_MESH):
					urdfCollision.geom_meshfile = v[4].decode("utf-8")
			#localInertiaFrame*childTrans
			if (v[2]==p.GEOM_CYLINDER):
				urdfCollision.geom_radius=v[3][1]
				urdfCollision.geom_length=v[3][0]

			pos,orn = p.multiplyTransforms(dyn[3],dyn[4],\
				v[5], v[6])
			urdfCollision.origin_xyz = pos
			urdfCollision.origin_rpy = p.getEulerFromQuaternion(orn)
			urdfLink.urdf_collision_shapes.append(urdfCollision)

	def initializeFromBulletBody(self, bodyUid):
		self.initialize()

		#always create a base link
		baseLink = UrdfLink()
		baseLinkIndex = -1
		self.convertLinkFromMultiBody(bodyUid, baseLinkIndex, baseLink)
		baseLink.link_name = 	p.getBodyInfo(bodyUid)[0].decode("utf-8") 		
		self.urdfLinks.append(baseLink)
	
		#print(visualShapes)
		#optionally create child links and joints
		for j in range(p.getNumJoints(bodyUid)):
			jointInfo = p.getJointInfo(bodyUid,j)
			urdfLink = UrdfLink()
			self.convertLinkFromMultiBody(bodyUid, j, urdfLink)
			urdfLink.link_name = jointInfo[12].decode("utf-8")
			self.urdfLinks.append(urdfLink)
	
			urdfJoint = UrdfJoint()
			urdfJoint.joint_name = jointInfo[1].decode("utf-8")
			urdfJoint.joint_type = jointInfo[2]
			urdfJoint.joint_axis_xyz = jointInfo[13]
			parentIndex = jointInfo[16]
			if (parentIndex<0):
				urdfJoint.parent_name = baseLink.link_name
			else:
				parentJointInfo = p.getJointInfo(bodyUid,parentIndex)
				urdfJoint.parent_name = parentJointInfo[12].decode("utf-8")

			urdfJoint.child_name = urdfLink.link_name

			#todo, compensate for inertia/link frame offset
			dyn = p.getDynamicsInfo(bodyUid,parentIndex)
			parentInertiaPos = dyn[3]
			parentInertiaOrn = dyn[4]
			
			pos,orn = p.multiplyTransforms(dyn[3],dyn[4],\
				jointInfo[14], jointInfo[15])
			urdfJoint.joint_origin_xyz = pos
			urdfJoint.joint_origin_rpy = p.getEulerFromQuaternion(orn)

				
			self.urdfJoints.append(urdfJoint)
			
	def writeInertial(self,file,urdfInertial, precision=5):
		file.write("\t\t<inertial>\n")
		str = '\t\t\t<origin rpy=\"{:.{prec}f} {:.{prec}f} {:.{prec}f}\" xyz=\"{:.{prec}f} {:.{prec}f} {:.{prec}f}\"/>\n'.format(\
		urdfInertial.origin_rpy[0],urdfInertial.origin_rpy[1],urdfInertial.origin_rpy[2],\
		urdfInertial.origin_xyz[0],urdfInertial.origin_xyz[1],urdfInertial.origin_xyz[2], prec=precision)
		file.write(str)
		str = '\t\t\t<mass value=\"{:.{prec}f}\"/>\n'.format(urdfInertial.mass,prec=precision)
		file.write(str)
		str = '\t\t\t<inertia ixx=\"{:.{prec}f}\" ixy=\"0\" ixz=\"0\" iyy=\"{:.{prec}f}\" iyz=\"0\" izz=\"{:.{prec}f}\"/>\n'.format(\
		urdfInertial.inertia_xxyyzz[0],\
		urdfInertial.inertia_xxyyzz[1],\
		urdfInertial.inertia_xxyyzz[2],prec=precision)
		file.write(str)
		file.write("\t\t</inertial>\n")
  
	def writeVisualShape(self,file,urdfVisual, precision=5):
		file.write("\t\t<visual>\n")
		str = '\t\t\t<origin rpy="{:.{prec}f} {:.{prec}f} {:.{prec}f}" xyz="{:.{prec}f} {:.{prec}f} {:.{prec}f}"/>\n'.format(\
			urdfVisual.origin_rpy[0],urdfVisual.origin_rpy[1],urdfVisual.origin_rpy[2],
			urdfVisual.origin_xyz[0],urdfVisual.origin_xyz[1],urdfVisual.origin_xyz[2], prec=precision)
		file.write(str)
		file.write("\t\t\t<geometry>\n")
		if urdfVisual.geom_type == p.GEOM_BOX:
			str = '\t\t\t\t<box size=\"{:.{prec}f} {:.{prec}f} {:.{prec}f}\"/>\n'.format(urdfVisual.geom_extents[0],\
				urdfVisual.geom_extents[1],urdfVisual.geom_extents[2], prec=precision)
			file.write(str)
		if urdfVisual.geom_type == p.GEOM_SPHERE:
			str = '\t\t\t\t<sphere radius=\"{:.{prec}f}\"/>\n'.format(urdfVisual.geom_radius,\
				prec=precision)
			file.write(str)
		if urdfVisual.geom_type == p.GEOM_MESH:
			str = '\t\t\t\t<mesh filename=\"{}\"/>\n'.format(urdfVisual.geom_meshfile,\
				prec=precision)
			file.write(str)
		if urdfVisual.geom_type == p.GEOM_CYLINDER:
			str = '\t\t\t\t<cylinder length=\"{:.{prec}f}\" radius=\"{:.{prec}f}\"/>\n'.format(\
				urdfVisual.geom_length, urdfVisual.geom_radius, prec=precision)
			file.write(str)

		file.write("\t\t\t</geometry>\n")
		str = '\t\t\t<material name=\"{}\">\n'.format(urdfVisual.material_name)
		file.write(str)
		str = '\t\t\t\t<color rgba="{:.{prec}f} {:.{prec}f} {:.{prec}f} {:.{prec}f}" />\n'.format(urdfVisual.material_rgba[0],\
			urdfVisual.material_rgba[1],urdfVisual.material_rgba[2],urdfVisual.material_rgba[3],prec=precision)
		file.write(str)
		file.write("\t\t\t</material>\n")
		file.write("\t\t</visual>\n")

	def writeCollisionShape(self,file,urdfCollision, precision=5):
		file.write("\t\t<collision>\n")
		str = '\t\t\t<origin rpy="{:.{prec}f} {:.{prec}f} {:.{prec}f}" xyz="{:.{prec}f} {:.{prec}f} {:.{prec}f}"/>\n'.format(\
			urdfCollision.origin_rpy[0],urdfCollision.origin_rpy[1],urdfCollision.origin_rpy[2],
			urdfCollision.origin_xyz[0],urdfCollision.origin_xyz[1],urdfCollision.origin_xyz[2], prec=precision)
		file.write(str)
		file.write("\t\t\t<geometry>\n")
		if urdfCollision.geom_type == p.GEOM_BOX:
			str = '\t\t\t\t<box size=\"{:.{prec}f} {:.{prec}f} {:.{prec}f}\"/>\n'.format(urdfCollision.geom_extents[0],\
				urdfCollision.geom_extents[1],urdfCollision.geom_extents[2], prec=precision)
			file.write(str)
		if urdfCollision.geom_type == p.GEOM_SPHERE:
			str = '\t\t\t\t<sphere radius=\"{:.{prec}f}\"/>\n'.format(urdfCollision.geom_radius,\
				prec=precision)
			file.write(str)
		if urdfCollision.geom_type == p.GEOM_MESH:
			str = '\t\t\t\t<mesh filename=\"{}\"/>\n'.format(urdfCollision.geom_meshfile,\
				prec=precision)
			file.write(str)
		if urdfCollision.geom_type == p.GEOM_CYLINDER:
			str = '\t\t\t\t<cylinder length=\"{:.{prec}f}\" radius=\"{:.{prec}f}\"/>\n'.format(\
				urdfCollision.geom_length, urdfCollision.geom_radius, prec=precision)
			file.write(str)
	
		file.write("\t\t\t</geometry>\n")
		file.write("\t\t</collision>\n")
		
	  
	def writeLink(self, file, urdfLink):
		file.write("\t<link name=\"")
		file.write(urdfLink.link_name)
		file.write("\">\n")
		
		self.writeInertial(file,urdfLink.urdf_inertial)
		for v in urdfLink.urdf_visual_shapes:
			self.writeVisualShape(file,v)
		for c in urdfLink.urdf_collision_shapes:
			self.writeCollisionShape(file,c)
		file.write("\t</link>\n")

	def writeJoint(self, file, urdfJoint, precision=5):
		jointTypeStr = "invalid"
		if urdfJoint.joint_type == p.JOINT_REVOLUTE:
			if urdfJoint.joint_upper_limit < urdfJoint.joint_lower_limit:
				jointTypeStr = "continuous"
			else:
				jointTypeStr = "revolute"
		if urdfJoint.joint_type == p.JOINT_FIXED:
			jointTypeStr  = "fixed"
		if urdfJoint.joint_type == p.JOINT_PRISMATIC:
			jointTypeStr  = "prismatic"
		str = '\t<joint name=\"{}\" type=\"{}\">\n'.format(urdfJoint.joint_name, jointTypeStr)
		file.write(str)
		str = '\t\t<parent link=\"{}\"/>\n'.format(urdfJoint.parent_name)
		file.write(str)
		str = '\t\t<child link=\"{}\"/>\n'.format(urdfJoint.child_name)
		file.write(str)
		file.write("\t\t<dynamics damping=\"1.0\" friction=\"0.0001\"/>\n")
		str = '\t\t<origin xyz=\"{:.{prec}f} {:.{prec}f} {:.{prec}f}\"/>\n'.format(urdfJoint.joint_origin_xyz[0],\
			urdfJoint.joint_origin_xyz[1],urdfJoint.joint_origin_xyz[2], prec=precision)
		file.write(str)
		str = '\t\t<axis xyz=\"{:.{prec}f} {:.{prec}f} {:.{prec}f}\"/>\n'.format(urdfJoint.joint_axis_xyz[0],\
			urdfJoint.joint_axis_xyz[1],urdfJoint.joint_axis_xyz[2], prec=precision)
		file.write(str)
		file.write("\t</joint>\n")
  
	def saveUrdf(self, fileName):
		file = open(fileName,"w")
		file.write("<?xml version=\"0.0\" ?>\n")
		file.write("<robot name=\"")
		file.write(self.robotName)
		file.write("\">\n")

		for link in self.urdfLinks:
			self.writeLink(file,link)
	
		for joint in self.urdfJoints:
			self.writeJoint(file,joint)
		
		file.write("</robot>\n")
		file.close()
  	
	def __del__(self):
		pass
      
parser = UrdfEditor()
parser.initializeFromBulletBody(door)
parser.saveUrdf("test.urdf")
parser=0

p.setRealTimeSimulation(1)
print("numJoints:",p.getNumJoints(door))

print("base name:",p.getBodyInfo(door))

for i in range(p.getNumJoints(door)):
	print("jointInfo(",i,"):",p.getJointInfo(door,i))
	print("linkState(",i,"):",p.getLinkState(door,i))

while (p.getConnectionInfo()["isConnected"]):
	time.sleep(0.01)
		