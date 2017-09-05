import pybullet as p
from robot_bases import BodyPart
import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
os.sys.path.insert(0,parentdir)
import pybullet_data



def get_cube(x, y, z):
	print("get cube")
	body = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"cube_small.urdf"), x, y, z)
	part_name, _ = p.getBodyInfo(body, 0)
	part_name = part_name.decode("utf8")
	bodies = [body]
	return BodyPart(part_name, bodies, 0, -1)


def get_sphere(x, y, z):
	body = p.loadURDF(os.path.join(pybullet_data.getDataPath(),"sphere_small.urdf"), x, y, z)
	part_name, _ = p.getBodyInfo(body, 0)
	part_name = part_name.decode("utf8")
	bodies = [body]
	return BodyPart(part_name, bodies, 0, -1)