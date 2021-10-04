import colorsys
from enum import Enum
import numpy as np
import pybullet as p
import time
import typing

p.connect(p.GUI)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(0)
p.setTimeStep(1 / 1000)

test_case = 1
rigid_body = False
apply_force_torque = True
apply_force_local = True
apply_torque_local = True


if test_case == 1:
    cs_id = p.createCollisionShape(p.GEOM_BOX,
                                   halfExtents=[0.25, 0.25, 0.25],
                                   collisionFramePosition=[0.25, 0, 0],
                                   collisionFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]))
    vs_id = p.createVisualShape(p.GEOM_BOX,
                                halfExtents=[0.25, 0.25, 0.25],
                                visualFramePosition=[0.25, 0.25, 0],
                                visualFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]))
    body = p.createMultiBody(baseMass=1,
                             baseCollisionShapeIndex=cs_id,
                             baseVisualShapeIndex=vs_id,
                             basePosition=[0, 0, 2],
                             baseOrientation=p.getQuaternionFromEuler([-1.7, -0.8, 0.1]),
                             baseInertialFramePosition=[0, 0.5, 0],
                             baseInertialFrameOrientation=p.getQuaternionFromEuler([0.6, 0, 0.4]),
                             useMaximalCoordinates=rigid_body)
else:
    cs_id = p.createCollisionShape(p.GEOM_BOX,
                                   halfExtents=[0.25, 0.25, 0.25],
                                   collisionFramePosition=[0, 0, 0],
                                   collisionFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]))
    vs_id = p.createVisualShape(p.GEOM_BOX,
                                halfExtents=[0.25, 0.25, 0.25],
                                visualFramePosition=[0, 0, 0],
                                visualFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]))
    body = p.createMultiBody(baseMass=1,
                             baseCollisionShapeIndex=cs_id,
                             baseVisualShapeIndex=vs_id,
                             basePosition=[0, 0, 2],
                             baseOrientation=p.getQuaternionFromEuler([0.9, 0.3, 0]),
                             baseInertialFramePosition=[0, 0, 0],
                             baseInertialFrameOrientation=p.getQuaternionFromEuler([0, 0, 0]),
                             linkMasses=[1],
                             linkCollisionShapeIndices=[cs_id],
                             linkVisualShapeIndices=[vs_id],
                             linkPositions=[[1, 0, 0]],
                             linkOrientations=[p.getQuaternionFromEuler([0, 0, 0])],
                             linkInertialFramePositions=[[0, 0, 0]],
                             linkInertialFrameOrientations=[p.getQuaternionFromEuler([0, 0, 0])],
                             linkParentIndices=[0],
                             linkJointTypes=[p.JOINT_FIXED],
                             linkJointAxis=[[1, 0, 0]],
                             useMaximalCoordinates=False)


def get_world_link_pose(body_unique_id: int, link_index: int):
    """Pose of link frame with respect to world frame expressed in world frame.

    Args:
        body_unique_id (int): The body unique id, as returned by loadURDF, etc.
        link_index (int): Link index or -1 for the base.

    Returns:
        pos_orn (tuple): See description.
    """
    if link_index == -1:
        world_inertial_pose = get_world_inertial_pose(body_unique_id, -1)
        dynamics_info = p.getDynamicsInfo(body_unique_id, -1)
        local_inertial_pose = (dynamics_info[3], dynamics_info[4])

        local_inertial_pose_inv = p.invertTransform(local_inertial_pose[0], local_inertial_pose[1])
        pos_orn = p.multiplyTransforms(world_inertial_pose[0],
                                       world_inertial_pose[1],
                                       local_inertial_pose_inv[0],
                                       local_inertial_pose_inv[1])
    else:
        state = p.getLinkState(body_unique_id, link_index)
        pos_orn = (state[4], state[5])

    return pos_orn


def get_world_inertial_pose(body_unique_id: int, link_index: int):
    """Pose of inertial frame with respect to world frame expressed in world frame.

    Args:
        body_unique_id (int): The body unique id, as returned by loadURDF, etc.
        link_index (int): Link index or -1 for the base.

    Returns:
        pos_orn (tuple): See description.
    """
    if link_index == -1:
        pos_orn = p.getBasePositionAndOrientation(body_unique_id)
    else:
        state = p.getLinkState(body_unique_id, link_index)
        pos_orn = (state[0], state[1])

    return pos_orn


def get_world_visual_pose(body_unique_id: int, link_index: int):
    """Pose of visual frame with respect to world frame expressed in world frame.

    Args:
        body_unique_id (int): The body unique id, as returned by loadURDF, etc.
        link_index (int): Link index or -1 for the base.

    Returns:
        pos_orn (tuple): See description.
    """
    world_link_pose = get_world_link_pose(body_unique_id, link_index)
    visual_shape_data = p.getVisualShapeData(body_unique_id, link_index)
    local_visual_pose = (visual_shape_data[link_index + 1][5], visual_shape_data[link_index + 1][6])

    return p.multiplyTransforms(world_link_pose[0],
                                world_link_pose[1],
                                local_visual_pose[0],
                                local_visual_pose[1])


def get_world_collision_pose(body_unique_id: int, link_index: int):
    """Pose of collision frame with respect to world frame expressed in world frame.

    Args:
        body_unique_id (int): The body unique id, as returned by loadURDF, etc.
        link_index (int): Link index or -1 for the base.

    Returns:
        pos_orn (tuple of ): See description.
    """
    world_inertial_pose = get_world_inertial_pose(body_unique_id, link_index)
    collision_shape_data = p.getCollisionShapeData(body_unique_id, link_index)
    if len(collision_shape_data) > 1:
        raise NotImplementedError
    local_collision_pose = (collision_shape_data[0][5], collision_shape_data[0][6])

    return p.multiplyTransforms(world_inertial_pose[0],
                                world_inertial_pose[1],
                                local_collision_pose[0],
                                local_collision_pose[1])


def get_link_name(body_unique_id: int, link_index: int):
    """Returns the link name.

    Args:
        body_unique_id (int): The body unique id, as returned by loadURDF, etc.
        link_index (int): Link index or -1 for the base.

    Returns:
        link_name (str): Name of the link.
    """
    if link_index == -1:
        link_name = p.getBodyInfo(body_unique_id)[0]
    else:
        link_name = p.getJointInfo(body_unique_id, link_index)[12]

    return link_name.decode('UTF-8')


class Frame(Enum):
    LINK = 1,
    INERTIAL = 2,
    COLLISION = 3,
    VISUAL = 4


FRAME_NAME = dict()
FRAME_NAME[Frame.LINK] = 'link'
FRAME_NAME[Frame.INERTIAL] = 'inertial'
FRAME_NAME[Frame.COLLISION] = 'collision'
FRAME_NAME[Frame.VISUAL] = 'visual'


def draw_frame(body_unique_id: int,
               link_index: int,
               frame: Frame,
               title: str,
               replace_item_unique_id: typing.Tuple[int, int, int, int] = None):
    """Visualizes one of the frames/coordinate systems associated with each link (or base):
    link, inertial, visual or collision frame.

    Args:
        body_unique_id (int): The body unique id, as returned by loadURDF, etc.
        link_index (int): Link index or -1 for the base.
        frame: The frame to draw (link, inertial, visual, collision)
        title: Text which is drawn at the origin of the axis.
        replace_item_unique_id (tuple of 4 ints): Replace existing axis and title to improve
            performance and to avoid flickering.

    Returns:
        indices (tuple of 4 ints): The object id of the x-axis, y-axis, z-axis, and the title text.
    """
    if frame == Frame.LINK:
        world_pose = get_world_link_pose(body_unique_id, link_index)
    elif frame == Frame.INERTIAL:
        world_pose = get_world_inertial_pose(body_unique_id, link_index)
    elif frame == Frame.COLLISION:
        world_pose = get_world_collision_pose(body_unique_id, link_index)
    elif frame == Frame.VISUAL:
        world_pose = get_world_visual_pose(body_unique_id, link_index)
    else:
        raise NotImplementedError

    axis_scale = 0.1

    pos = np.array(world_pose[0])
    orn_mat = np.array(p.getMatrixFromQuaternion(world_pose[1])).reshape((3, 3))

    kwargs = dict()
    kwargs['lineWidth'] = 3

    kwargs['lineColorRGB'] = [1, 0, 0]
    if replace_item_unique_id is not None:
        kwargs['replaceItemUniqueId'] = replace_item_unique_id[0]
    x = p.addUserDebugLine(pos, pos + axis_scale * orn_mat[0:3, 0], **kwargs)

    kwargs['lineColorRGB'] = [0, 1, 0]
    if replace_item_unique_id is not None:
        kwargs['replaceItemUniqueId'] = replace_item_unique_id[1]
    y = p.addUserDebugLine(pos, pos + axis_scale * orn_mat[0:3, 1], **kwargs)

    kwargs['lineColorRGB'] = [0, 0, 1]
    if replace_item_unique_id is not None:
        kwargs['replaceItemUniqueId'] = replace_item_unique_id[2]
    z = p.addUserDebugLine(pos, pos + axis_scale * orn_mat[0:3, 2], **kwargs)

    kwargs.clear()
    if replace_item_unique_id is not None:
        kwargs['replaceItemUniqueId'] = replace_item_unique_id[3]
    title_index = p.addUserDebugText(title, pos, **kwargs)

    return x, y, z, title_index


class FrameDrawManager:
    """Provides a straightforward and efficient way to draw frames/coordinate systems that are
    associated with each link (or base). This includes the link, inertial, collision, and
    visual frame.
    """

    def __init__(self):
        self.line_indices = dict()

    def _add_frame(self, frame: Frame, body_unique_id: int, link_index: int):
        # Workaround for the following problem:
        # When too many lines are added within a short period of time, the following error can occur
        # "b3Printf: b3Warning[examples/SharedMemory/PhysicsClientSharedMemory.cpp,1286]:
        # b3Printf: User debug draw failed".
        time.sleep(1 / 100)

        if self.line_indices.get(body_unique_id) is None:
            self.line_indices[body_unique_id] = dict()

        if self.line_indices[body_unique_id].get(frame) is None:
            self.line_indices[body_unique_id][frame] = dict()

        if self.line_indices[body_unique_id][frame].get(link_index) is None:
            data = dict()
            data['title'] = \
                get_link_name(body_unique_id, link_index) + " (" + FRAME_NAME[frame] + " frame)"
            data['replace_item_unique_id'] = draw_frame(body_unique_id,
                                                        link_index,
                                                        frame,
                                                        data['title'])

            self.line_indices[body_unique_id][frame][link_index] = data

    def add_link_frame(self, body_unique_id: int, link_index: int):
        self._add_frame(Frame.LINK, body_unique_id, link_index)

    def add_inertial_frame(self, body_unique_id: int, link_index: int):
        self._add_frame(Frame.INERTIAL, body_unique_id, link_index)

    def add_collision_frame(self, body_unique_id: int, link_index: int):
        self._add_frame(Frame.COLLISION, body_unique_id, link_index)

    def add_visual_frame(self, body_unique_id: int, link_index: int):
        self._add_frame(Frame.VISUAL, body_unique_id, link_index)

    def update(self):
        """Updates the drawing of all known frames. Note that this function is supposed to be
        called after each simulation step.
        """
        for body_unique_id, dict0 in self.line_indices.items():
            for frame, dict1 in dict0.items():
                for link_index, dict2 in dict1.items():
                    draw_frame(body_unique_id,
                               link_index,
                               frame,
                               dict2['title'],
                               dict2['replace_item_unique_id'])


def high_contrast_bodies(alpha: float = 0.5):
    """Makes all bodies transparent and gives each body an individual color.

    Args:
        alpha (float): Regulates the strength of transparency.
    """
    num_bodies = p.getNumBodies()

    hsv = [(x * 1.0 / num_bodies, 0.5, 0.5) for x in range(num_bodies)]
    rgb = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv))

    for i in range(num_bodies):
        body_unique_id = p.getBodyUniqueId(i)
        for link_index in range(-1, p.getNumJoints(body_unique_id)):
            p.changeVisualShape(body_unique_id,
                                link_index,
                                rgbaColor=[rgb[i][0], rgb[i][1], rgb[i][2], alpha])


high_contrast_bodies()

frame_draw_manager = FrameDrawManager()
if test_case == 1:
    frame_draw_manager.add_link_frame(body, -1)
    frame_draw_manager.add_inertial_frame(body, -1)
    if not rigid_body:
        frame_draw_manager.add_collision_frame(body, -1)
    frame_draw_manager.add_visual_frame(body, -1)
else:
    for i in range(-1, p.getNumJoints(body)):
        frame_draw_manager.add_inertial_frame(body, i)

if apply_force_torque:
    while 1:
        # The following two options are equivalent and are suppose to hold the body in place.
        if apply_force_local:
            for i in range(-1, p.getNumJoints(body)):
                pose = get_world_inertial_pose(body, i)
                pose_inv = p.invertTransform(pose[0], pose[1])
                force = p.multiplyTransforms([0, 0, 0], pose_inv[1], [0, 0, 10], [0, 0, 0, 1])
                p.applyExternalForce(body, i, force[0], [0, 0, 0], flags=p.LINK_FRAME)
        else:
            for i in range(-1, p.getNumJoints(body)):
                pose = get_world_inertial_pose(body, i)
                p.applyExternalForce(body, i, [0, 0, 10], pose[0], flags=p.WORLD_FRAME)

        if test_case == 1:
            if apply_torque_local:
                p.applyExternalTorque(body, -1, [0, 0, 100], flags=p.LINK_FRAME)
            else:
                p.applyExternalTorque(body, -1, [0, 0, 100], flags=p.WORLD_FRAME)
        else:
            if apply_torque_local:
                p.applyExternalTorque(body, -1, [0, 0, 100], flags=p.LINK_FRAME)
                p.applyExternalTorque(body, 0, [0, 0, 100], flags=p.LINK_FRAME)
            else:
                p.applyExternalTorque(body, -1, [0, 0, 100], flags=p.WORLD_FRAME)
                p.applyExternalTorque(body, 0, [0, 0, 100], flags=p.WORLD_FRAME)

        p.stepSimulation()
        frame_draw_manager.update()
        time.sleep(1 / 10)
else:
    while 1:
        time.sleep(1 / 10)
