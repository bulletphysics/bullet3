import os
from .scene_abstract import Scene
import pybullet as p

class StadiumScene(Scene):
	zero_at_running_strip_start_line = True   # if False, center of coordinates (0,0,0) will be at the middle of the stadium
	stadium_halflen   = 105*0.25	# FOOBALL_FIELD_HALFLEN
	stadium_halfwidth = 50*0.25	 # FOOBALL_FIELD_HALFWID

	def episode_restart(self):
		Scene.episode_restart(self)   # contains cpp_world.clean_everything()
		# stadium_pose = cpp_household.Pose()
		# if self.zero_at_running_strip_start_line:
		#	 stadium_pose.set_xyz(27, 21, 0)  # see RUN_STARTLINE, RUN_RAD constants
		self.stadium = p.loadSDF("stadium.sdf")
		self.ground_plane_mjcf = p.loadMJCF("mjcf/ground_plane.xml")

class SinglePlayerStadiumScene(StadiumScene):
	"This scene created by environment, to work in a way as if there was no concept of scene visible to user."
	multiplayer = False

class MultiplayerStadiumScene(StadiumScene):
	multiplayer = True
	players_count = 3
	def actor_introduce(self, robot):
		StadiumScene.actor_introduce(self, robot)
		i = robot.player_n - 1  # 0 1 2 => -1 0 +1
		robot.move_robot(0, i, 0)
