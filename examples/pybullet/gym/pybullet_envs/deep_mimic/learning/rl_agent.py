import numpy as np
import copy
import os
import time
import sys
from abc import abstractmethod
import abc
if sys.version_info >= (3, 4):
  ABC = abc.ABC
else:
  ABC = abc.ABCMeta('ABC', (), {})

from enum import Enum

from pybullet_envs.deep_mimic.learning.path import *
from pybullet_envs.deep_mimic.learning.exp_params import ExpParams
from pybullet_envs.deep_mimic.learning.normalizer import Normalizer
from pybullet_envs.deep_mimic.learning.replay_buffer import ReplayBuffer
from pybullet_utils.logger import Logger
import pybullet_utils.mpi_util as MPIUtil
import pybullet_utils.math_util as MathUtil


class RLAgent(ABC):

  class Mode(Enum):
    TRAIN = 0
    TEST = 1
    TRAIN_END = 2

  NAME = "None"

  UPDATE_PERIOD_KEY = "UpdatePeriod"
  ITERS_PER_UPDATE = "ItersPerUpdate"
  DISCOUNT_KEY = "Discount"
  MINI_BATCH_SIZE_KEY = "MiniBatchSize"
  REPLAY_BUFFER_SIZE_KEY = "ReplayBufferSize"
  INIT_SAMPLES_KEY = "InitSamples"
  NORMALIZER_SAMPLES_KEY = "NormalizerSamples"

  OUTPUT_ITERS_KEY = "OutputIters"
  INT_OUTPUT_ITERS_KEY = "IntOutputIters"
  TEST_EPISODES_KEY = "TestEpisodes"

  EXP_ANNEAL_SAMPLES_KEY = "ExpAnnealSamples"
  EXP_PARAM_BEG_KEY = "ExpParamsBeg"
  EXP_PARAM_END_KEY = "ExpParamsEnd"

  def __init__(self, world, id, json_data):
    self.world = world
    self.id = id
    self.logger = Logger()
    self._mode = self.Mode.TRAIN

    assert self._check_action_space(), \
        Logger.print2("Invalid action space, got {:s}".format(str(self.get_action_space())))

    self._enable_training = True
    self.path = Path()
    self.iter = int(0)
    self.start_time = time.time()
    self._update_counter = 0

    self.update_period = 1.0  # simulated time (seconds) before each training update
    self.iters_per_update = int(1)
    self.discount = 0.95
    self.mini_batch_size = int(32)
    self.replay_buffer_size = int(50000)
    self.init_samples = int(1000)
    self.normalizer_samples = np.inf
    self._local_mini_batch_size = self.mini_batch_size  # batch size for each work for multiprocessing
    self._need_normalizer_update = True
    self._total_sample_count = 0

    self._output_dir = ""
    self._int_output_dir = ""
    self.output_iters = 100
    self.int_output_iters = 100

    self.train_return = 0.0
    self.test_episodes = int(0)
    self.test_episode_count = int(0)
    self.test_return = 0.0
    self.avg_test_return = 0.0

    self.exp_anneal_samples = 320000
    self.exp_params_beg = ExpParams()
    self.exp_params_end = ExpParams()
    self.exp_params_curr = ExpParams()

    self._load_params(json_data)
    self._build_replay_buffer(self.replay_buffer_size)
    self._build_normalizers()
    self._build_bounds()
    self.reset()

    return

  def __str__(self):
    action_space_str = str(self.get_action_space())
    info_str = ""
    info_str += '"ID": {:d},\n "Type": "{:s}",\n "ActionSpace": "{:s}",\n "StateDim": {:d},\n "GoalDim": {:d},\n "ActionDim": {:d}'.format(
        self.id, self.NAME, action_space_str[action_space_str.rfind('.') + 1:],
        self.get_state_size(), self.get_goal_size(), self.get_action_size())
    return "{\n" + info_str + "\n}"

  def get_output_dir(self):
    return self._output_dir

  def set_output_dir(self, out_dir):
    self._output_dir = out_dir
    if (self._output_dir != ""):
      self.logger.configure_output_file(out_dir + "/agent" + str(self.id) + "_log.txt")
    return

  output_dir = property(get_output_dir, set_output_dir)

  def get_int_output_dir(self):
    return self._int_output_dir

  def set_int_output_dir(self, out_dir):
    self._int_output_dir = out_dir
    return

  int_output_dir = property(get_int_output_dir, set_int_output_dir)

  def reset(self):
    self.path.clear()
    return

  def update(self, timestep):
    if self.need_new_action():
      #print("update_new_action!!!")
      self._update_new_action()

    if (self._mode == self.Mode.TRAIN and self.enable_training):
      self._update_counter += timestep

      while self._update_counter >= self.update_period:
        self._train()
        self._update_exp_params()
        self.world.env.set_sample_count(self._total_sample_count)
        self._update_counter -= self.update_period

    return

  def end_episode(self):
    if (self.path.pathlength() > 0):
      self._end_path()

      if (self._mode == self.Mode.TRAIN or self._mode == self.Mode.TRAIN_END):
        if (self.enable_training and self.path.pathlength() > 0):
          self._store_path(self.path)
      elif (self._mode == self.Mode.TEST):
        self._update_test_return(self.path)
      else:
        assert False, Logger.print2("Unsupported RL agent mode" + str(self._mode))

      self._update_mode()
    return

  def has_goal(self):
    return self.get_goal_size() > 0

  def predict_val(self):
    return 0

  def get_enable_training(self):
    return self._enable_training

  def set_enable_training(self, enable):
    print("set_enable_training=", enable)
    self._enable_training = enable
    if (self._enable_training):
      self.reset()
    return

  enable_training = property(get_enable_training, set_enable_training)

  def enable_testing(self):
    return self.test_episodes > 0

  def get_name(self):
    return self.NAME

  @abstractmethod
  def save_model(self, out_path):
    pass

  @abstractmethod
  def load_model(self, in_path):
    pass

  @abstractmethod
  def _decide_action(self, s, g):
    pass

  @abstractmethod
  def _get_output_path(self):
    pass

  @abstractmethod
  def _get_int_output_path(self):
    pass

  @abstractmethod
  def _train_step(self):
    pass

  @abstractmethod
  def _check_action_space(self):
    pass

  def get_action_space(self):
    return self.world.env.get_action_space(self.id)

  def get_state_size(self):
    return self.world.env.get_state_size(self.id)

  def get_goal_size(self):
    return self.world.env.get_goal_size(self.id)

  def get_action_size(self):
    return self.world.env.get_action_size(self.id)

  def get_num_actions(self):
    return self.world.env.get_num_actions(self.id)

  def need_new_action(self):
    return self.world.env.need_new_action(self.id)

  def _build_normalizers(self):
    self.s_norm = Normalizer(self.get_state_size(),
                             self.world.env.build_state_norm_groups(self.id))
    self.s_norm.set_mean_std(-self.world.env.build_state_offset(self.id),
                             1 / self.world.env.build_state_scale(self.id))

    self.g_norm = Normalizer(self.get_goal_size(), self.world.env.build_goal_norm_groups(self.id))
    self.g_norm.set_mean_std(-self.world.env.build_goal_offset(self.id),
                             1 / self.world.env.build_goal_scale(self.id))

    self.a_norm = Normalizer(self.world.env.get_action_size())
    self.a_norm.set_mean_std(-self.world.env.build_action_offset(self.id),
                             1 / self.world.env.build_action_scale(self.id))
    return

  def _build_bounds(self):
    self.a_bound_min = self.world.env.build_action_bound_min(self.id)
    self.a_bound_max = self.world.env.build_action_bound_max(self.id)
    return

  def _load_params(self, json_data):
    if (self.UPDATE_PERIOD_KEY in json_data):
      self.update_period = int(json_data[self.UPDATE_PERIOD_KEY])

    if (self.ITERS_PER_UPDATE in json_data):
      self.iters_per_update = int(json_data[self.ITERS_PER_UPDATE])

    if (self.DISCOUNT_KEY in json_data):
      self.discount = json_data[self.DISCOUNT_KEY]

    if (self.MINI_BATCH_SIZE_KEY in json_data):
      self.mini_batch_size = int(json_data[self.MINI_BATCH_SIZE_KEY])

    if (self.REPLAY_BUFFER_SIZE_KEY in json_data):
      self.replay_buffer_size = int(json_data[self.REPLAY_BUFFER_SIZE_KEY])

    if (self.INIT_SAMPLES_KEY in json_data):
      self.init_samples = int(json_data[self.INIT_SAMPLES_KEY])

    if (self.NORMALIZER_SAMPLES_KEY in json_data):
      self.normalizer_samples = int(json_data[self.NORMALIZER_SAMPLES_KEY])

    if (self.OUTPUT_ITERS_KEY in json_data):
      self.output_iters = json_data[self.OUTPUT_ITERS_KEY]

    if (self.INT_OUTPUT_ITERS_KEY in json_data):
      self.int_output_iters = json_data[self.INT_OUTPUT_ITERS_KEY]

    if (self.TEST_EPISODES_KEY in json_data):
      self.test_episodes = int(json_data[self.TEST_EPISODES_KEY])

    if (self.EXP_ANNEAL_SAMPLES_KEY in json_data):
      self.exp_anneal_samples = json_data[self.EXP_ANNEAL_SAMPLES_KEY]

    if (self.EXP_PARAM_BEG_KEY in json_data):
      self.exp_params_beg.load(json_data[self.EXP_PARAM_BEG_KEY])

    if (self.EXP_PARAM_END_KEY in json_data):
      self.exp_params_end.load(json_data[self.EXP_PARAM_END_KEY])

    num_procs = MPIUtil.get_num_procs()
    self._local_mini_batch_size = int(np.ceil(self.mini_batch_size / num_procs))
    self._local_mini_batch_size = np.maximum(self._local_mini_batch_size, 1)
    self.mini_batch_size = self._local_mini_batch_size * num_procs

    assert (self.exp_params_beg.noise == self.exp_params_end.noise)  # noise std should not change
    self.exp_params_curr = copy.deepcopy(self.exp_params_beg)
    self.exp_params_end.noise = self.exp_params_beg.noise

    self._need_normalizer_update = self.normalizer_samples > 0

    return

  def _record_state(self):
    s = self.world.env.record_state(self.id)
    return s

  def _record_goal(self):
    g = self.world.env.record_goal(self.id)
    return g

  def _record_reward(self):
    r = self.world.env.calc_reward(self.id)
    return r

  def _apply_action(self, a):
    self.world.env.set_action(self.id, a)
    return

  def _record_flags(self):
    return int(0)

  def _is_first_step(self):
    return len(self.path.states) == 0

  def _end_path(self):
    s = self._record_state()
    g = self._record_goal()
    r = self._record_reward()

    self.path.rewards.append(r)
    self.path.states.append(s)
    self.path.goals.append(g)
    self.path.terminate = self.world.env.check_terminate(self.id)

    return

  def _update_new_action(self):
    #print("_update_new_action!")
    s = self._record_state()
    #np.savetxt("pb_record_state_s.csv", s, delimiter=",")
    g = self._record_goal()

    if not (self._is_first_step()):
      r = self._record_reward()
      self.path.rewards.append(r)

    a, logp = self._decide_action(s=s, g=g)
    assert len(np.shape(a)) == 1
    assert len(np.shape(logp)) <= 1

    flags = self._record_flags()
    self._apply_action(a)

    self.path.states.append(s)
    self.path.goals.append(g)
    self.path.actions.append(a)
    self.path.logps.append(logp)
    self.path.flags.append(flags)

    if self._enable_draw():
      self._log_val(s, g)

    return

  def _update_exp_params(self):
    lerp = float(self._total_sample_count) / self.exp_anneal_samples
    lerp = np.clip(lerp, 0.0, 1.0)
    self.exp_params_curr = self.exp_params_beg.lerp(self.exp_params_end, lerp)
    return

  def _update_test_return(self, path):
    path_reward = path.calc_return()
    self.test_return += path_reward
    self.test_episode_count += 1
    return

  def _update_mode(self):
    if (self._mode == self.Mode.TRAIN):
      self._update_mode_train()
    elif (self._mode == self.Mode.TRAIN_END):
      self._update_mode_train_end()
    elif (self._mode == self.Mode.TEST):
      self._update_mode_test()
    else:
      assert False, Logger.print2("Unsupported RL agent mode" + str(self._mode))
    return

  def _update_mode_train(self):
    return

  def _update_mode_train_end(self):
    self._init_mode_test()
    return

  def _update_mode_test(self):
    if (self.test_episode_count * MPIUtil.get_num_procs() >= self.test_episodes):
      global_return = MPIUtil.reduce_sum(self.test_return)
      global_count = MPIUtil.reduce_sum(self.test_episode_count)
      avg_return = global_return / global_count
      self.avg_test_return = avg_return

      if self.enable_training:
        self._init_mode_train()
    return

  def _init_mode_train(self):
    self._mode = self.Mode.TRAIN
    self.world.env.set_mode(self._mode)
    return

  def _init_mode_train_end(self):
    self._mode = self.Mode.TRAIN_END
    return

  def _init_mode_test(self):
    self._mode = self.Mode.TEST
    self.test_return = 0.0
    self.test_episode_count = 0
    self.world.env.set_mode(self._mode)
    return

  def _enable_output(self):
    return MPIUtil.is_root_proc() and self.output_dir != ""

  def _enable_int_output(self):
    return MPIUtil.is_root_proc() and self.int_output_dir != ""

  def _calc_val_bounds(self, discount):
    r_min = self.world.env.get_reward_min(self.id)
    r_max = self.world.env.get_reward_max(self.id)
    assert (r_min <= r_max)

    val_min = r_min / (1.0 - discount)
    val_max = r_max / (1.0 - discount)
    return val_min, val_max

  def _calc_val_offset_scale(self, discount):
    val_min, val_max = self._calc_val_bounds(discount)
    val_offset = 0
    val_scale = 1

    if (np.isfinite(val_min) and np.isfinite(val_max)):
      val_offset = -0.5 * (val_max + val_min)
      val_scale = 2 / (val_max - val_min)

    return val_offset, val_scale

  def _calc_term_vals(self, discount):
    r_fail = self.world.env.get_reward_fail(self.id)
    r_succ = self.world.env.get_reward_succ(self.id)

    r_min = self.world.env.get_reward_min(self.id)
    r_max = self.world.env.get_reward_max(self.id)
    assert (r_fail <= r_max and r_fail >= r_min)
    assert (r_succ <= r_max and r_succ >= r_min)
    assert (not np.isinf(r_fail))
    assert (not np.isinf(r_succ))

    if (discount == 0):
      val_fail = 0
      val_succ = 0
    else:
      val_fail = r_fail / (1.0 - discount)
      val_succ = r_succ / (1.0 - discount)

    return val_fail, val_succ

  def _update_iter(self, iter):
    if (self._enable_output() and self.iter % self.output_iters == 0):
      output_path = self._get_output_path()
      output_dir = os.path.dirname(output_path)
      if not os.path.exists(output_dir):
        os.makedirs(output_dir)
      self.save_model(output_path)

    if (self._enable_int_output() and self.iter % self.int_output_iters == 0):
      int_output_path = self._get_int_output_path()
      int_output_dir = os.path.dirname(int_output_path)
      if not os.path.exists(int_output_dir):
        os.makedirs(int_output_dir)
      self.save_model(int_output_path)

    self.iter = iter
    return

  def _enable_draw(self):
    return self.world.env.enable_draw

  def _log_val(self, s, g):
    pass

  def _build_replay_buffer(self, buffer_size):
    num_procs = MPIUtil.get_num_procs()
    buffer_size = int(buffer_size / num_procs)
    self.replay_buffer = ReplayBuffer(buffer_size=buffer_size)
    self.replay_buffer_initialized = False
    return

  def _store_path(self, path):
    path_id = self.replay_buffer.store(path)
    valid_path = path_id != MathUtil.INVALID_IDX

    if valid_path:
      self.train_return = path.calc_return()

      if self._need_normalizer_update:
        self._record_normalizers(path)

    return path_id

  def _record_normalizers(self, path):
    states = np.array(path.states)
    self.s_norm.record(states)

    if self.has_goal():
      goals = np.array(path.goals)
      self.g_norm.record(goals)

    return

  def _update_normalizers(self):
    self.s_norm.update()

    if self.has_goal():
      self.g_norm.update()
    return

  def _train(self):
    samples = self.replay_buffer.total_count
    self._total_sample_count = int(MPIUtil.reduce_sum(samples))
    end_training = False

    if (self.replay_buffer_initialized):
      if (self._valid_train_step()):
        prev_iter = self.iter
        iters = self._get_iters_per_update()
        avg_train_return = MPIUtil.reduce_avg(self.train_return)

        for i in range(iters):
          curr_iter = self.iter
          wall_time = time.time() - self.start_time
          wall_time /= 60 * 60  # store time in hours

          has_goal = self.has_goal()
          s_mean = np.mean(self.s_norm.mean)
          s_std = np.mean(self.s_norm.std)
          g_mean = np.mean(self.g_norm.mean) if has_goal else 0
          g_std = np.mean(self.g_norm.std) if has_goal else 0

          self.logger.log_tabular("Iteration", self.iter)
          self.logger.log_tabular("Wall_Time", wall_time)
          self.logger.log_tabular("Samples", self._total_sample_count)
          self.logger.log_tabular("Train_Return", avg_train_return)
          self.logger.log_tabular("Test_Return", self.avg_test_return)
          self.logger.log_tabular("State_Mean", s_mean)
          self.logger.log_tabular("State_Std", s_std)
          self.logger.log_tabular("Goal_Mean", g_mean)
          self.logger.log_tabular("Goal_Std", g_std)
          self._log_exp_params()

          self._update_iter(self.iter + 1)
          self._train_step()

          Logger.print2("Agent " + str(self.id))
          self.logger.print_tabular()
          Logger.print2("")

          if (self._enable_output() and curr_iter % self.int_output_iters == 0):
            self.logger.dump_tabular()

        if (prev_iter // self.int_output_iters != self.iter // self.int_output_iters):
          end_training = self.enable_testing()

    else:

      Logger.print2("Agent " + str(self.id))
      Logger.print2("Samples: " + str(self._total_sample_count))
      Logger.print2("")

      if (self._total_sample_count >= self.init_samples):
        self.replay_buffer_initialized = True
        end_training = self.enable_testing()

    if self._need_normalizer_update:
      self._update_normalizers()
      self._need_normalizer_update = self.normalizer_samples > self._total_sample_count

    if end_training:
      self._init_mode_train_end()

    return

  def _get_iters_per_update(self):
    return MPIUtil.get_num_procs() * self.iters_per_update

  def _valid_train_step(self):
    return True

  def _log_exp_params(self):
    self.logger.log_tabular("Exp_Rate", self.exp_params_curr.rate)
    self.logger.log_tabular("Exp_Noise", self.exp_params_curr.noise)
    self.logger.log_tabular("Exp_Temp", self.exp_params_curr.temp)
    return
