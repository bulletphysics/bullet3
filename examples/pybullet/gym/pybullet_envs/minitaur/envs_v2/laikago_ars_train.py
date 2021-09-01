'''
Parallel implementation of the Augmented Random Search method.
Horia Mania --- hmania@berkeley.edu
Aurelia Guy
Benjamin Recht 
'''

import parser
import time
import os
import numpy as np
import gym

import arspb.logz as logz
import ray
import arspb.utils as utils
import arspb.optimizers as optimizers
from arspb.policies import *
import socket
from arspb.shared_noise import *

##############################
#create an envs_v2 laikago env

import os

import gin
from pybullet_envs.minitaur.envs_v2 import env_loader
import pybullet_data as pd



def create_laikago_env():
  CONFIG_DIR = pd.getDataPath()+"/configs_v2/"
  CONFIG_FILES = [
      os.path.join(CONFIG_DIR, "base/laikago_with_imu.gin"),
      os.path.join(CONFIG_DIR, "tasks/fwd_task_no_termination.gin"),
      os.path.join(CONFIG_DIR, "wrappers/pmtg_wrapper.gin"),
      os.path.join(CONFIG_DIR, "scenes/simple_scene.gin")
  ]
  
  #gin.bind_parameter("scene_base.SceneBase.data_root", pd.getDataPath()+"/")
  for gin_file in CONFIG_FILES:
    gin.parse_config_file(gin_file)
  gin.bind_parameter("SimulationParameters.enable_rendering", False)
  gin.bind_parameter("terminal_conditions.maxstep_terminal_condition.max_step",
                     10000)
  env = env_loader.load()
  return env
  
  
##############################

@ray.remote
class Worker(object):
    """ 
    Object class for parallel rollout generation.
    """

    def __init__(self, env_seed,
                 env_name='',
                 policy_params = None,
                 deltas=None,
                 rollout_length=1000,
                 delta_std=0.01):

        # initialize OpenAI environment for each worker
        try:
          import pybullet_envs
        except:
          pass
        try:
          import tds_environments
        except:
          pass

        self.env = create_laikago_env()
        self.env.seed(env_seed)

        # each worker gets access to the shared noise table
        # with independent random streams for sampling
        # from the shared noise table. 
        self.deltas = SharedNoiseTable(deltas, env_seed + 7)
        self.policy_params = policy_params
        if policy_params['type'] == 'linear':
            print("LinearPolicy2")
            self.policy = LinearPolicy2(policy_params)
        elif policy_params['type'] == 'nn':
            print("FullyConnectedNeuralNetworkPolicy")
            self.policy = FullyConnectedNeuralNetworkPolicy(policy_params)
        else:
            raise NotImplementedError
            
        self.delta_std = delta_std
        self.rollout_length = rollout_length

        
    def get_weights_plus_stats(self):
        """ 
        Get current policy weights and current statistics of past states.
        """
        #assert self.policy_params['type'] == 'linear'
        return self.policy.get_weights_plus_stats()
    

    def rollout(self, shift = 0., rollout_length = None):
        """ 
        Performs one rollout of maximum length rollout_length. 
        At each time-step it substracts shift from the reward.
        """
        
        if rollout_length is None:
            rollout_length = self.rollout_length

        total_reward = 0.
        steps = 0

        ob = self.env.reset()
        for i in range(rollout_length):
            action = self.policy.act(ob)
            ob, reward, done, _ = self.env.step(action)
            steps += 1
            total_reward += (reward - shift)
            if done:
                break
            
        return total_reward, steps

    def do_rollouts(self, w_policy, num_rollouts = 1, shift = 1, evaluate = False):
        """ 
        Generate multiple rollouts with a policy parametrized by w_policy.
        """

        rollout_rewards, deltas_idx = [], []
        steps = 0

        for i in range(num_rollouts):

            if evaluate:
                self.policy.update_weights(w_policy)
                deltas_idx.append(-1)
                
                # set to false so that evaluation rollouts are not used for updating state statistics
                self.policy.update_filter = False

                # for evaluation we do not shift the rewards (shift = 0) and we use the
                # default rollout length (1000 for the MuJoCo locomotion tasks)
                reward, r_steps = self.rollout(shift = 0., rollout_length = self.rollout_length)
                rollout_rewards.append(reward)
                
            else:
                idx, delta = self.deltas.get_delta(w_policy.size)
             
                delta = (self.delta_std * delta).reshape(w_policy.shape)
                deltas_idx.append(idx)

                # set to true so that state statistics are updated 
                self.policy.update_filter = True

                # compute reward and number of timesteps used for positive perturbation rollout
                self.policy.update_weights(w_policy + delta)
                pos_reward, pos_steps  = self.rollout(shift = shift)

                # compute reward and number of timesteps used for negative pertubation rollout
                self.policy.update_weights(w_policy - delta)
                neg_reward, neg_steps = self.rollout(shift = shift) 
                steps += pos_steps + neg_steps

                rollout_rewards.append([pos_reward, neg_reward])
                            
        return {'deltas_idx': deltas_idx, 'rollout_rewards': rollout_rewards, "steps" : steps}
    
    def stats_increment(self):
        self.policy.observation_filter.stats_increment()
        return

    def get_weights(self):
        return self.policy.get_weights()
    
    def get_filter(self):
        return self.policy.observation_filter

    def sync_filter(self, other):
        self.policy.observation_filter.sync(other)
        return

    
class ARSLearner(object):
    """ 
    Object class implementing the ARS algorithm.
    """

    def __init__(self, env_name='HalfCheetah-v1',
                 policy_params=None,
                 num_workers=32, 
                 num_deltas=320, 
                 deltas_used=320,
                 delta_std=0.01, 
                 logdir=None, 
                 rollout_length=4000,
                 step_size=0.01,
                 shift='constant zero',
                 params=None,
                 seed=123):

        logz.configure_output_dir(logdir)
        logz.save_params(params)
        try:
          import pybullet_envs
        except:
          pass
        try:
          import tds_environments
        except:
          pass

        env = create_laikago_env()
        
        self.timesteps = 0
        self.action_size = env.action_space.shape[0]
        self.ob_size = env.observation_space.shape[0]
        self.num_deltas = num_deltas
        self.deltas_used = deltas_used
        self.rollout_length = rollout_length
        self.step_size = step_size
        self.delta_std = delta_std
        self.logdir = logdir
        self.shift = shift
        self.params = params
        self.max_past_avg_reward = float('-inf')
        self.num_episodes_used = float('inf')

        
        # create shared table for storing noise
        print("Creating deltas table.")
        deltas_id = create_shared_noise.remote()
        self.deltas = SharedNoiseTable(ray.get(deltas_id), seed = seed + 3)
        print('Created deltas table.')

        # initialize workers with different random seeds
        print('Initializing workers.') 
        self.num_workers = num_workers
        self.workers = [Worker.remote(seed + 7 * i,
                                      env_name=env_name,
                                      policy_params=policy_params,
                                      deltas=deltas_id,
                                      rollout_length=rollout_length,
                                      delta_std=delta_std) for i in range(num_workers)]


        # initialize policy 
        if policy_params['type'] == 'linear':
            print("LinearPolicy2")
            self.policy = LinearPolicy2(policy_params)
            self.w_policy = self.policy.get_weights()
        elif policy_params['type'] == 'nn':
            print("FullyConnectedNeuralNetworkPolicy")
            self.policy = FullyConnectedNeuralNetworkPolicy(policy_params)
            self.w_policy = self.policy.get_weights()
        else:
            raise NotImplementedError
            
        # initialize optimization algorithm
        self.optimizer = optimizers.SGD(self.w_policy, self.step_size)        
        print("Initialization of ARS complete.")

    def aggregate_rollouts(self, num_rollouts = None, evaluate = False):
        """ 
        Aggregate update step from rollouts generated in parallel.
        """

        if num_rollouts is None:
            num_deltas = self.num_deltas
        else:
            num_deltas = num_rollouts
            
        # put policy weights in the object store
        policy_id = ray.put(self.w_policy)

        t1 = time.time()
        num_rollouts = int(num_deltas / self.num_workers)
            
        # parallel generation of rollouts
        rollout_ids_one = [worker.do_rollouts.remote(policy_id,
                                                 num_rollouts = num_rollouts,
                                                 shift = self.shift,
                                                 evaluate=evaluate) for worker in self.workers]

        rollout_ids_two = [worker.do_rollouts.remote(policy_id,
                                                 num_rollouts = 1,
                                                 shift = self.shift,
                                                 evaluate=evaluate) for worker in self.workers[:(num_deltas % self.num_workers)]]

        # gather results 
        results_one = ray.get(rollout_ids_one)
        results_two = ray.get(rollout_ids_two)

        rollout_rewards, deltas_idx = [], [] 

        for result in results_one:
            if not evaluate:
                self.timesteps += result["steps"]
            deltas_idx += result['deltas_idx']
            rollout_rewards += result['rollout_rewards']

        for result in results_two:
            if not evaluate:
                self.timesteps += result["steps"]
            deltas_idx += result['deltas_idx']
            rollout_rewards += result['rollout_rewards']

        deltas_idx = np.array(deltas_idx)
        rollout_rewards = np.array(rollout_rewards, dtype = np.float64)
        
        print('Maximum reward of collected rollouts:', rollout_rewards.max())
        t2 = time.time()

        print('Time to generate rollouts:', t2 - t1)

        if evaluate:
            return rollout_rewards

        # select top performing directions if deltas_used < num_deltas
        max_rewards = np.max(rollout_rewards, axis = 1)
        if self.deltas_used > self.num_deltas:
            self.deltas_used = self.num_deltas
            
        idx = np.arange(max_rewards.size)[max_rewards >= np.percentile(max_rewards, 100*(1 - (self.deltas_used / self.num_deltas)))]
        deltas_idx = deltas_idx[idx]
        rollout_rewards = rollout_rewards[idx,:]
        
        # normalize rewards by their standard deviation
        np_std = np.std(rollout_rewards)
        if np_std>1e-6:
          rollout_rewards /= np_std

        t1 = time.time()
        # aggregate rollouts to form g_hat, the gradient used to compute SGD step
        g_hat, count = utils.batched_weighted_sum(rollout_rewards[:,0] - rollout_rewards[:,1],
                                                  (self.deltas.get(idx, self.w_policy.size)
                                                   for idx in deltas_idx),
                                                  batch_size = 500)
        g_hat /= deltas_idx.size
        t2 = time.time()
        print('time to aggregate rollouts', t2 - t1)
        return g_hat
        

    def train_step(self):
        """ 
        Perform one update step of the policy weights.
        """
        
        g_hat = self.aggregate_rollouts()                    
        print("Euclidean norm of update step:", np.linalg.norm(g_hat))
        self.w_policy -= self.optimizer._compute_step(g_hat).reshape(self.w_policy.shape)
        return

    def train(self, num_iter):

        start = time.time()
        best_mean_rewards = -1e30
        
        for i in range(num_iter):
            
            t1 = time.time()
            self.train_step()
            t2 = time.time()
            print('total time of one step', t2 - t1)           
            print('iter ', i,' done')

            # record statistics every 10 iterations
            if ((i + 1) % 10 == 0):
                
                rewards = self.aggregate_rollouts(num_rollouts = 100, evaluate = True)
                w = ray.get(self.workers[0].get_weights_plus_stats.remote())
                np.savez(self.logdir + "/lin_policy_plus_latest", w)
                
                mean_rewards = np.mean(rewards)
                if (mean_rewards > best_mean_rewards):
                  best_mean_rewards = mean_rewards
                  np.savez(self.logdir + "/lin_policy_plus_best_"+str(i+1), w)
                  
                
                print(sorted(self.params.items()))
                logz.log_tabular("Time", time.time() - start)
                logz.log_tabular("Iteration", i + 1)
                logz.log_tabular("AverageReward", np.mean(rewards))
                logz.log_tabular("StdRewards", np.std(rewards))
                logz.log_tabular("MaxRewardRollout", np.max(rewards))
                logz.log_tabular("MinRewardRollout", np.min(rewards))
                logz.log_tabular("timesteps", self.timesteps)
                logz.dump_tabular()
                
            t1 = time.time()
            # get statistics from all workers
            for j in range(self.num_workers):
                self.policy.observation_filter.update(ray.get(self.workers[j].get_filter.remote()))
            self.policy.observation_filter.stats_increment()

            # make sure master filter buffer is clear
            self.policy.observation_filter.clear_buffer()
            # sync all workers
            filter_id = ray.put(self.policy.observation_filter)
            setting_filters_ids = [worker.sync_filter.remote(filter_id) for worker in self.workers]
            # waiting for sync of all workers
            ray.get(setting_filters_ids)
         
            increment_filters_ids = [worker.stats_increment.remote() for worker in self.workers]
            # waiting for increment of all workers
            ray.get(increment_filters_ids)            
            t2 = time.time()
            print('Time to sync statistics:', t2 - t1)
                        
        return 

def run_ars(params):
    dir_path = params['dir_path']
    
    if not(os.path.exists(dir_path)):
        os.makedirs(dir_path)
    logdir = dir_path
    if not(os.path.exists(logdir)):
        os.makedirs(logdir)

    try:
      import pybullet_envs
    except:
      pass
    try:
      import tds_environments
    except:
      pass
    env = create_laikago_env()
    ob_dim = env.observation_space.shape[0]
    ac_dim = env.action_space.shape[0]
    ac_lb = env.action_space.low
    ac_ub = env.action_space.high

    # set policy parameters. Possible filters: 'MeanStdFilter' for v2, 'NoFilter' for v1.
    if params["policy_type"]=="nn":
      policy_sizes_string = params['policy_network_size_list'].split(',')
      print("policy_sizes_string=",policy_sizes_string)
      policy_sizes_list = [int(item) for item in policy_sizes_string]
      print("policy_sizes_list=",policy_sizes_list)
      activation = params['activation']
      policy_params={'type': params["policy_type"],
                     'ob_filter':params['filter'],
                     'policy_network_size' : policy_sizes_list,
                     'ob_dim':ob_dim,
                     'ac_dim':ac_dim,
                     'activation' : activation,
                     'action_lower_bound' : ac_lb,
                     'action_upper_bound' : ac_ub,
      }
    else:
      del params['policy_network_size_list']
      del params['activation']
      policy_params={'type': params["policy_type"],
                     'ob_filter':params['filter'],
                     'ob_dim':ob_dim,
                     'ac_dim':ac_dim,
                     'action_lower_bound' : ac_lb,
                     'action_upper_bound' : ac_ub,
      }
    
    
    ARS = ARSLearner(env_name=params['env_name'],
                     policy_params=policy_params,
                     num_workers=params['n_workers'], 
                     num_deltas=params['n_directions'],
                     deltas_used=params['deltas_used'],
                     step_size=params['step_size'],
                     delta_std=params['delta_std'], 
                     logdir=logdir,
                     rollout_length=params['rollout_length'],
                     shift=params['shift'],
                     params=params,
                     seed = params['seed'])
        
    ARS.train(params['n_iter'])
       
    return 


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--env_name', type=str, default='InvertedPendulumSwingupBulletEnv-v0')
    parser.add_argument('--n_iter', '-n', type=int, default=1000)
    parser.add_argument('--n_directions', '-nd', type=int, default=16)
    parser.add_argument('--deltas_used', '-du', type=int, default=16)
    parser.add_argument('--step_size', '-s', type=float, default=0.03)
    parser.add_argument('--delta_std', '-std', type=float, default=.03)
    parser.add_argument('--n_workers', '-e', type=int, default=18)
    parser.add_argument('--rollout_length', '-r', type=int, default=2000)

    # for Swimmer-v1 and HalfCheetah-v1 use shift = 0
    # for Hopper-v1, Walker2d-v1, and Ant-v1 use shift = 1
    # for Humanoid-v1 used shift = 5
    parser.add_argument('--shift', type=float, default=0)
    parser.add_argument('--seed', type=int, default=37)
    parser.add_argument('--policy_type', type=str, help="Policy type, linear or nn (neural network)", default= 'linear')
    parser.add_argument('--dir_path', type=str, default='data')

    # for ARS V1 use filter = 'NoFilter'
    parser.add_argument('--filter', type=str, default='MeanStdFilter')
    parser.add_argument('--activation', type=str, help="Neural network policy activation function, tanh or clip", default="tanh")
    parser.add_argument('--policy_network_size', action='store', dest='policy_network_size_list',type=str, nargs='*', default='64,64')
    


    

    local_ip = socket.gethostbyname(socket.gethostname())
    ray.init(address= local_ip + ':6379')
    
    args = parser.parse_args()
    params = vars(args)
    run_ars(params)

