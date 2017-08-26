from baselines import deepq


def add_opts(parser):
	pass


class BaselinesDQNAgent(object):
	'''
	classdocs
	'''

	def __init__(self, opts):
		self.metadata = {
			'discrete_actions': True,
		}

		self.opts = opts
		self.agent = None

	def configure(self, observation_space_shape, nb_actions):
		pass

	def train(self, env, nb_steps, visualize, verbosity):
		model = deepq.models.mlp([64])
		self.agent = deepq.learn(
			env,
			q_func=model,
			lr=1e-3,
			max_timesteps=nb_steps,
			buffer_size=50000,
			exploration_fraction=0.1,
			exploration_final_eps=0.02,
			print_freq=10 if verbosity else None,
			callback=env.render if visualize else None
		)

	def test(self, env, nb_episodes, visualize):
		episodes = 0
		while episodes < nb_episodes:
			obs, done = env.reset(), False
			episode_rew = 0
			while not done:
				if visualize:
					env.render()
				obs, rew, done, _ = env.step(self.agent(obs[None])[0])
				episode_rew += rew
			print("Episode reward", episode_rew)
			episodes += 1

	def load_weights(self, load_file):
		self.agent = deepq.load(load_file)

	def save_weights(self, save_file, overwrite):
		self.agent.save(save_file)