# with some extra arg parsing

from keras.models import Sequential, Model # The Sequential model is a sequential, feed-forward stack of layers.
from keras.layers import Dense, Activation, Flatten, Input, merge # Different types of layers
from keras.optimizers import Adam # A special type of optimizer

from rl.agents.cem import CEMAgent
from rl.memory import EpisodeParameterMemory

from rl.agents import DDPGAgent
from rl.memory import SequentialMemory  # A first-in-first-out type of memory to do the experience replay on
from rl.random import OrnsteinUhlenbeckProcess  # a noise process

from rl.agents.dqn import DQNAgent
from rl.policy import BoltzmannQPolicy # Instead of random actions, we tend to pick actions that have generated rewards before. As time goes on we only focus on one or two actions in each state.
from rl.memory import SequentialMemory # A first-in-first-out type of memory to do the experience replay on

from rl.agents import ContinuousDQNAgent

def add_opts(parser):
	parser.add_argument('--model-type', type=int, default=1,
						help="the dense-softmax-layer model (1) or the deep network (2)")


class KerasCEMAgent(object):
	'''
	The cross-entropy method Learning Agent as described in http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.81.6579&rep=rep1&type=pdf
	'''

	def __init__(self, opts):
		self.metadata = {
			'discrete_actions': True,
		}

		self.opts = opts

	def configure(self, observation_space_shape, nb_actions):
		if self.opts.model_type == 1:
			# Option 1 : Simple model
			model = Sequential()
			model.add(Flatten(input_shape=(1,) + observation_space_shape))
			model.add(Dense(nb_actions))
			model.add(Activation('softmax'))
			print(model.summary())
		elif self.opts.model_type == 2:
			# Option 2: deep network
			model = Sequential()
			model.add(Flatten(input_shape=(1,) + observation_space_shape))
			model.add(Dense(16))
			model.add(Activation('relu'))
			model.add(Dense(16))
			model.add(Activation('relu'))
			model.add(Dense(16))
			model.add(Activation('relu'))
			model.add(Dense(nb_actions))
			model.add(Activation('softmax'))
			print(model.summary())

		# Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
		# even the metrics!
		memory = EpisodeParameterMemory(limit=1000, window_length=1)

		self.agent = CEMAgent(model=model, nb_actions=nb_actions, memory=memory,
							  batch_size=50, nb_steps_warmup=2000, train_interval=50, elite_frac=0.05)
		self.agent.compile()

	def train(self, env, nb_steps, visualize, verbosity):
		# Okay, now it's time to learn something! We visualize the training here for show, but this
		# slows down training quite a lot. You can always safely abort the training prematurely using
		# Ctrl + C.
		self.agent.fit(env, nb_steps=nb_steps, visualize=visualize, verbose=verbosity)

	def test(self, env, nb_episodes, visualize):
		# Finally, evaluate our algorithm for 5 episodes.
		self.agent.test(env, nb_episodes=nb_episodes, visualize=visualize)

	def load_weights(self, load_file):
		self.agent.load_weights(load_file)

	def save_weights(self, save_file, overwrite):
		# After training is done, we save the best weights.
		self.agent.save_weights(save_file, overwrite=overwrite)

class KerasDDPGAgent(object):
	'''
	The Deep Differential Policy Gradient Learning Agent as described in http://arxiv.org/abs/1509.02971
	'''

	def __init__(self, opts):
		self.metadata = {
			'discrete_actions': False,
		}
		self.opts = opts

	def configure(self, observation_space_shape, nb_actions):
		# Next, we build a simple model.
		# actor network
		actor = Sequential()
		actor.add(Flatten(input_shape=(1,) + observation_space_shape))
		actor.add(Dense(16))
		actor.add(Activation('relu'))
		actor.add(Dense(16))
		actor.add(Activation('relu'))
		actor.add(Dense(16))
		actor.add(Activation('relu'))
		actor.add(Dense(nb_actions))
		actor.add(Activation('linear'))
		print(actor.summary())

		# critic network
		action_input = Input(shape=(nb_actions,), name='action_input')
		observation_input = Input(shape=(1,) + observation_space_shape, name='observation_input')
		flattened_observation = Flatten()(observation_input)
		x = merge([action_input, flattened_observation], mode='concat')
		x = Dense(32)(x)
		x = Activation('relu')(x)
		x = Dense(32)(x)
		x = Activation('relu')(x)
		x = Dense(32)(x)
		x = Activation('relu')(x)
		x = Dense(1)(x)
		x = Activation('linear')(x)
		critic = Model(input=[action_input, observation_input], output=x)
		print(critic.summary())

		# Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
		# even the metrics!
		memory = SequentialMemory(limit=100000, window_length=1)
		random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=.15, mu=0., sigma=.3)
		self.agent = DDPGAgent(nb_actions=nb_actions, actor=actor, critic=critic, critic_action_input=action_input,
							   memory=memory, nb_steps_warmup_critic=100, nb_steps_warmup_actor=100,
							   random_process=random_process, gamma=.99, target_model_update=1e-3)
		self.agent.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])

	def train(self, env, nb_steps, visualize, verbosity):
		# Okay, now it's time to learn something! We visualize the training here for show, but this
		# slows down training quite a lot. You can always safely abort the training prematurely using
		# Ctrl + C.
		self.agent.fit(env, nb_steps=nb_steps, visualize=visualize, verbose=verbosity, nb_max_episode_steps=200)

	def test(self, env, nb_episodes, visualize):
		# Finally, evaluate our algorithm for 5 episodes.
		self.agent.test(env, nb_episodes=nb_episodes, visualize=visualize, nb_max_episode_steps=200)

	def load_weights(self, load_file):
		self.agent.load_weights(load_file)

	def save_weights(self, save_file, overwrite):
		self.agent.save_weights(save_file, overwrite=True)


class KerasDDQNAgent(object):
	'''
	The deep Double Q Learning Agent as described in https://arxiv.org/abs/1509.06461
	'''

	def __init__(self, opts):
		self.metadata = {
			'discrete_actions': True,
		}

		self.opts = opts

	def configure(self, observation_space_shape, nb_actions):
		# Next, we build a simple model.
		model = Sequential()
		model.add(Flatten(input_shape=(1,) + observation_space_shape))  # input layer
		model.add(Dense(32))  # Just your regular fully connected NN layer
		model.add(Activation('tanh'))  # tanh activation layer
		model.add(Dense(16))  # more model capacity through fully connected NN layers
		model.add(Activation('relu'))  # Rectified Linear Units
		model.add(Dense(16))  # more model capacity through fully connected NN layers
		model.add(Activation('relu'))  # Rectified Linear Units
		model.add(Dense(nb_actions))  # fully connected NN layer with one output for each action
		model.add(Activation('linear'))  # we want linear activations in the end
		print(model.summary())

		memory = SequentialMemory(limit=50000, window_length=1)
		policy = BoltzmannQPolicy()
		self.agent = DQNAgent(enable_double_dqn=True, model=model, nb_actions=nb_actions, memory=memory,
							  nb_steps_warmup=10,
							  target_model_update=1e-2, policy=policy)
		self.agent.compile(Adam(lr=1e-3), metrics=['mae'])

	def train(self, env, nb_steps, visualize, verbosity):
		self.agent.fit(env, nb_steps=nb_steps, visualize=visualize, verbose=verbosity)

	def test(self, env, nb_episodes, visualize):
		self.agent.test(env, nb_episodes=nb_episodes, visualize=visualize)

	def load_weights(self, load_file):
		self.agent.load_weights(load_file)

	def save_weights(self, save_file, overwrite):
		self.agent.save_weights(save_file, overwrite)


class KerasDQNAgent(object):
	'''
	The deep Q Learning Agent as described in https://www.cs.toronto.edu/~vmnih/docs/dqn.pdf
	'''

	def __init__(self, opts):
		self.metadata = {
			'discrete_actions': True,
		}
		self.opts = opts

	def configure(self, observation_space_shape, nb_actions):
		# Next, we build a simple model.
		model = Sequential()
		model.add(Flatten(input_shape=(1,) + observation_space_shape))  # input layer
		model.add(Dense(32))  # Just your regular fully connected NN layer
		model.add(Activation('tanh'))  # tanh activation layer
		model.add(Dense(16))  # more model capacity through fully connected NN layers
		model.add(Activation('relu'))  # Rectified Linear Units
		model.add(Dense(16))  # more model capacity through fully connected NN layers
		model.add(Activation('relu'))  # Rectified Linear Units
		model.add(Dense(nb_actions))  # fully connected NN layer with one output for each action
		model.add(Activation('linear'))  # we want linear activations in the end
		print(model.summary())

		memory = SequentialMemory(limit=50000, window_length=1)
		policy = BoltzmannQPolicy()
		self.agent = DQNAgent(enable_double_dqn=False, model=model, nb_actions=nb_actions, memory=memory,
							  nb_steps_warmup=10,
							  target_model_update=1e-2, policy=policy)
		self.agent.compile(Adam(lr=1e-3), metrics=['mae'])

	def train(self, env, nb_steps, visualize, verbosity):
		self.agent.fit(env, nb_steps=nb_steps, visualize=visualize, verbose=verbosity)

	def test(self, env, nb_episodes, visualize):
		self.agent.test(env, nb_episodes=nb_episodes, visualize=visualize)

	def load_weights(self, load_file):
		self.agent.load_weights(load_file)

	def save_weights(self, save_file, overwrite):
		self.agent.save_weights(save_file, overwrite)


class KerasNAFAgent(object):
	'''
	The Normalized Advantage Functions Agent as described in https://arxiv.org/abs/1603.00748
	'''

	def __init__(self, opts):
		self.metadata = {
			'discrete_actions': False,
		}

		self.opts = opts

	def configure(self, observation_space_shape, nb_actions):
		# Build all necessary models: V, mu, and L networks.
		V_model = Sequential()
		V_model.add(Flatten(input_shape=(1,) + observation_space_shape))
		V_model.add(Dense(16))
		V_model.add(Activation('relu'))
		V_model.add(Dense(16))
		V_model.add(Activation('relu'))
		V_model.add(Dense(16))
		V_model.add(Activation('relu'))
		V_model.add(Dense(1))
		V_model.add(Activation('linear'))
		print(V_model.summary())

		mu_model = Sequential()
		mu_model.add(Flatten(input_shape=(1,) + observation_space_shape))
		mu_model.add(Dense(16))
		mu_model.add(Activation('relu'))
		mu_model.add(Dense(16))
		mu_model.add(Activation('relu'))
		mu_model.add(Dense(16))
		mu_model.add(Activation('relu'))
		mu_model.add(Dense(nb_actions))
		mu_model.add(Activation('linear'))
		print(mu_model.summary())

		action_input = Input(shape=(nb_actions,), name='action_input')
		observation_input = Input(shape=(1,) + observation_space_shape, name='observation_input')
		x = merge([action_input, Flatten()(observation_input)], mode='concat')
		x = Dense(32)(x)
		x = Activation('relu')(x)
		x = Dense(32)(x)
		x = Activation('relu')(x)
		x = Dense(32)(x)
		x = Activation('relu')(x)
		x = Dense(((nb_actions * nb_actions + nb_actions) / 2))(x)
		x = Activation('linear')(x)
		L_model = Model(input=[action_input, observation_input], output=x)
		print(L_model.summary())

		# Finally, we configure and compile our agent. You can use every built-in Keras optimizer and
		# even the metrics!
		memory = SequentialMemory(limit=100000, window_length=1)
		random_process = OrnsteinUhlenbeckProcess(theta=.15, mu=0., sigma=.3, size=nb_actions)
		self.agent = ContinuousDQNAgent(nb_actions=nb_actions, V_model=V_model, L_model=L_model, mu_model=mu_model,
										memory=memory, nb_steps_warmup=100, random_process=random_process,
										gamma=.99, target_model_update=1e-3)
		self.agent.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])

	def train(self, env, nb_steps, visualize, verbosity):
		# Okay, now it's time to learn something! We visualize the training here for show, but this
		# slows down training quite a lot. You can always safely abort the training prematurely using
		# Ctrl + C.
		self.agent.fit(env, nb_steps=nb_steps, visualize=visualize, verbose=verbosity, nb_max_episode_steps=200)

	def test(self, env, nb_episodes, visualize):
		self.agent.test(env, nb_episodes=nb_episodes, visualize=visualize, nb_max_episode_steps=200)

	def load_weights(self, load_file):
		self.agent.load_weights(load_file)

	def save_weights(self, save_file, overwrite):
		# After training is done, we save the final weights.
		self.agent.save_weights(save_file, overwrite=overwrite)


class TemplateAgent(object):
	'''
	This is your template to copy from which an agent should fullfill. I know this is not as duck-type as possible, but helps to quickly implement new ones.
	'''

	def __init__(self, opts):
		self.opts = opts

	def configure(self, observation_space_shape, nb_actions):
		pass

	def train(self, env, nb_steps, visualize, verbosity):
		pass

	def test(self, env, nb_episodes, visualize):
		pass

	def load_weights(self, load_file):
		pass

	def save_weights(self, save_file, overwrite):
		pass


