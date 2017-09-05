# ---- register agents ----------
import agent_register

# agent_register.register(
# 	id='BaselinesDQNAgent-v0',
# 	entry_point='agents_baselines:BaselinesDQNAgent'
# )

agent_register.register(
	id='KerasCEMAgent-v0',
	entry_point='pybullet_envs.agents.agents_kerasrl:KerasCEMAgent'
)

agent_register.register(
	id='KerasDDPGAgent-v0',
	entry_point='pybullet_envs.agents.agents_kerasrl:KerasDDPGAgent'
)

agent_register.register(
	id='KerasDDQNAgent-v0',
	entry_point='pybullet_envs.agents.agents_kerasrl:KerasDDQNAgent'
)

agent_register.register(
	id='KerasDQNAgent-v0',
	entry_point='pybullet_envs.agents.agents_kerasrl:KerasDQNAgent'
)

agent_register.register(
	id='KerasNAFAgent-v0',
	entry_point='pybullet_envs.agents.agents_kerasrl:KerasNAFAgent'
)

# from agents_baselines import BaselinesDQNAgent
from agents_kerasrl import KerasCEMAgent, KerasDDPGAgent, KerasDDQNAgent, KerasDQNAgent, KerasNAFAgent
