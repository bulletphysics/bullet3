# ---- register agents ----------
from agents import agent_register

# agent_register.register(
# 	id='BaselinesDQNAgent-v0',
# 	entry_point='agents.BaselinesDQNAgent:BaselinesDQNAgent'
# )

agent_register.register(
	id='KerasCEMAgent-v0',
	entry_point='agents.KerasCEMAgent:KerasCEMAgent'
)

agent_register.register(
	id='KerasDDPGAgent-v0',
	entry_point='agents.KerasDDPGAgent:KerasDDPGAgent'
)

agent_register.register(
	id='KerasDDQNAgent-v0',
	entry_point='agents.KerasDDQNAgent:KerasDDQNAgent'
)

agent_register.register(
	id='KerasDQNAgent-v0',
	entry_point='agents.KerasDQNAgent:KerasDQNAgent'
)

agent_register.register(
	id='KerasNAFAgent-v0',
	entry_point='agents.KerasNAFAgent:KerasNAFAgent'
)

# from agents.BaselinesDQNAgent import BaselinesDQNAgent
from agents.KerasCEMAgent import KerasCEMAgent
from agents.KerasDDPGAgent import KerasDDPGAgent
from agents.KerasDDQNAgent import KerasDDQNAgent
from agents.KerasDQNAgent import KerasDQNAgent
from agents.KerasNAFAgent import KerasNAFAgent
