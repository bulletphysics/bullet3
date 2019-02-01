import json
import numpy as np
from learning.ppo_agent import PPOAgent

AGENT_TYPE_KEY = "AgentType"

def build_agent(world, id, file):
    agent = None
    with open(file) as data_file:    
        json_data = json.load(data_file)
        
        assert AGENT_TYPE_KEY in json_data
        agent_type = json_data[AGENT_TYPE_KEY]
        
        if (agent_type == PPOAgent.NAME):
            agent = PPOAgent(world, id, json_data)
        else:
            assert False, 'Unsupported agent type: ' + agent_type

    return agent