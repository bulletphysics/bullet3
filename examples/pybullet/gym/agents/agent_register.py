import logging
import pkg_resources
import re
from gym import error
import warnings

logger = logging.getLogger(__name__)
# This format is true today, but it's *not* an official spec.
# [username/](agent-name)-v(version)    env-name is group 1, version is group 2
#
# 2017-08-26: We're experimentally expanding the agent ID format
# to include an optional username.
agent_id_re = re.compile(r'^(?:[\w:-]+\/)?([\w:.-]+)-v(\d+)$')

def load(name):
    entry_point = pkg_resources.EntryPoint.parse('x={}'.format(name))
    result = entry_point.load(False)
    return result

class AgentSpec(object):
    """A specification for a particular instance of the agent. Used
    to register the parameters for official evaluations.
    Args:
        id (str): The official agent ID
        entry_point (Optional[str]): The Python entrypoint of the agent class (e.g. module.name:Class)
        kwargs (dict): The kwargs to pass to the agent class
    Attributes:
        id (str): The official agent ID
    """

    def __init__(self, id, entry_point=None, kwargs=None):
        self.id = id

        # We may make some of these other parameters public if they're
        # useful.
        match = agent_id_re.search(id)
        if not match:
            raise error.Error('Attempted to register malformed agent ID: {}. (Currently all IDs must be of the form {}.)'.format(id, agent_id_re.pattern))
        self._agent_name = match.group(1)
        self._entry_point = entry_point
        self._kwargs = {} if kwargs is None else kwargs

    def make(self, **kwargs):
        """Instantiates an instance of the agent with appropriate kwargs"""
        if self._entry_point is None:
            raise error.Error('Attempting to make deprecated agent {}. (HINT: is there a newer registered version of this agent?)'.format(self.id))

        cls = load(self._entry_point)
        agent = cls(**kwargs)

        # Make the agent aware of which spec it came from.
        #agent.unwrapped._spec = self

        return agent

    def __repr__(self):
        return "AgentSpec({})".format(self.id)

class AgentRegistry(object):
    """Register an agent by ID.
    """

    def __init__(self):
        self.agent_specs = {}

    def make(self, id, **kwargs):
        logger.info('Making new agent: %s', id)
        spec = self.spec(id)
        agent = spec.make(**kwargs)

        return agent


    def all(self):
        return self.agent_specs.values()

    def spec(self, id):
        match = agent_id_re.search(id)
        if not match:
            raise error.Error('Attempted to look up malformed agent ID: {}. (Currently all IDs must be of the form {}.)'.format(id.encode('utf-8'), agent_id_re.pattern))

        try:
            return self.agent_specs[id]
        except KeyError:
            # Parse the agent name and check to see if it matches the non-version
            # part of a valid agent (could also check the exact number here)
            agent_name = match.group(1)
            matching_agents = [valid_agent_name for valid_agent_name, valid_agent_spec in self.agent_specs.items()
                             if agent_name == valid_agent_spec._agent_name]
            if matching_agents:
                raise error.DeprecatedEnv('Agent {} not found (valid versions include {})'.format(id, matching_agents))
            else:
                raise error.UnregisteredEnv('No registered agent with id: {}'.format(id))

    def register(self, id, **kwargs):
        if id in self.agent_specs:
            raise error.Error('Cannot re-register id: {}'.format(id))
        self.agent_specs[id] = AgentSpec(id, **kwargs)

# Have a global registry
agent_registry = AgentRegistry()

def register(id, **kwargs):
    return agent_registry.register(id, **kwargs)

def make(id, **kwargs):
    return agent_registry.make(id, **kwargs)

def spec(id):
    return agent_registry.spec(id)

