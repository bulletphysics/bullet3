"""An actor network."""
import tensorflow as tf
import sonnet as snt

class ActorNetwork(snt.AbstractModule):
  """An actor network as a sonnet Module."""

  def __init__(self, layer_sizes, action_size, name='target_actor'):
    super(ActorNetwork, self).__init__(name=name)
    self._layer_sizes = layer_sizes
    self._action_size = action_size

  def _build(self, inputs):
    state = inputs
    for output_size in self._layer_sizes:
      state = snt.Linear(output_size)(state)
      state = tf.nn.relu(state)

    action = tf.tanh(
        snt.Linear(self._action_size, name='action')(state))
    return action
