
from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
import os
import ruamel.yaml as yaml

def save_config(config, logdir):
  """Save a new configuration by name.

  If a logging directory is specified, is will be created and the configuration
  will be stored there. Otherwise, a log message will be printed.

  Args:
    config: Configuration object.
    logdir: Location for writing summaries and checkpoints if specified.

  Returns:
    Configuration object.
  """
  message = 'Start a new run and write summaries and checkpoints to {}.'
  print(message.format(logdir))
  config_path = os.path.join(logdir, 'config.yaml')
  yaml.dump(config, config_path, default_flow_style=False)
  return config


def load_config(logdir):
  """Load a configuration from the log directory.

  Args:
    logdir: The logging directory containing the configuration file.

  Raises:
    IOError: The logging directory does not contain a configuration file.

  Returns:
    Configuration object.
  """
  config_path = logdir and os.path.join(logdir, 'config.yaml')
  if not config_path:
    message = (
        'Cannot resume an existing run since the logging directory does not '
        'contain a configuration file.')
    raise IOError(message)
  print("config_path=",config_path)

  stream = open(config_path, 'r')
  config = yaml.load(stream)
  message = 'Resume run and write summaries and checkpoints to {}.'
  print(message.format(logdir))
  return config
