"""Base class for simulation client."""

import enum
from typing import Text

GRAY = (0.3, 0.3, 0.3, 1)


class ClientType(enum.Enum):
  """Type of client."""
  BULLET = "pybullet"
  


class ClientMode(enum.Enum):
  """Client running mode."""

  # Client is being initialized, object is being loaded, teleported, etc.
  CONFIGURATION = 1

  # Client is in a mode that simulate motion according to physics and control.
  SIMULATION = 2


class WrongClientModeError(Exception):
  """Client mode does not meet expectation (e.g. load object in sim mode)."""


class BaseClient(object):
  """Base class for simulation client."""

  def __init__(self, client_type: Text = ""):
    self._client_type = client_type

    # Default to configuration mode.
    self._client_mode = ClientMode.CONFIGURATION

  @property
  def client_type(self) -> ClientType:
    return self._client_type

  def switch_mode(self, mode: ClientMode) -> bool:
    """Switches running mode of simulation client and return if mode changed."""
    if mode not in (ClientMode.CONFIGURATION, ClientMode.SIMULATION):
      raise ValueError(f"Invalid client mode {mode}.")
    if mode == self._client_mode:
      return False
    self._client_mode = mode
    return True

  @property
  def client_mode(self) -> ClientMode:
    """Returns current client mode."""
    return self._client_mode

  def _assert_in_configuration_mode(self, operation: Text = "this operation"):
    """Raises exception if client is not in configuration mode."""
    if self._client_mode != ClientMode.CONFIGURATION:
      raise WrongClientModeError(
          f"Sim client is expected to be in configuration mode for "
          f"{operation}.")


