"""Exception types for safety related error."""


class SafetyError(Exception):
  """The base safety exception."""
  pass


class OutOfBoundError(SafetyError):
  """Rasied when values like motor position or velocity is out of bound."""
  pass
