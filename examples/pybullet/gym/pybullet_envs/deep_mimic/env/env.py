from abc import abstractmethod
import sys, abc
if sys.version_info >= (3, 4):
  ABC = abc.ABC
else:
  ABC = abc.ABCMeta('ABC', (), {})

import numpy as np
from enum import Enum


class Env(ABC):

  class Terminate(Enum):
    Null = 0
    Fail = 1
    Succ = 2

  def __init__(self, args, enable_draw):
    self.enable_draw = enable_draw
    return
