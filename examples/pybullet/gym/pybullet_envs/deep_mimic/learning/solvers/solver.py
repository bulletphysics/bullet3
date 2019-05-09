from abc import abstractmethod
import sys, abc
if sys.version_info >= (3, 4):
  ABC = abc.ABC
else:
  ABC = abc.ABCMeta('ABC', (), {})


class Solver(ABC):

  def __init__(self, vars):
    self.vars = vars
    return

  @abstractmethod
  def update(self, grads):
    pass
