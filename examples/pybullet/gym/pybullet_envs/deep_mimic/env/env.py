from abc import ABC, abstractmethod
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