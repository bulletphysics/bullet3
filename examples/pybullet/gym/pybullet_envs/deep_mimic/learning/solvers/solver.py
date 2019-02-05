from abc import ABC, abstractmethod

class Solver(ABC):
    def __init__(self, vars):
        self.vars = vars
        return

    @abstractmethod
    def update(self, grads):
        pass