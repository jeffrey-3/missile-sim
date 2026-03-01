from abc import ABC, abstractmethod

class Controller(ABC):
    @abstractmethod
    def update(self, missile, target):
        pass
