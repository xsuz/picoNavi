from abc import ABCMeta, abstractmethod

class Sensor(metaclass=ABCMeta):
    @property
    def ID(self)->int:
        raise NotImplementedError
    @abstractmethod
    def parse(self,array:list[int])->int:
        raise NotImplementedError
    @abstractmethod
    def database(self)->dict[str,list]:
        raise NotImplementedError
    @property
    @abstractmethod
    def delta_t(self)->int:
        raise NotImplementedError