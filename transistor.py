from enum import Enum

class TransistorType(Enum):
    Invalid = 0
    NMOS = 1
    PMOS = 2

class Transistor:
    def __init__(self, ttype, conns):
        self.type = ttype
        self.connections = conns