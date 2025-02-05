from enum import Enum

class Command(Enum):
    NO_INPUT = 0
    START = 1
    EMERGENCY_STOP = 2
    HOME_UP = 3
    HOME_DOWN = 4
    TROT_STOP = 5
    TROT_SLOW = 6
    TROT_FAST = 7
    TROT_OVERLAP = 8
    RESTART = 9
    ARM_HOME = 10
    ARM_MOVE = 11
    ARM_TELE_ON = 12
    ARM_TELE_OFF = 13
    ARM_GRP_OPEN = 14
    ARM_GRP_CLOSE = 15