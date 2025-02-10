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

class CanineFSM(Enum):
    INITIAL = 0
    READY = 1
    HOME_UP = 2
    HOME_DOWN = 3
    STAND = 4
    TROT_STOP = 5
    TROT_SLOW = 6
    TROT_FAST = 7
    OVERLAP_TROT_FAST = 8
    EMERGENCY_STOP = 9
    RESTART = 10

class ArmFSM(Enum):
    INITIAL = 0
    EMEREGENCY_STOP = 1
    MOTOR_READY = 2
    HOME = 3
    TELE_OPERATION = 4
    MOVE = 5
    READY = 6