import enum

class StateType(enum.Enum):
    GB_TRACK = 'GB_TRACK' 
    TRAILING = 'TRAILING' 
    OVERTAKE = 'OVERTAKE' 
    FTGONLY = 'FTGONLY'
    RECOVERY = 'RECOVERY'
    ATTACK = 'ATTACK'
    START = 'START'
    LOSTLINE = 'LOSTLINE'
    