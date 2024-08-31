from enum import Enum

class AstarAction(Enum):
    LEFT = 1
    DOWN = 1
    RIGHT = 2
    UP = 3
    IDLE = 4
    
    
if __name__ == "__main__":
    left_action = AstarAction.LEFT
    
    import numpy as np
    
    a = [0,0]
    b = [0,-1]
    
    print(np.rad2deg(np.arctan2(b[1]-a[1], b[0]-a[0])))