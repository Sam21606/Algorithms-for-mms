from typing import List, Tuple
import math
import API
import sys
import Helper

def main() -> None:
    API.turnRight()
    API.moveForward(5)
    API.setColor(6, 0, "R")#x,y
    API.setText(6, 0, str(1))
    API.setColor(7, 0, "R")#x,y
    API.setText(7, 0, str(2))
    y = 1
    while y < 8:
        API.setColor(7, y, "R")#x,y
        API.setText(7, y, str(y + 2))
        y += 1

if __name__ == "__main__":
    main()