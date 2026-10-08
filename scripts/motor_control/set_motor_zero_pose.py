#!/usr/bin/env python3
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from set_motor_pose import main

if __name__ == "__main__":
    sys.exit(main(zero_pose=True))
