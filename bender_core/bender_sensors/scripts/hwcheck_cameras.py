#! /usr/bin/env python
import sys
from hwcheck import camera_check

if __name__  == '__main__':
    if camera_check.check_cameras():
        sys.exit(0)
    sys.exit(-1)

