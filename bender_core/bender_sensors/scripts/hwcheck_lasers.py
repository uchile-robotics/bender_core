#! /usr/bin/env python
import sys
from hwcheck import laser_check

if __name__  == '__main__':
    if laser_check.check_lasers():
        sys.exit(0)
    sys.exit(-1)

