#! /usr/bin/env python
import sys
from hwcheck import base_check

if __name__  == '__main__':
    if base_check.check():
        sys.exit(0)
    sys.exit(-1)

