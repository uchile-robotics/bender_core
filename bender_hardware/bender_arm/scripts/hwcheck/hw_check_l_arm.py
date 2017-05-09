#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import hw_check

if __name__ == "__main__":
    if hw_check.l_arm_check():
        sys.exit(0)
    sys.exit(1)
