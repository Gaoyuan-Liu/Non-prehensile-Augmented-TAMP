#!/usr/bin/env python

from __future__ import print_function

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import time
from random import random
from examples.pybullet.utils.pybullet_tools.utils import connect, get_client



#####################################

def main():
    a = dict()
    print(a)

    # a['a'] = 1
    print(a)
    print(len(a))

    connect(use_gui = True)

 

if __name__ == '__main__':
    main()
