#!/usr/bin/env python
import tf
import rospy
import sys
import math
import numpy as np
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Transform, Vector3, Twist
import exp_quat_func as eqf
import time
import scipy as sp
from scipy import linalg
import pickle

timeout = str(time.time())
# with open('alphabet.' + timeout + 'txt', 'wb') as f:
#     pickle.dump(alphabet, f)
with open('alphabet.' +  'txt', 'rb') as out_file:
    my_list = pickle.load(out_file)
    
    print("A: 75, 82, 88, 93")
    print(my_list['A'][75])
    print(my_list['A'][82])
    print(my_list['A'][88])
    print(my_list['A'][93])


    print("E: 75, 82, 88, 93 ")
    print(my_list['E'][75])
    print(my_list['E'][82])
    print(my_list['E'][88])
    print(my_list['E'][93])


