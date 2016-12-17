#!/usr/bin/env python3

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


#-------------------
# To get joint states, use rostopic echo robot/joint_states
#-------------------



import argparse
import rospy
import time
import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION
import os




#-----------------------------------------------------------
# Pose step definitions
"""
	theta1L = left e0
	theta2L = left e1
	theta3L = left s0
	theta4L = left s1
	theta5L = left w0
	theta6L = left w1
	theta7L = left w2


	lj1 = left w0
	lj2 = left w1
	lj3 = left w2
	lj4 = left e0
	lj5 = left e1
	lj6 = left s0
	lj7 = left s1	
"""
#-----------------------------------------------------------

def InitialPose():
	print('Initial Pose')
#Left arm joint positions
	theta1L = -1.4680
	theta2L = 1.47108
	theta3L = 0.65769
	theta4L = 1.04694
	theta5L = 1.79744
	theta6L = 0.98059
	theta7L = -1.2732

#Right arm joint positions
	theta1R = 1.33379
	theta2R = 1.66091
	theta3R = -0.7198
	theta4R = 0.94569
	theta5R = -1.8024
	theta6R = 0.96065
	theta7R = -0.1637

	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 5, 80, 80)





def ShrugBase():
#Left arm joint positions
	theta1L = -2.58168
	theta2L = 1.350670
	theta3L = 0.936111
	theta4L = 0.835636
	theta5L = -0.60860
	theta6L = 0.794602
	theta7L = 0.062893

#Right arm joint positions
	theta1R = 2.337403
	theta2R = 1.448844
	theta3R = -1.34223
	theta4R = 1.046558
	theta5R = 0.407655
	theta6R = 0.770058
	theta7R = -0.03911


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 6, 80, 80)




def ShrugUp():
#Left arm joint positions
	theta1L = -2.81101
	theta2L = 1.566194
	theta3L = 0.982898
	theta4L = 0.338626
	theta5L = -0.66613
	theta6L = -1.12670
	theta7L = -0.02416

#Right arm joint positions
	theta1R = 2.814471
	theta2R = 2.026388
	theta3R = -1.14665
	theta4R = 0.551082
	theta5R = 0.259626
	theta6R = -1.14895
	theta7R = 0.068262


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 4, 80, 80)





def RightArmMidPoint():
#Left arm joint positions
	theta1L = -1.46878
	theta2L = 1.473005
	theta3L = 0.659611
	theta4L = 1.054611
	theta5L = 1.795908
	theta6L = 0.978679
	theta7L = -1.27051

#Right arm joint positions
	theta1R = 1.800509
	theta2R = 1.746437
	theta3R = -0.85250
	theta4R = 0.426446
	theta5R = -2.03635
	theta6R = 0.113514
	theta7R = 0.083985


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right,1.5, 80, 80)





def RightArmWaveBase():
#Left arm joint positions
	theta1L = -1.46955
	theta2L = 1.473388
	theta3L = 0.659228
	theta4L = 1.054228
	theta5L = 1.795908
	theta6L = 0.978679
	theta7L = -1.27128

#Right arm joint positions
	theta1R = 1.874141
	theta2R = 1.593422
	theta3R = -0.73976
	theta4R = 0.432582
	theta5R = -2.11190
	theta6R = -1.16582
	theta7R = 0.006135


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right,4, 80, 80)






def RightArmWaveRight():
#Left arm joint positions
	theta1L = -1.46993
	theta2L = 1.472621
	theta3L = 0.659611
	theta4L = 1.054228
	theta5L = 1.795524
	theta6L = 0.978296
	theta7L = -1.27090

#Right arm joint positions
	theta1R = 2.075092
	theta2R = 1.444242
	theta3R = -0.63353
	theta4R = 0.508898
	theta5R = -1.55737
	theta6R = -1.15278
	theta7R = -0.06672


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 2, 80, 80)





def RightArmWaveLeft():
#Left arm joint positions
	theta1L = -1.46917
	theta2L = 1.473388
	theta3L = 0.659995
	theta4L = 1.054228
	theta5L = 1.795908
	theta6L = 0.978679
	theta7L = -1.27090

#Right arm joint positions
	theta1R = 2.076626
	theta2R = 1.517873
	theta3R = -0.60285
	theta4R = 0.520402
	theta5R = -2.84208
	theta6R = -1.21376
	theta7R = 0.009970


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 2, 80, 80)




def ArmCrossedRightUp():
#Left arm joint positions
	theta1L = -1.47147
	theta2L = 1.588053
	theta3L = 0.640053
	theta4L = 1.046941
	theta5L = 1.034670
	theta6L = 1.355272
	theta7L = -1.66245

#Right arm joint positions
	theta1R = 1.605310
	theta2R = 1.922077
	theta3R = -0.51234
	theta4R = 1.041956
	theta5R = -1.22948
	theta6R = 1.211077
	theta7R = -1.32689


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 2, 80, 80)




def RightArmtoMouth():
#Left arm joint positions
	theta1L = -1.47147
	theta2L = 1.588053
	theta3L = 0.640053
	theta4L = 1.046941
	theta5L = 1.034670
	theta6L = 1.355272
	theta7L = -1.66245

#Right arm joint positions
	theta1R = 2.312859
	theta2R = 2.476995
	theta3R = -0.09165
	theta4R = 1.047325
	theta5R = -0.90236
	theta6R = 0.930742
	theta7R = -2.25034


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 3, 80, 80)


def RightArmtoMouthThank():
#Left arm joint positions
	theta1L = -1.4680
	theta2L = 1.47108
	theta3L = 0.65769
	theta4L = 1.04694
	theta5L = 1.79744
	theta6L = 0.98059
	theta7L = -1.2732

#Right arm joint positions
	theta1R = 2.312859
	theta2R = 2.476995
	theta3R = -0.09165
	theta4R = 1.047325
	theta5R = -0.90236
	theta6R = 0.930742
	theta7R = -2.25034


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 3, 80, 80)

def RightArmCrossHandDown():
#Left arm joint positions
	theta1L = -1.469170099597
	theta2L = 1.5861361
	theta3L = 0.640053
	theta4L = 1.046941
	theta5L = 1.034670
	theta6L = 1.355272
	theta7L = -1.66245

#Right arm joint positions
	theta1R = 1.780184
	theta2R = 1.671272
	theta3R = -0.21360
	theta4R = 1.048092
	theta5R = -1.32267
	theta6R = 1.297747
	theta7R = 1.665136


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 4, 80, 80)





def RightArmCrossHandUp():
#Left arm joint positions
	theta1L = -1.469170099597
	theta2L = 1.5861361
	theta3L = 0.640053
	theta4L = 1.046941
	theta5L = 1.034670
	theta6L = 1.355272
	theta7L = -1.66245

#Right arm joint positions
	theta1R = 1.428136
	theta2R = 1.848446
	theta3R = -0.44562
	theta4R = 1.046558
	theta5R = -1.30503
	theta6R = 0.970242
	theta7R = -1.50406


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 2, 80, 80)



def BothArmOutMidpoint():
#Left arm joint positions
	theta1L = -2.00798
	theta2L = 1.283941
	theta3L = 0.327504
	theta4L = 1.046941
	theta5L = -0.20363
	theta6L = 0.597102
	theta7L = -0.09549

#Right arm joint positions
	theta1R = 2.052466
	theta2R = 1.338014
	theta3R = -0.51580
	theta4R = 1.047708
	theta5R = 0.112747
	theta6R = 0.456742
	theta7R = 0.066728


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 4, 80, 80)



def BothArmFace():
#Left arm joint positions
	theta1L = -1.89293
	theta2L = 2.518796
	theta3L = 0.358951
	theta4L = 1.046941
	theta5L = 0.212839
	theta6L = 0.637752
	theta7L = -0.02492

#Right arm joint positions
	theta1R = 1.925912
	theta2R = 2.386490
	theta3R = -0.55146
	theta4R = 1.046558
	theta5R = -0.11734
	theta6R = 1.018946
	theta7R = -0.04103


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 4, 80, 80)





def Yesfist():
#Left arm joint positions
	theta1L = -1.46917
	theta2L = 1.475306
	theta3L = 0.659995
	theta4L = 1.053461
	theta5L = 1.795524
	theta6L = 0.979830
	theta7L = -1.27128

#Right arm joint positions
	theta1R = 2.526082
	theta2R = 1.838475
	theta3R = -0.42491
	theta4R = 0.538810
	theta5R = -2.28563
	theta6R = 0.972160
	theta7R = 0.016106


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 3.5, 80, 80)





def NofistOpen():
#Left arm joint positions
	theta1L = -1.46955
	theta2L = 1.471471
	theta3L = 0.659995
	theta4L = 1.054611
	theta5L = 1.795524
	theta6L = 0.978296
	theta7L = -1.27128

#Right arm joint positions
	theta1R = 3.054922
	theta2R = 2.083529
	theta3R = -0.00997
	theta4R = 0.631616
	theta5R = -2.44478
	theta6R = 1.690446
	theta7R = -1.77366


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

	#needs gripper closed here
#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 1, 80, 80)




def NofistClose():
#Left arm joint positions
	theta1L = -1.46955
	theta2L = 1.471471
	theta3L = 0.659995
	theta4L = 1.054611
	theta5L = 1.795524
	theta6L = 0.978296
	theta7L = -1.27128

#Right arm joint positions
	theta1R = 3.054922
	theta2R = 2.083529
	theta3R = -0.00997
	theta4R = 0.631616
	theta5R = -2.44478
	theta6R = 1.690446
	theta7R = -1.77366


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

	#needs gripper closed here
#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 1, 0, 80)



def Nofist():
#Left arm joint positions
	theta1L = -1.46955
	theta2L = 1.471471
	theta3L = 0.659995
	theta4L = 1.054611
	theta5L = 1.795524
	theta6L = 0.978296
	theta7L = -1.27128

#Right arm joint positions
	theta1R = 3.054922
	theta2R = 2.083529
	theta3R = -0.00997
	theta4R = 0.631616
	theta5R = -2.44478
	theta6R = 1.690446
	theta7R = -1.77366


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

	#needs gripper closed here
#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 3, 0, 80)




def YesnodDown():
#Left arm joint positions
	theta1L = -1.470320
	theta2L = 1.4722380
	theta3L = 0.6596117
	theta4L = 1.0538448
	theta5L = 1.7959080
	theta6L = 0.9782962
	theta7L = -1.270903

#Right arm joint positions
	theta1R = 2.6334615
	theta2R = 1.8492138
	theta3R = -0.340160
	theta4R = 0.5694903
	theta5R = -2.302505
	theta6R = 1.4860438
	theta7R = 0.0026844


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 1, 80, 80)


def YesnodUp():
#Left arm joint positions
	theta1L = -1.47032
	theta2L = 1.472238
	theta3L = 0.659611
	theta4L = 1.053844
	theta5L = 1.795908
	theta6L = 0.978296
	theta7L = -1.27090

#Right arm joint positions
	theta1R = 2.633461
	theta2R = 1.849213
	theta3R = -0.34016
	theta4R = 0.569490
	theta5R = -2.30250
	theta6R = 0.806490
	theta7R = 0.002684


	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()

	#Makes a dictionary input for the left arm. Matches joints to thetas
	dictionary_commands_left = {
	lj[0]: theta3L,
	lj[1]: theta4L,
	lj[2]: theta1L,
	lj[3]: theta2L,
	lj[4]: theta5L,
	lj[5]: theta6L,
	lj[6]: theta7L
	}

	#Makes a dictionary input for the right arm. Matches joints to thetas
	dictionary_commands_right = {
	rj[0]: theta3R,
	rj[1]: theta4R,
	rj[2]: theta1R,
	rj[3]: theta2R,
	rj[4]: theta5R,
	rj[5]: theta6R,
	rj[6]: theta7R
	}

#Actually carries out commands and moves arm:
	Carryout(dictionary_commands_left, dictionary_commands_right, 1, 80, 80)




def NTMYHandsTogether():
#Left arm joint positions
    theta1L = -1.69313129
    theta2L = 2.08966532
    theta3L = 0.67878649   
    theta4L = 1.04694188   
    theta5L = 1.91325753   
    theta6L = 0.97983022   
    theta7L = -2.29905370


#Right arm joint positions
    theta1R = 1.52017496   
    theta2R = 1.7847866   
    theta3R =  -0.08283
    theta4R = 1.04502441
    theta5R = -3.0491703
    theta6R = -0.28263596
    theta7R = 0.7086991


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }

#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3, 80, 80)








def NTMYHandSlide():
#Left arm joint positions
    theta1L = -1.69236430
    theta2L = 2.0873643
    theta3L = 0.6791699
    theta4L = 1.04694188
    theta5L = 1.9140245
    theta6L = 0.9802137
    theta7L = -2.300204

#Right arm joint positions
    theta1R = 1.65938
    theta2R = 1.7855536
    theta3R = -0.163752449
    theta4R = 1.04655839
    theta5R =  -2.9901120
    theta6R = 0.38848063
    theta7R = 0.8260486


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3, 80, 80)




def NTMYFistTogether1():
#Left arm joint positions
    theta1L =  -1.6739565
    theta2L = 1.908272100
    theta3L = 0.4275971
    theta4L = 1.04694188
    theta5L = -0.12808739
    theta6L = 1.27358754
    theta7L = -1.4572817

#Right arm joint positions
    theta1R = 1.69965071
    theta2R = 1.68047595
    theta3R = -0.1672039
    theta4R = 1.046941887
    theta5R =  -2.9993159
    theta6R =  -1.4227671
    theta7R = 0.8678496


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 2,0, 0)




def NTMYFistTogether2():
#Left arm joint positions
    theta1L = -1.6904468
    theta2L = 1.9105730
    theta3L = 0.1545485
    theta4L =  1.0469418
    theta5L =  -0.3106311
    theta6L = 0.8801214770
    theta7L = -0.9890341

#Right arm joint positions
    theta1R = 1.8829614
    theta2R = 1.7004177
    theta3R =  0.12616991
    theta4R = 1.04655839
    theta5R = -2.9114955
    theta6R = -1.23830599
    theta7R =  0.8620972


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 2, 0, 0)



def RightHandPoint():
#Left arm joint positions
    theta1L = -1.4680
    theta2L = 1.47108
    theta3L = 0.65769
    theta4L = 1.04694
    theta5L = 1.79744
    theta6L = 0.98059
    theta7L = -1.2732

#Right arm joint positions
    theta1R = 2.4355779
    theta2R = 2.200111945
    theta3R = 0.29414081
    theta4R = 1.04540790
    theta5R = -3.04725283
    theta6R = 1.1397477253
    theta7R = 1.5029176


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3,0,80)




def AgeRightHandFace():
#Left arm joint positions
    theta1L = -1.4699370
    theta2L = 1.473005
    theta3L = 0.656160282
    theta4L = 1.04694188
    theta5L = 1.79667499
    theta6L = 0.9786797
    theta7L = -1.270903082

#Right arm joint positions
    theta1R =  1.754107030
    theta2R = 2.58935956
    theta3R =  -0.43833501
    theta4R = 0.88088846
    theta5R = -0.60937386
    theta6R =  0.633917560
    theta7R = 0.476684529


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3,80,80)




def AgeHandMoveDown():
#Left arm joint positions
    theta1L = -1.47032058
    theta2L = 1.47377204
    theta3L = 0.65577678
    theta4L = 1.04694188
    theta5L = 1.795908007
    theta6L =  0.9798302
    theta7L = -1.270903082

#Right arm joint positions
    theta1R = 1.7000342
    theta2R = 1.866471123
    theta3R =  -0.395000
    theta4R = 1.04655839
    theta5R = -1.15623801
    theta6R = 1.232553563
    theta7R = 0.59671852


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3, 0, 80)





def RepeatHandsOut():
#Left arm joint positions
    theta1L = -1.45498077
    theta2L = 1.6275536
    theta3L = 0.735927282
    theta4L = 1.04655839
    theta5L = 1.2413739
    theta6L = 0.2761165418
    theta7L = -1.719975958

#Right arm joint positions
    theta1R = 1.291228
    theta2R = 1.7840196
    theta3R = -0.9974710
    theta4R = 1.0427234
    theta5R = -1.28700
    theta6R =  -0.0203252
    theta7R =  1.7537235


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 2, 80, 80)




def RepeatHalfway():
#Left arm joint positions
    theta1L = -1.3257428959
    theta2L = 1.70732061691
    theta3L = 0.9123350735
    theta4L = 1.03888848
    theta5L = 1.16122345642
    theta6L = 0.3693058746
    theta7L = -1.65900022


#Right arm joint positions
    theta1R = 1.75410703
    theta2R = 1.4615001956
    theta3R = 0.339393249
    theta4R = -0.2362330
    theta5R = -1.4894953
    theta6R =  1.50598563
    theta7R = 1.013194310


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1,30,80)




def RepeatRightHandPalm():
#Left arm joint positions
    theta1L = -1.3257428959
    theta2L = 1.70732061691
    theta3L = 0.9123350735
    theta4L = 1.03888848
    theta5L = 1.16122345642
    theta6L = 0.3693058746
    theta7L = -1.65900022

#Right arm joint positions
    theta1R =  1.634456529
    theta2R =0.8187622455
    theta3R =  0.9598884
    theta4R = -0.082834962
    theta5R = -1.60185943
    theta6R = 1.4139467
    theta7R = 1.17426229


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3.5,30,80)






def PleaseHandHeart():
#Left arm joint positions
    theta1L = -1.466102138
    theta2L = 1.47300505
    theta3L = 0.659228243
    theta4L = 1.04655839
    theta5L = 1.79590800
    theta6L =  0.978679742
    theta7L = -1.27588852

#Right arm joint positions
    theta1R = 2.111141059
    theta2R = 2.50000518
    theta3R = -0.39346607209
    theta4R = 1.04809237
    theta5R = -0.99823799
    theta6R = 1.267835121
    theta7R =  1.08068946


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3,80,80)





def PleaseCircle1():
#Left arm joint positions
    theta1L = -1.4664856
    theta2L = 1.472621556
    theta3L = 0.6592282
    theta4L = 1.047325382
    theta5L = 1.7962915
    theta6L = 0.980213723
    theta7L = -1.275505025

#Right arm joint positions
    theta1R = 2.1207284392
    theta2R = 2.51879645
    theta3R = 0.10814564554
    theta4R = 1.04694188
    theta5R =-1.78785460
    theta6R =  1.3387817326
    theta7R = 1.226417639


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1,80,80)




def PleaseCircle2():
#Left arm joint positions
    theta1L = -1.46610213
    theta2L = 1.473005051566
    theta3L = 0.65922824
    theta4L = 1.04655839
    theta5L = 1.795908007
    theta6L = 0.977912752
    theta7L = -1.2755050251

#Right arm joint positions
    theta1R = 2.215451752
    theta2R =  2.14450514
    theta3R = 0.2676796474
    theta4R = 1.0461748973
    theta5R = -1.6214176
    theta6R = 1.41701475
    theta7R = 0.905432160


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1, 80, 80)




def PleaseCircle3():
#Left arm joint positions
    theta1L =  -1.466102
    theta2L = 1.473005051
    theta3L = 0.65961173
    theta4L = 1.047325382
    theta5L = 1.794757521
    theta6L = 0.97752925
    theta7L = -1.275121529

#Right arm joint positions
    theta1R = 1.7587089
    theta2R =  2.603165397
    theta3R = -0.263461200
    theta4R = 0.696810772
    theta5R = -1.785170141
    theta6R = 1.532830302
    theta7R =  1.43964096


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1,80,80)





def PleaseCircle4():
#Left arm joint positions
    theta1L = -1.46571864
    theta2L = 1.472621556
    theta3L = 0.659995233
    theta4L = 1.04694188
    theta5L = 1.79514101
    theta6L = 0.98098071
    theta7L = -1.275121529

#Right arm joint positions
    theta1R = 1.880660445
    theta2R = 2.603932387
    theta3R =  -0.586364156
    theta4R = 0.704480676
    theta5R =  -1.400907954
    theta6R = 0.982514694
    theta7R = 1.20302443


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1,80,80)




def ThankYouLowerMiddle():
#Left arm joint positions
    theta1L = -1.4680
    theta2L = 1.47108
    theta3L = 0.65769
    theta4L = 1.04694
    theta5L = 1.79744
    theta6L = 0.98059
    theta7L = -1.2732

#Right arm joint positions
    theta1R = 2.228107094
    theta2R = 1.77136431
    theta3R =  0.08283496
    theta4R = 1.0469418
    theta5R = -1.3207574
    theta6R = 1.055762277
    theta7R = -1.1428156


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3, 80,80)



def YoureWelcomePalm1():
#Left arm joint positions
    theta1L = -1.8319565559
    theta2L = 2.541422670
    theta3L = 0.6515583396
    theta4L = 1.043873926
    theta5L = 1.62985458
    theta6L = 0.753951557
    theta7L = -1.334179790

#Right arm joint positions
    theta1R = 1.33417979
    theta2R = 1.66245167
    theta3R = -0.7182865
    theta4R = 0.94723313
    theta5R = -1.8005099
    theta6R = 0.9591214
    theta7R = -0.162601963


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1,80,30)



def YoureWelcomePalm2():
#Left arm joint positions
    theta1L = -2.0083643465
    theta2L = 1.91824297
    theta3L = 0.2531068300
    theta4L = 1.0473253829
    theta5L = 1.231403077
    theta6L = 1.2444419
    theta7L = -1.875675008

#Right arm joint positions
    theta1R = 1.33533027
    theta2R = 1.663602164
    theta3R = -0.71790300
    theta4R = 0.9468496413
    theta5R = -1.80050994
    theta6R = 0.958354497
    theta7R = -0.1633689


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1,80,30)




def YoureWelcomePalm3():
#Left arm joint positions
    theta1L = -1.20954385
    theta2L = 1.63790798
    theta3L = 0.47093210
    theta4L =  1.023548680
    theta5L =  0.536509780
    theta6L = 1.38058270
    theta7L = -2.72051492

#Right arm joint positions
    theta1R = 1.33648076
    theta2R = 1.6628351740
    theta3R =  -0.71828650
    theta4R = 0.947233136
    theta5R = -1.80012645
    theta6R = 0.95950498
    theta7R = -0.16260196


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1,80,30)


def HappyBothArmUp():
#Left arm joint positions
    theta1L = -1.451912815
    theta2L = 2.10577212
    theta3L = 0.6615292147
    theta4L = 1.0473253829
    theta5L = 1.1501020
    theta6L = 1.2885438
    theta7L = 3.058757

#Right arm joint positions
    theta1R = 1.65171381
    theta2R = 2.4095003
    theta3R = -0.79191758
    theta4R = 1.0258496
    theta5R = -1.0097428
    theta6R = 1.18730112
    theta7R = 0.74359718


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3,80,80)




def HappyHandsOpen():
#Left arm joint positions
    theta1L = -1.775199266
    theta2L = 1.9282138
    theta3L = 0.409956365
    theta4L = 1.0465583925
    theta5L = 1.147034134
    theta6L = 1.1094516048
    theta7L = 3.058757

#Right arm joint positions
    theta1R = 2.15255854
    theta2R = 2.27374302
    theta3R =  -0.45904375
    theta4R = 1.04617489
    theta5R = -1.09142733
    theta6R =  1.23217006
    theta7R = 0.743980682


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1, 80, 80)


def HappyHandsOpen2():
#Left arm joint positions
    theta1L = -1.7863206
    theta2L = 1.926296374
    theta3L = 0.376592283
    theta4L = 1.046941887731
    theta5L =  1.47760699
    theta6L =  1.1577719
    theta7L = 3.058757

#Right arm joint positions
    theta1R = 2.16598087
    theta2R = 2.258403214
    theta3R =  -0.3232864510
    theta4R = 1.047325382
    theta5R = -1.363708920
    theta6R = 1.20724288
    theta7R = 0.7604709


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1, 80, 80)


def SadHandsFace():
#Left arm joint positions
    theta1L =  -1.8350245
    theta2L = 2.5406556
    theta3L = 0.576009785
    theta4L = 1.04847586851
    theta5L =  0.2381505173
    theta6L = 0.6772525178
    theta7L = -3.05952468

#Right arm joint positions
    theta1R = 1.6873788
    theta2R =  2.413718769
    theta3R = -0.7347767973
    theta4R = 1.042723440
    theta5R = 0.07363107
    theta6R =  1.060364219
    theta7R = -0.1085291407428


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 4, 40, 40)


def SadHandsDown():
#Left arm joint positions
    theta1L = -1.60684487
    theta2L = 2.1399031991
    theta3L = 0.448305885
    theta4L = 1.04694188
    theta5L = -0.015339807
    theta6L =  1.2709030
    theta7L = -2.87966543

#Right arm joint positions
    theta1R = 1.624102159
    theta2R = 2.1218789248
    theta3R = -0.31600004
    theta4R = 1.04732538
    theta5R = 0.06749515
    theta6R = 1.313471049
    theta7R = -0.56872337


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1.5, 40, 40)



def Tired1():
#Left arm joint positions
    theta1L = -1.54471865
    theta2L = 2.03712648631
    theta3L =  0.68415543
    theta4L = 1.04694188
    theta5L = 0.6354515413
    theta6L = 1.72726236
    theta7L = -3.05914118

#Right arm joint positions
    theta1R = 1.496014763
    theta2R = 2.134534266
    theta3R = -0.790767096
    theta4R = 1.0465583
    theta5R = -0.68415543
    theta6R = 1.680859448
    theta7R = 0.27765052


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3, 30, 30)


def Tired2():
#Left arm joint positions
    theta1L =-1.34146619
    theta2L = 1.910189576
    theta3L =0.7182865039
    theta4L = 1.04233994
    theta5L =0.233165079
    theta6L = 1.762927420
    theta7L = -2.576320733

#Right arm joint positions
    theta1R =1.29659726
    theta2R = 1.86110219
    theta3R = -0.56872337
    theta4R = 1.02661664
    theta5R = -0.15991749
    theta6R = 1.747587612
    theta7R = -0.79882049


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1, 30, 30)





def TiredFinal():
#Left arm joint positions
    theta1L =-1.24866036
    theta2L = 1.606844875
    theta3L =0.62701464
    theta4L = 1.046941887
    theta5L =0.124635939
    theta6L = 1.93281579273
    theta7L = -2.53797121

#Right arm joint positions
    theta1R =1.280873957
    theta2R = 1.6382914
    theta3R = -0.54763114
    theta4R = 1.046558392
    theta5R = -0.1138980
    theta6R = 1.904053652
    theta7R = -0.798820495


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1, 30, 30)




def ConfusedHandAboveHead():
#Left arm joint positions
    theta1L = -1.4661021380
    theta2L =  1.469170099
    theta3L = 0.656160282
    theta4L = 1.046941887
    theta5L = 1.795524512
    theta6L = 0.98174770
    theta7L = -1.275505

#Right arm joint positions
    theta1R = 1.9216944320
    theta2R = 2.2273401
    theta3R = 0.48818938
    theta4R = -0.08053399
    theta5R = -1.25287880
    theta6R = -1.4607332
    theta7R = 1.1489516


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 4, 0, 100)

def ConfusedHandAboveHeadClose():
#Left arm joint positions
    theta1L = -1.4661021380
    theta2L =  1.469170099
    theta3L = 0.656160282
    theta4L = 1.046941887
    theta5L = 1.795524512
    theta6L = 0.98174770
    theta7L = -1.275505

#Right arm joint positions
    theta1R = 1.9216944320
    theta2R = 2.2273401
    theta3R = 0.48818938
    theta4R = -0.08053399
    theta5R = -1.25287880
    theta6R = -1.4607332
    theta7R = 1.1489516


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1, 0, 100)




def ConfusedHandAboveHeadOpen():
#Left arm joint positions
    theta1L = -1.4661021380
    theta2L =  1.469170099
    theta3L = 0.656160282
    theta4L = 1.046941887
    theta5L = 1.795524512
    theta6L = 0.98174770
    theta7L = -1.275505

#Right arm joint positions
    theta1R = 1.9216944320
    theta2R = 2.2273401
    theta3R = 0.48818938
    theta4R = -0.08053399
    theta5R = -1.25287880
    theta6R = -1.4607332
    theta7R = 1.1489516


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1, 100, 100)





def HAYHands1():
#Left arm joint positions
    theta1L = -1.673573039
    theta2L = 2.3945440
    theta3L = 0.2753495514
    theta4L = 1.046174897
    theta5L =  2.1705828
    theta6L = 1.9719323028
    theta7L = -2.60393238

#Right arm joint positions
    theta1R = 1.54855360
    theta2R = 2.50422363
    theta3R = -0.27419906
    theta4R = 0.90236419
    theta5R = -2.1303158
    theta6R = 1.69965071
    theta7R = -0.582145


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3, 60, 60)




def HAYHands2():
#Left arm joint positions
    theta1L = -1.722276
    theta2L = 1.6586167
    theta3L = 0.120800987
    theta4L = 1.04655839
    theta5L = 0.80035447
    theta6L = 2.06588862
    theta7L = -2.7584809

#Right arm joint positions
    theta1R = 1.7916895
    theta2R = 1.73646625
    theta3R = -0.33824276
    theta4R = 1.04655839
    theta5R = -1.06995159
    theta6R = 2.08698086
    theta7R = -0.564121434


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 2, 60, 60)




def HAYHands3():
#Left arm joint positions
    theta1L = -1.389403098
    theta2L = 1.81584975
    theta3L = 0.108912635
    theta4L = 1.04694188
    theta5L = -0.59748551
    theta6L = 1.353738045
    theta7L = -0.5514660932

#Right arm joint positions
    theta1R = 1.35373804
    theta2R = 1.84077694
    theta3R =  -0.28685440
    theta4R = 1.0477088
    theta5R = 0.345912667
    theta6R = 1.472621556
    theta7R = 0.46364569313


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 4, 60, 60)


def HAYHandsFinal():
#Left arm joint positions
    theta1L = -1.433121551
    theta2L = 1.748738098
    theta3L = 0.24620391645
    theta4L = 1.04425742
    theta5L = -0.13000487
    theta6L = 1.287393376
    theta7L = -0.69757776

#Right arm joint positions
    theta1R = 1.9661798748
    theta2R = 2.053233284
    theta3R = -0.1775582761
    theta4R = 1.0473253829
    theta5R = 0.7980535048
    theta6R = -0.1092961
    theta7R = 0.584063184


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 2, 60, 60)






def NameStopHand():
#Left arm joint positions
    theta1L = -1.469553594
    theta2L = 1.47262155
    theta3L = 0.65577678
    theta4L = 1.04464091654
    theta5L = 1.79552451
    theta6L = 0.9825146946
    theta7L = -1.2712865

#Right arm joint positions
    theta1R = 1.5592914
    theta2R = 2.04939833
    theta3R = -0.1484126
    theta4R = 1.04732538
    theta5R = -2.517262472
    theta6R =  -0.931126
    theta7R = -1.17464578


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3, 80, 80)




def NameHandTap():
#Left arm joint positions
    theta1L = -1.46955359
    theta2L =  1.47262155
    theta3L = 0.65577678
    theta4L = 1.0446409
    theta5L = 1.79552451
    theta6L =  0.982514694
    theta7L = -1.27128657

#Right arm joint positions
    theta1R =  1.5592914
    theta2R = 2.0493983
    theta3R = -0.1484126
    theta4R = 1.0473253
    theta5R = -2.51726247
    theta6R = -0.931126338
    theta7R = -1.1746457


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3, 0, 0)



def NameHandsUpRight():
#Left arm joint positions
    theta1L = -1.645577890
    theta2L = 1.744136155
    theta3L = 0.81876224
    theta4L =  1.04694188
    theta5L =  1.447694368
    theta6L = 0.433733067
    theta7L = -1.9949420146

#Right arm joint positions
    theta1R = 1.57233030
    theta2R = 1.6363740
    theta3R = -0.6534758156
    theta4R = 1.04732538
    theta5R = -1.84614587
    theta6R = 0.16797089
    theta7R = 2.32704885


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 3, 80, 80)





def NameHandsUpLeft():
#Left arm joint positions
    theta1L = -1.60070895
    theta2L = 1.71192255
    theta3L = 0.59096609
    theta4L = 1.0465583
    theta5L = 1.395922516
    theta6L = 0.47860200582
    theta7L = -2.0486313

#Right arm joint positions
    theta1R = 1.64059245
    theta2R = 1.79859247
    theta3R = -0.5357427
    theta4R = 1.04732538
    theta5R = -1.8472963
    theta6R = 0.52500492
    theta7R = 2.360796432


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()

    #Makes a dictionary input for the left arm. Matches joints to thetas
    dictionary_commands_left = {
    lj[0]: theta3L,
    lj[1]: theta4L,
    lj[2]: theta1L,
    lj[3]: theta2L,
    lj[4]: theta5L,
    lj[5]: theta6L,
    lj[6]: theta7L
    }

    #Makes a dictionary input for the right arm. Matches joints to thetas
    dictionary_commands_right = {
    rj[0]: theta3R,
    rj[1]: theta4R,
    rj[2]: theta1R,
    rj[3]: theta2R,
    rj[4]: theta5R,
    rj[5]: theta6R,
    rj[6]: theta7R
    }


#Actually carries out commands and moves arm:
    Carryout(dictionary_commands_left, dictionary_commands_right, 1, 80, 80)





def Carryout(dictionary_commands_left, dictionary_commands_right, timestep, rightGrip, leftGrip):
	print('Moving to Pose')
	left = baxter_interface.Limb('left')
	right = baxter_interface.Limb('right')
	grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
	grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
	lj = left.joint_names()
	rj = right.joint_names()
	timeout = time.time() + timestep;
	#grip_left.calibrate()
	#grip_right.calibrate()
	while True:
				right.set_joint_positions(dictionary_commands_right);
				left.set_joint_positions(dictionary_commands_left);
				#grip_left.calibrate()
				#grip_right.calibrate()
				grip_left.command_position(leftGrip)
				grip_right.command_position(rightGrip)
				time.sleep(0.1);
				if time.time() > timeout:
					break
	done = False
	#print("Controlling joints. Press ? for help, Esc to quit.")
	print('Movement Complete')
	#while not done and not rospy.is_shutdown():
	#		c = baxter_external_devices.getch()
	#		if c:
     #       #catch Esc or ctrl-c
	#			if c in ['\x1b', '\x03']:
	#				done = True
	#				rospy.signal_shutdown("Example finished.")
	#			elif c in bindings:
	#				cmd = bindings[c]
     #               #expand binding to something like "set_j(right, 's0', 0.1)"
	#				cmd[0](*cmd[1])
	#				print("command: %s" % (cmd[2],))
	#			else:
	#				print("  Esc: Quit")
	#				print("  ?: Help")
	#				for key, val in sorted(bindings.items(),
	#										key=lambda x: x[1][2]):
	#					print("  %s: %s" % (key, val[2]))
	return
	




#Collection of movement definitions:

def DontKnow():
	ShrugBase()
	print('Shrug Position 1 complete... moving to Shrug Position 2')
	ShrugUp()
	print('Shrug Position 2 complete... moving to Shrug Position 1')
	ShrugBase()
	print('Shrug Movement complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete.... Full Movement Complete')


def Hi():
	RightArmWaveBase()
	print('Right Arm Wave Base complete... moving to Left Wave')
	RightArmWaveLeft()
	print('Left Wave Base complete... moving to Right Wave')
	RightArmWaveRight()
	print('Right Wave Base complete... moving to Left Wave')
	RightArmWaveLeft()
	print('Left Wave Base complete... moving to Right Wave')
	RightArmWaveRight()
	print('Right Wave Base complete... moving to Wave Base')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def Good():
	RightArmtoMouth()
	print('Right Hand Mouth Position complete... moving to Right Hand Up Position')
	time.sleep(0.5)
	RightArmCrossHandUp()
	print('Right Hand Up Position complete... moving back to Initial Pose')
	time.sleep(0.5)
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def Bad():
	time.sleep(0.5)
	RightArmtoMouth()
	print('Right Hand Mouth Position complete... moving to Right Hand Down Position')
	time.sleep(0.5)
	RightArmCrossHandDown()
	print('Right Hand Down Position complete... moving back to Initial Pose')
	time.sleep(0.5)
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def Wonderful():
	BothArmFace()
	print('Both Arm Face Position complete... moving to Both Arm Out Position')
	BothArmOutMidpoint()
	print('Both Arm Out Position complete... moving back to Initial Pose')
	time.sleep(1)
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def Yes():
	Yesfist()
	print('Yes Fist Position complete... moving to Yes Nod Down')
	YesnodDown()
	print('Yes Nod Down complete... moving to Yes Nod Up')
	YesnodUp()
	print('Yes Nod Up complete... moving to Yes Nod Down')
	YesnodDown()
	print('Yes Nod Down complete... moving to Yes Nod Up')
	YesnodUp()
	print('Yes Nod Position complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def No():
	Nofist()
	print('No Fist Position complete... moving to No Fist Open')
	NofistOpen()
	print('No Fist Open complete... moving to No Fist Closed')
	NofistClose()
	print('No Fist closed complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def NTMY():
	NTMYHandsTogether()
	print('NTMY Hands Together complete... moving to the Hand Slide')
	NTMYHandSlide()
	print('Hand Slide complete... moving to the fist bump 1')
	NTMYFistTogether1()
	print('Fist bump 1 complete... moving to the fist bump 2')
	NTMYFistTogether2()
	print('Fist bump 2 complete... moving to the Right hand point')
	RightHandPoint()
	print('Right hand point complete... moving back to Initial Pose')
	time.sleep(1)
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')

def Age():
	AgeRightHandFace()
	print('Hand to face complete... moving to Hand Down position')
	AgeHandMoveDown()
	print('Hand down position complete... moving to the Right hand point')
	RightHandPoint()
	RightHandPoint()
	print('Right hand point complete... moving back to Initial Pose')
	time.sleep(1)
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')

def Repeat():
	RepeatHandsOut()
	print('Repeat hands out complete... moving to halfway point')
	RepeatHalfway()
	print('Halfway point complete. Moving to put gripper in hand')
	RepeatRightHandPalm()
	print('Gripper in hand complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def Please():
	PleaseHandHeart()
	print('Hand to heart movement complete... moving to circle 1')
	PleaseCircle1()
	print('Circle 1 movement complete... moving to circle 2')
	PleaseCircle2()
	print('Circle 2 movement complete... moving to circle 1')
	PleaseCircle1()
	print('Circle 1 movement complete... moving to circle 2')	
	PleaseCircle2()
	print('Circle 2 movement complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def Thankyou():
	RightArmtoMouthThank()
	print('Hand to mouth movement complete... moving to lower hand')
	ThankYouLowerMiddle()
	print('Hand lower complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def YoureWelcome():
	YoureWelcomePalm1()
	print('First movement complete... moving to second position')
	YoureWelcomePalm2()
	print('Second movement complete... moving to third position')
	YoureWelcomePalm3()
	print('Third movement complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def Happy():
	HappyBothArmUp()
	print('Both arms to chest complete... moving to lift them away')
	HappyHandsOpen()
	print('Lifted arms complete... moving arms back to chest')
	HappyBothArmUp()
	print('Both arms to chest complete... moving to lift them away')
	HappyHandsOpen()
	print('Lifted arms complete... moving arms back to chest')
	HappyBothArmUp()
	print('Both arms to chest complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')



def Sad():
	SadHandsFace()
	print('Hands to face complete... moving hands down')
	SadHandsDown()
	print('Hands moved down complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def Tired():
	Tired1()
	print('Tired first position complete... moving to second position')
	Tired2()
	print('Tired second position complete... moving to final position')
	TiredFinal()
	print('Tired Final position complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def Confused():
	ConfusedHandAboveHead()
	print('Hand moved to above head complete... moving to open gripper')
	ConfusedHandAboveHeadOpen()
	print('Gripper open... moving to close gripper')
	ConfusedHandAboveHeadClose()
	print('Gripper closed... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def HAY():
	HAYHands1()
	print('First position complete... moving to second position')
	HAYHands2()
	print('Second position complete... moving to third position')
	HAYHands3()
	print('Third position complete... moving to the final position')
	HAYHandsFinal()
	print('Final position complete... moving to Right hand Point')
	RightHandPoint()
	print('Right hand point complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')


def Name():
	RightHandPoint()
	print('Stop Hand complete... moving to double tap position')
	NameHandTap()
	print('Double tap position complete... moving to hand shake')
	NameHandsUpLeft()
	print('Hand shake left complete... moving to hand shake right')
	NameHandsUpRight()
	print('Hand shake right complete... moving to hand shake left')
	NameHandsUpLeft()
	print('Hand shake left complete... moving to hand shake right')
	NameHandsUpRight()
	print('Hand shake right complete... moving to hand shake left')
	NameHandsUpLeft()
	print('Hand shake left complete... moving to hand shake right')
	NameHandsUpRight()	
	print('Hand shake right complete... moving back to Initial Pose')
	InitialPose()
	print('Initial Pose complete... Full Movement Complete')



def Calibrate():
	InitialPose()
	print('working)')
	time.sleep(2)
	Hi() 					  #good
	time.sleep(2)
	Good()  				  #good
	time.sleep(2)
	Bad() 					  #goodenough
	time.sleep(2)
	Wonderful()				  #good
	time.sleep(2)
	Yes()					  #good
	time.sleep(2)
	No()				#good, need gripper check
	time.sleep(2)
	Please()			#need to check
	time.sleep(2)
	Thankyou()				  #good
	time.sleep(2)
	YoureWelcome()		#needs work
	time.sleep(2)
	Repeat()			#need to check
	time.sleep(2)
	Happy()					  #good
	time.sleep(2)
	Tired()				#need to check 
	time.sleep(2)
	Sad()				#need to check
	time.sleep(2)
	Confused()			#need to check grippers
	time.sleep(2)
	DontKnow()				#good
	time.sleep(2)
	Name()				#all messed up....
	time.sleep(2)
	Age()				#need gripper closed
	time.sleep(2)
	NTMY()				#good, need to check grippers
	time.sleep(2)
	HAY()				#need to check
	time.sleep(2)
	Hi()				#Calibration done. 





def main():
    """RSDK Joint Position Example: Keyboard Control

    Use your dev machine's keyboard to control joint positions.

    Each key corresponds to increasing or decreasing the angle
    of a joint on one of Baxter's arms. Each arm is represented
    by one side of the keyboard and inner/outer key pairings
    on each row for each joint.
    """
    epilog = """
See help inside the example with the '?' key for key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.parse_args(rospy.myargv()[1:])
    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_keyboard")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example...")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)


    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lj = left.joint_names()
    rj = right.joint_names()
    grip_left.calibrate()
    grip_right.calibrate()
    InitialPose()

    print("Enabling robot... ")
    rs.enable()
    print("Done.")


if __name__ == '__main__':
	main()
    #This is where the input from speech recognition should go. If statements to be print(open(file).read())made later
	
	Calibrate()

	file = "command.txt"
	while(True):
		if os.path.exists(file) and os.path.getsize(file) > 0:
			print(open(file).read())
			f = open(file).read()
			if f == 'I do not know':
				print('Making -Dont Know- Movement')
				DontKnow()
				
			elif f == 'hello':
				print('Making -Hello- Movement')
				Hi()
				#os.remove("command.txt")


			elif f == 'good':
				print('Making - Good- Movement')
				Good()
				#os.remove("command.txt")
			
			elif f == 'bad':
				print('Making -Bad- Movement')
				Bad()
				#os.remove("command.txt")

			elif f == 'wonderful':
				print('Making -Wonderful- Movement')
				Wonderful()
				#os.remove("command.txt")


			elif f == 'yes':
				print('Making -Yes- Movement')
				Yes()
				#os.remove("command.txt")


			elif f == 'no':
				print('Making -No- Movement')
				No()
				#os.remove("command.txt")


			elif f == 'nice to meet you':
				print('Making -Nice to Meet You- Movement')
				NTMY()
				#os.remove("command.txt")


			elif f == 'how old are you':
				print('Making -How old are you? - Movement')
				Age()
				#os.remove("command.txt")


			elif f == 'repeat':
				print('Making -Repeat- Movement')
				Repeat()
				#os.remove("command.txt")


			elif f == 'please':
				print('Making -Please- Movement')
				Please()
				#os.remove("command.txt")

			elif f == 'thank you':
				print('Making -Thank You- Movement')
				Thankyou()
				#os.remove("command.txt")


			elif f == 'you are welcome':
				print('Making -Youre Welcome- Movement')
				YoureWelcome()
				#os.remove("command.txt")


			elif f == 'happy':
				print('Making -Happy- Movement')
				Happy()
				#os.remove("command.txt")


			elif f == 'sad':
				print('Making -Sad- Movement')
				Sad()
				#os.remove("command.txt")


			elif f == 'tired':
				print('Making -Tired- Movement')
				Tired()
				#os.remove("command.txt")


			elif f == 'confused':
				print('Making -Confused- Movement')
				Confused()
				#os.remove("command.txt")


			elif f == 'how are you':
				print('Making -How are you?- Movement')
				HAY()
				#os.remove("command.txt")


			elif f == 'what is your name':
				print('Making -What is you name?- Movement')
				Name()
				#os.remove("command.txt")

			elif f == 'bye':
				print('Making -Goodbye- Movement')
				Hi()
			
			else:
				DontKnow()

			os.remove("command.txt")




	#make hug
