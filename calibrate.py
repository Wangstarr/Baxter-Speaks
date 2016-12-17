#!/usr/bin/env python

## The below code is used to calibrate, or record transformations for, each signed letter of the ASL alphabet and is then used as a reference during the translation stage of the sign language recognition.
## The output is a written .txt file of the calibration matrix, and that file can be overwritten each time you want to recalibrate.  We suggest calibration for each new user for most successful translation, although it is not necessary for decent functionality.

## NOTE: The current code requires the user to calibrate all 26 letters and SPACE each time they want to recalibrate, in the future we will add in the ability to spot calibrate one letter at a time if they desire
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

#Main function which is run during execution
def do_stuff():
    #jump is used for indexing our transformation matrix
    jump = [0,13,12,11,10,9,8,7,6,5,4,3,2,1,0]
    
    # Below we initialize the tf listener, wait for the user to press enter to begin, and initialize an all zeros dictionary for our stored transformation matrices or "alphabet" which will later be filled with transformations that are seen by the camera
    listener = tf.TransformListener()
    raw_input("Press enter to begin: ")
    bindings = sums = []
    for i in np.linspace(1,106,106):
        bindings.append([np.zeros((3,3)),np.zeros((3,1))])
    sums = bindings
    print("CALIBRATING --> Please follow the directions :) ")

    alphabet = {'A': bindings, 'B': bindings, 'C': bindings, 'D': bindings, 'E': bindings, 'F': bindings, 'G': bindings, 'H': bindings, 'I': bindings, 'J': bindings, 'K': bindings, 'L': bindings, 'M': bindings, 'N': bindings, 'O': bindings, 'P': bindings, 'Q': bindings, 'R': bindings, 'S': bindings, 'T': bindings, 'U': bindings, 'V': bindings, 'W': bindings, 'X': bindings, 'Y': bindings, 'Z': bindings, '_' : bindings}
    letters = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z', '_']


    # Here we make our trans_final matrix which gives the alphabet matrix indices for the transformations that should be seen if each sign is done properly (all AR tags recognized) during the calibration stage... we'll use this later as quality control during the calibration loop
    tag_trans_eyeball = {'A': [0,6,7,8,9,12], 'B': [0,1,2,3,4,12], 'C': [5,11], 'D': [0,4, 6, 7, 8, 12], 'E': [0, 6, 7, 8, 9, 12], 'F': [0,1,2,3,9,12], 'G': [9, 10], 'H': [8, 9, 10], 'I': [0,1,7,8,9, 12], 'J': [0,1,7,8,9, 12], 'K': [0,3,4,6, 7, 12], 'L': [0,4,5,6,7,8], 'M': [0, 6, 7, 8, 9], 'N': [0,6, 7, 8, 9], 'O': [5,11], 'P': [13, 14], 'Q': [13, 14], 'R': [0,3,4,12], 'S': [0,6,7,12], 'T': [0, 6,7,8,9], 'U': [0,3,4,12], 'V': [0,3,4,12], 'W': [0,2,3,4,12], 'X': [0, 6,7,8,12], 'Y': [0, 1, 5,7,8,9], 'Z': [0,4, 6, 7, 8, 12], '_' : [0,1,2,3,4,5]}
    
    trans_final = {'A': [], 'B': [], 'C': [], 'D': [], 'E': [], 'F': [], 'G': [], 'H': [], 'I': [], 'J': [], 'K': [], 'L': [], 'M': [], 'N': [], 'O': [], 'P': [], 'Q': [], 'R': [], 'S': [], 'T': [], 'U': [], 'V': [], 'W': [], 'X': [], 'Y': [], 'Z': [], '_':[]}

    #make trans_final
    for i in letters:
        if len(tag_trans_eyeball[i]) == 1:
            trans_final[i] = [0]
        else:
            for n in tag_trans_eyeball[i]:
                for m in tag_trans_eyeball[i]:
                    if m > n:
                        trans_final[i].append(str(n + sum(jump[0:(n+1)]) + (m - n)))


    #The below while loop iterates through each letter of the alphabet and space and 'listens' for 3 seconds to record the average transformations between any pair of AR tags that are seen by the camera
    loop = 1         
    while loop <= 27:
        #initialize useful loop variables
        n = np.zeros(106)  
        scale = s = 0
        bindings = sums = []
        for i in np.linspace(1,106,106):
            bindings.append([np.zeros((3,3)),np.zeros((3,1))])
        sums = bindings
        listener.clear()
        t_end = time.time() + 3
        print("Calibrating " + letters[loop-1])
        cal_check = []
        #this loop listens for each letter
        while time.time() < t_end:
            for i in [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]:
                for j in [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]:            
                    if (j > i):

                        #only store transformations that exist
                        if (listener.frameExists('ar_marker_' + str(i))) and (listener.frameExists('ar_marker_'+str(j))):
                        #Get translation and rotation from tf topic
                            try:
                                (trans, rot) = listener.lookupTransform('ar_marker_' + str(i), 'ar_marker_' + str(j), rospy.Time(0))
                        #Convert quaternion to an omega and theta.
                                (omega,theta) = eqf.quaternion_to_exp(rot)
                        #Create relative configuration of each AR tag to AR tag 0
                                g = eqf.create_rbt(omega,theta,trans)
                            #Configuration matrix is initially zero; skip index 0 to match index with AR tag number
                                if (np.sum(bindings[i + sum(jump[0:(i+1)]) + (j - i)][0]) == 0):
                                    print(str(i) + ","+ str(j))
                                    cal_check.append(str(i) + str(j))
                                    sums[i + sum(jump[0:(i+1)]) + (j - i)][0] = bindings[i + sum(jump[0:(i+1)]) + (j - i)][0] = g[0:3,0:3]
                                    sums[i + sum(jump[0:(i+1)]) + (j - i)][1] = bindings[i + sum(jump[0:(i+1)]) + (j - i)][1] = g[0:3,3]
                                #Sum the values in the configuration matrix to later be averaged
                                else:
                                    n[i + sum(jump[0:(i+1)]) + (j - i)] = n[i + sum(jump[0:(i+1)]) + (j - i)] + 1
                                    sums[i + sum(jump[0:(i+1)]) + (j - i)][0] = (sums[i + sum(jump[0:(i+1)]) + (j - i)][0] + g[0:3,0:3])
                                    sums[i + sum(jump[0:(i+1)]) + (j - i)][1] = (sums[i + sum(jump[0:(i+1)]) + (j - i)][1] + g[0:3,3])
                            except:
                                continue
            
            #SCALE FACTOR loop below: needed because depending on distance from camera different translation values will be recored; we have three different scale transformations 0 - 17, 11 - 15, and 13 - 16 depending on which configuration the hand is in because there isnt one pair of tags that can be seen for all signs            
            if (listener.canTransform('ar_marker_0','ar_marker_17',rospy.Time(0))):   
                try:
                    (trans, rot) = listener.lookupTransform('ar_marker_0', 'ar_marker_17', rospy.Time(0))
                except:
                    continue
                        #Convert quaternion to an omega and theta.
                (omega,theta) = eqf.quaternion_to_exp(rot)
                g_scale = eqf.create_rbt(omega,theta,trans)
                case = 0;

                if scale == 0:
                    scale = np.linalg.norm((g_scale[0:3,3]))
                    s = s + 1
                else:
                    scale = scale + np.linalg.norm((g_scale[0:3,3]))
                    s = s + 1

            if (listener.canTransform('ar_marker_11','ar_marker_15',rospy.Time(0))):    
                try:
                    (trans, rot) = listener.lookupTransform('ar_marker_11', 'ar_marker_15', rospy.Time(0))
                except:
                    continue
                        #Convert quaternion to an omega and theta.
                (omega,theta) = eqf.quaternion_to_exp(rot)
                g_scale = eqf.create_rbt(omega,theta,trans)
                case = 1;
                if scale == 0:
                    scale = np.linalg.norm((g_scale[0:3,3]))
                    s = s + 1
                else:
                    scale = scale + np.linalg.norm((g_scale[0:3,3]))
                    s = s + 1

            if (listener.canTransform('ar_marker_13','ar_marker_16',rospy.Time(0))):    
                try:
                    (trans, rot) = listener.lookupTransform('ar_marker_13', 'ar_marker_16', rospy.Time(0))
                except:
                    continue
                        #Convert quaternion to an omega and theta.
                (omega,theta) = eqf.quaternion_to_exp(rot)
                g_scale = eqf.create_rbt(omega,theta,trans)
                case = 2;

                if scale == 0:
                    scale = np.linalg.norm((g_scale[0:3,3]))
                    s = s + 1
                else:
                    scale = scale + np.linalg.norm((g_scale[0:3,3]))
                    s = s + 1

        #letting the user know if the scale tags were seen or not
        if s != 0:
            scale = scale/s
            print("SCALE HAPPY :)")
            #print(case)
        else:
            scale = 1
            print("WARNING: S = 0; if G or H was signed all is OK")
            #print(case)

        #averaging out all respective transformations seen during the 3 seconds
        for i in xrange(1,106):
        #Perform calculation if tag was seen
            if (np.sum(bindings[i][0]) != 0):
                bindings[i][0] = sums[i][0]/n[i]
                bindings[i][1] = sums[i][1]/n[i]
        alphabet[letters[loop-1]] = bindings
        print(str(loop) + " :loop number")
        print(letters[loop-1])

        #Quality assurance, if proper tags weren't seen user is prompted to recalibrate that letter
        if len(trans_final[letters[loop - 1]]) == len(cal_check):
            raw_input("Press enter when you're ready to move to the next letter.")
            for z in xrange(0,106):
                alphabet[letters[loop-1]][z].append(scale)
            loop = loop + 1 #successful calibration
        else:
            raw_input("Try calibration again for same letter")

    # below we write and save a timestamped file to be later read by the final translation code
    timeout = str(time.time())
    with open('alphabet.' + timeout + 'txt', 'wb') as f:
        pickle.dump(alphabet, f)
    with open('alphabet.' + timeout + 'txt', 'rb') as out_file:
        my_list = pickle.load(out_file)


    return my_list

#general code needed to run the function
if __name__=='__main__':
    rospy.init_node('final_code')
    do_stuff()
    print('main function')
