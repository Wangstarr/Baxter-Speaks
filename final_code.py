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
import calibrate as cal
import pickle
from std_msgs.msg import String
import os

def do_stuff():

	#Initialize values of arrays:
    jump = [0,13,12,11,10,9,8,7,6,5,4,3,2,1,0]
    bindings = sums = []
    for i in np.linspace(1,106,106):
        bindings.append([np.zeros((3,3)),np.zeros((3,1))])
    sums = bindings

    tags = {'A': [0,6,7,8,9,12], 'B': [0,1,2,3,4,12], 'C': [5,11], 'D': [0,4,6,7,8,12], 'E': [0,6,7,8,9,12], 'F': [0,1,2,3,9,12], 'G': [9,10], 'H': [8,9,10], 'I': [0,1,7,8,9, 12], 'J': [0,1,7,8,9, 12], 'K': [0,3,4,6, 7, 12], 'L': [0,4,5,6,7,8], 'M': [0, 6, 7, 8, 9], 'N': [0,6, 7, 8, 9], 'O': [5,11], 'P': [13, 14], 'Q': [13, 14], 'R': [0,3,4,12], 'S': [0,6,7,12], 'T': [0, 6,7,8,9], 'U': [0,3,4,12], 'V': [0,3,4,12], 'W': [0,2,3,4,12], 'X': [0, 6,7,8,12], 'Y': [0, 1, 5,7,8,9], 'Z': [0,4, 6, 7, 8, 12], '_' : [0,1,2,3,4,5]}

    letters = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z','_']

    all_frames_rec = []
    all_bindings = []
    all_scales = []
    all_moves = []

    #Wait for user to press a key to begin
    raw_input("Press enter to begin: ")

    #Bring in calibration file
    with open('alphabet.txt', 'rb') as out_file:
        alphabet = pickle.load(out_file)
    print('Listening...')

    ###################LISTENING LOOP###########################    
    listener = tf.TransformListener()
    stop = False
    k = 1

    ###################### ONE LETTER BEGIN ##################
    while stop == False:
        n = np.zeros(106) #counter for averaging
        s = scale = 0
        frames = []
        listener.clear()

        t_end = time.time() + 1.3
        print("Letter " + str(k))
        avg = True
        g_init = []
        g_final = []
        while time.time() < t_end:

        	#Get translation and rotation of each tag relative to origin tag from tf topic if the frame exists
            #Check for stop signal
            markExist = np.zeros(15)

            for i in xrange(6,10):
                if listener.canTransform(  ('ar_marker_' + str(i)) ,('ar_marker_10'), rospy.Time(0)) == True:
                    markExist[i] = 1

            if(markExist[6] and markExist[7] and markExist[8] and markExist[9]):
                print("Stop signal received. Proceeding to translation.")
                stop = True
                avg = False
                break

            ####SCALE FACTOR#### 0 17 is all but PQ, CO and PQ is 1316 and CO is 1511
            if (listener.canTransform('ar_marker_0','ar_marker_17',rospy.Time(0))):    
                try:
                    (trans, rot) = listener.lookupTransform('ar_marker_0', 'ar_marker_17', rospy.Time(0))
                except:
                    continue
                        #Convert quaternion to an omega and theta.
                (omega,theta) = eqf.quaternion_to_exp(rot)
                g_scale = eqf.create_rbt(omega,theta,trans)

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

                if scale == 0:
                    scale = np.linalg.norm((g_scale[0:3,3]))
                    s = s + 1
                else:
                    scale = scale + np.linalg.norm((g_scale[0:3,3]))
                    s = s + 1

			#Determine if the sign is a moving sign by comparing the initial transformation from the USB cam to AR Tag 8 to the final transformation from the USB cam to AR Tag 8.
			#Obtain the initial configuration here.
            if g_init == []:
                if (listener.canTransform('usb_cam','ar_marker_8',rospy.Time(0))):
                    try:
                        (trans, rot) = listener.lookupTransform('usb_cam', 'ar_marker_8', rospy.Time(0))
                    except:
                        continue
                    (omega,theta) = eqf.quaternion_to_exp(rot)
                    g_init = eqf.create_rbt(omega,theta,trans)

            #Check all 110 transformations
            if avg == True:
                for i in [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]:
                    for j in [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14]:            
                        if (i < j):
                            if (listener.frameExists('ar_marker_' + str(i))) and (listener.frameExists('ar_marker_'+str(j))):
                                
                        #Get translation and rotation from tf topic
                                try:
                                    (trans, rot) = listener.lookupTransform('ar_marker_' + str(i), 'ar_marker_' + str(j), rospy.Time(0))
                                except:
                                    continue
                                (omega,theta) = eqf.quaternion_to_exp(rot)
                        		#Create relative configuration of each AR tag to AR tag 0
                                g = eqf.create_rbt(omega,theta,trans)
                       			#Configuration matrix is initially zero; skip index 0 to match index with AR tag number
                                if (np.sum(bindings[i + sum(jump[0:(i+1)]) + (j - i)][0]) == 0):
                                    print(str(i)+","+ str(j))
                                    frames.append(i + sum(jump[0:(i+1)]) + (j - i))
                                    n[i + sum(jump[0:(i+1)]) + (j - i)] = n[i + sum(jump[0:(i+1)]) + (j - i)] + 1
                                    sums[i + sum(jump[0:(i+1)]) + (j - i)][0] = bindings[i + sum(jump[0:(i+1)]) + (j - i)][0] = g[0:3,0:3]
                                    sums[i + sum(jump[0:(i+1)]) + (j - i)][1] = bindings[i + sum(jump[0:(i+1)]) + (j - i)][1] = g[0:3,3]
                                	#Sum the values in the configuration matrix to later be averaged
                                else:
                                    n[i + sum(jump[0:(i+1)]) + (j - i)] = n[i + sum(jump[0:(i+1)]) + (j - i)] + 1
                                    sums[i + sum(jump[0:(i+1)]) + (j - i)][0] = (sums[i + sum(jump[0:(i+1)]) + (j - i)][0] + g[0:3,0:3])
                                    sums[i + sum(jump[0:(i+1)]) + (j - i)][1] = (sums[i + sum(jump[0:(i+1)]) + (j - i)][1] + g[0:3,3])          

            #Obtain the final configuration from USB cam to AR Tag 8
            if (listener.canTransform('usb_cam','ar_marker_8',rospy.Time(0))):    
                try:
                    (trans, rot) = listener.lookupTransform('usb_cam', 'ar_marker_8', rospy.Time(0))
                except:
                    continue
                (omega,theta) = eqf.quaternion_to_exp(rot)
                g_final = eqf.create_rbt(omega,theta,trans)

        #Determine if the sign is static. If the difference between the initial and final rotation of AR Tag 8 relative to the USB cam is large enough, then consider the tag as moving.
        #The threshold is arbitrary. However in static signs, the values obtained for move_rot are on the hundredths order of magnitude. It is obvious when a tag is static or dynamic.
        move_rot = 0
        move = 0

        if g_init != [] and g_final != []:
            for i in xrange(0,2):
                move_rot = move_rot + abs(g_init[i][i]-g_final[i][i])
            if move_rot > 0.3:
                move = 1            

        #Perform calculation to obtain average and assign final value to bindings
        if avg == True:
            for i in xrange(1,106):
                #Perform calculation if tag was seen
                if (np.sum(bindings[i][0]) != 0):
                    bindings[i][0] = sums[i][0]/n[i]
                    bindings[i][1] = sums[i][1]/n[i]
        if s != 0:
            scale = scale/s
        else:
            scale = 1

        if (s != 0) or (bindings[91][1][1] != 0):
            all_frames_rec.append(frames)
            all_bindings.append(bindings)
            all_scales.append(scale)
            all_moves.append(move)
            if avg == True:
                k = k + 1
                raw_input("Press enter when you're ready to move to the next letter.")
        else:
            if avg == True:
                raw_input("Something went wrong, please sign again (HINT: check if the scale tags are in view)")

        #Reset bindings and sums to zero for the next letter.
        bindings = sums = []
        for i in np.linspace(1,106,106):
            bindings.append([np.zeros((3,3)),np.zeros((3,1))])
        sums = bindings       
        ###################### ONE LETTER END ##################
    ###################LISTENING LOOP###########################

#####################TRANSLATE################################
	#Initialize the arrays used for translating.
    phrase_out = []
    tag_trans = {'A': [0,6,7,8,9,12], 'B': [0,1,2,3,4,12], 'C': [5,11], 'D': [0,4, 6, 7, 8, 12], 'E': [0, 6, 7, 8, 9, 12], 'F': [0,1,2,3,9,12], 'G': [9, 10], 'H': [8, 9, 10], 'I': [0,1,7,8,9, 12], 'J': [0,1,7,8,9, 12], 'K': [0,3,4,6, 7, 12], 'L': [0,4,5,6,7,8], 'M': [0, 6, 7, 8, 9], 'N': [0,6, 7, 8, 9], 'O': [5,11], 'P': [13, 14], 'Q': [13, 14], 'R': [0,3,4,12], 'S': [0,6,7,12], 'T': [0, 6,7,8,9], 'U': [0,3,4,12], 'V': [0,3,4,12], 'W': [0,2,3,4,12], 'X': [0, 6,7,8,12], 'Y': [0, 1, 5,7,8,9], 'Z': [0,4, 6, 7, 8, 12], '_' : [0,1,2,3,4,5]}
    trans_final = {'A': [], 'B': [], 'C': [], 'D': [], 'E': [], 'F': [], 'G': [], 'H': [], 'I': [], 'J': [], 'K': [], 'L': [], 'M': [], 'N': [], 'O': [], 'P': [], 'Q': [], 'R': [], 'S': [], 'T': [], 'U': [], 'V': [], 'W': [], 'X': [], 'Y': [], 'Z': [], '_': []}

    #Initialize trans_final with the indices in which a transformation should be found for that letter.
    for i in letters:
        if len(tag_trans[i]) == 1:
            trans_final[i] = [0]
        else:
            for n in tag_trans[i]:
                for m in tag_trans[i]:
                    if m > n:
                        trans_final[i].append(str(n + sum(jump[0:(n+1)]) + (m - n)))

    #Start the translation process
    for k in xrange(0, len(all_frames_rec)-1):
        letters = ['A','B','C','D','E','F','G','H','I','J','K','L','M','N','O','P','Q','R','S','T','U','V','W','X','Y','Z', '_']
        possible = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0, 'G': 0, 'H': 0, 'I': 0, 'J': 0, 'K': 0, 'L': 0, 'M': 0, 'N': 0, 'O': 0, 'P': 0, 'Q': 0, 'R': 0, 'S': 0, 'T': 0, 'U': 0, 'V': 0, 'W': 0, 'X': 0, 'Y': 0, 'Z': 0, '_' : 0}
        matches = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0, 'G': 0, 'H': 0, 'I': 0, 'J': 0, 'K': 0, 'L': 0, 'M': 0, 'N': 0, 'O': 0, 'P': 0, 'Q': 0, 'R': 0, 'S': 0, 'T': 0, 'U': 0, 'V': 0, 'W': 0, 'X': 0, 'Y': 0, 'Z': 0, '_' : 0}
        error = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0, 'G': 0, 'H': 0, 'I': 0, 'J': 0, 'K': 0, 'L': 0, 'M': 0, 'N': 0, 'O': 0, 'P': 0, 'Q': 0, 'R': 0, 'S': 0, 'T': 0, 'U': 0, 'V': 0, 'W': 0, 'X': 0, 'Y': 0, 'Z': 0, '_' : 0}
        delta = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0, 'G': 0, 'H': 0, 'I': 0, 'J': 0, 'K': 0, 'L': 0, 'M': 0, 'N': 0, 'O': 0, 'P': 0, 'Q': 0, 'R': 0, 'S': 0, 'T': 0, 'U': 0, 'V': 0, 'W': 0, 'X': 0, 'Y': 0, 'Z': 0, '_' : 0}
 
        #Determine if AR tags seen in the signed letter are too many, exact, or too little for a certain letter 
        exact = []
        over = []
        under = []
        for i in letters:
            for j in all_frames_rec[k]:
                if np.sum(alphabet[i][j][1]) != 0:
                    matches[i] = matches[i] + 1
                else:
                    continue

            delta[i] = matches[i] - len(trans_final[i])

        #Append the letter to the exact, over, and under matches for letter 'k' signed
            if (delta[i] == 0) and (len(all_frames_rec[k]) == len(trans_final[i])):
                exact.append(i)

            elif (delta[i] >= 0) and (len(all_frames_rec[k]) > len(trans_final[i])):
                over.append(i)
             
            elif delta[i] < 0:
                under.append(i)

        #Now that we have determined which letters are possible, we want to narrow down the possibilities by comparing to the calibrated letters
        error_rot = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0, 'G': 0, 'H': 0, 'I': 0, 'J': 0, 'K': 0, 'L': 0, 'M': 0, 'N': 0, 'O': 0, 'P': 0, 'Q': 0, 'R': 0, 'S': 0, 'T': 0, 'U': 0, 'V': 0, 'W': 0, 'X': 0, 'Y': 0, 'Z': 0, '_': 0}
        error_trans = {'A': 0, 'B': 0, 'C': 0, 'D': 0, 'E': 0, 'F': 0, 'G': 0, 'H': 0, 'I': 0, 'J': 0, 'K': 0, 'L': 0, 'M': 0, 'N': 0, 'O': 0, 'P': 0, 'Q': 0, 'R': 0, 'S': 0, 'T': 0, 'U': 0, 'V': 0, 'W': 0, 'X': 0, 'Y': 0, 'Z': 0, '_' : 0} 

        #For exact matches, there are specific cases that we must specifically consider to increase the accuracy
        if exact != []:
        	#Special case for M,N,T. The distinguishing factor is which two fingers the thumb is inserted between. The two fingers that have the largest translational difference determines which letter was signed.
        	#i.e. if the largest translation between the tag pairs (6,7),(7,8),(8.9) is between pair (6,7), then the thumb has been inserted between the pinky and ring finger, which is letter M.
            if exact == ['M', 'N', 'T']:
                em = np.linalg.norm(all_bindings[k][70][1])
                en = np.linalg.norm(all_bindings[k][78][1])
                tee = np.linalg.norm(all_bindings[k][85][1])
                if (em > en) and (em > tee):
                    phrase_out.append('M')
                if (em < en) and (en > tee):
                    phrase_out.append('N')
                if (em < tee) and (en < tee):
                    phrase_out.append('T')
            #Special case for R,U,V. R has the index and middle finger crossed, resulting in a negative translation between (3,4).
            #U and V and distinguished by looking at the magnitude of the translation between (3,4). Larger translation means V, because in V the index and middle are spread apart more.
            elif exact == ['R', 'U', 'V']:
                error_transU = 0
                error_transV = 0
                if all_bindings[k][40][1][0] < 0:
                    phrase_out.append('R')
                else:
                    for i in [0,1,2]:
                        for z in xrange(0,106):
                            all_bindings[k][z][1] = all_bindings[k][z][1]*all_scales[k]/alphabet['U'][1][2]
                        error_transU = error_transU + (all_bindings[k][40][1][i] - alphabet['U'][40][1][i])**2

                        for z in xrange(0,106):
                            all_bindings[k][z][1] = all_bindings[k][z][1]*all_scales[k]/alphabet['U'][1][2]
                        error_transV = error_transV + (all_bindings[k][40][1][i] - alphabet['V'][40][1][i])**2
                    if error_transV > error_transU:
                        phrase_out.append('U')
                    else:
                        phrase_out.append('V')
            #Special case for C,O. The rotation of tag 5 on the thumb changes depending on whether C or O is signed. In signing O, the axes of tag 5 align with the axes of tag 11 on the side of the hand near the pinky.
            #In signing C, the axes of tag 5 misalign with tag 11. The decision is made by determining if the signed letter is closer to the rotation of C or O.
            elif exact == ['C', 'O']:
                error_rot = error_rotC = error_rotO = error_transC = error_transO = 0
                for i in [0,1]:
                    error_rot = error_rot + all_bindings[k][66][0][i,i]
                    error_rotC = error_rotC + alphabet['C'][66][0][i,i]
                    error_rotO = error_rotO + alphabet['O'][66][0][i,i]

                if abs(error_rot - error_rotC) > abs(error_rot - error_rotO):
                    phrase_out.append('O')
                else:
                    phrase_out.append('C')

            #Special case for A,E. The translation of tag 12 relative to tags 6,7,8, and 9 are larger in magnitude for E than in A.
            elif exact == ['A', 'E']:
                error_transA = 0
                error_transE = 0

                for i in [75,82,88,93]:
                    error_transA = error_transA + (all_bindings[k][i][1][1] - alphabet['A'][i][1][1])**2
                    error_transE = error_transE + (all_bindings[k][i][1][1] - alphabet['E'][i][1][1])**2

                if error_transA > error_transE:
                    phrase_out.append('E')
                else:
                    phrase_out.append('A')

            #Special case for D,Z. If in the signing phase, the sign was determined to have moved, then the letter is Z. If the sign was a static sign, then the letter is D.
            elif exact == ['D', 'Z']:
                if all_moves[k] == 1:
                    phrase_out.append('Z')
                else:
                    phrase_out.append('D')

            #Special case for I,J. If in the signing phase, the sign was determined to have moved, then the letter is J. If the sign was a static sign, then the letter is I.
            elif exact == ['I', 'J']:
                if all_moves[k] == 1:
                    phrase_out.append('J')
                else:
                    phrase_out.append('I')
            #If letter matches none of these special cases then look purely at a least squares error calculation comparison with the calibrated letters. Select the letter with the smallest error.
            else:
                for m in exact:
                    for z in xrange(0,106):
                            all_bindings[k][z][1] = all_bindings[k][z][1]*all_scales[k]/alphabet[m][1][2]
                    for l in all_frames_rec[k]:
                        for i in [0,1,2]:
                            for j in [0,1,2]:
                                error_rot[m] = error_rot[m] + (all_bindings[k][l][0][i,j] - alphabet[m][l][0][i,j])**2
                                  
                        for i in [0,1,2]:
                            error_trans[m] = error_trans[m] + (all_bindings[k][l][1][i] - alphabet[m][l][1][i])**2

                    error[m] = error_trans[m] + error_rot[m]

                for i in letters:
                    if error[i] == 0:
                        error[i] = 100000000
                phrase_out.append(min(error, key=error.get))

        #If there were no exact matches and the camera saw more tags than a letter should have, do a least squares error calculation with the "over" letters.
        elif over != []:
            for m in over:
                for z in xrange(0,106):
                            all_bindings[k][z][1] = all_bindings[k][z][1]*all_scales[k]/alphabet[m][1][2]
                for l in all_frames_rec[k]:
                    for i in [0,1,2]:
                        for j in [0,1,2]:
                            error_rot[m] = error_rot[m] + (all_bindings[k][l][0][i,j] - alphabet[m][l][0][i,j])**2
                              
                    for i in [0,1,2]:
                        error_trans[m] = error_trans[m] + (all_bindings[k][l][1][i] - alphabet[m][l][1][i])**2
                error[m] = error_trans[m] + error_rot[m]

            for i in letters:
                if error[i] == 0:
                    error[i] = 100000000
            phrase_out.append(min(error, key=error.get))

        #If there were no exact matches and no over matches, then compute least squares error calculations with letters in which less tags were seen.
        else:
            for m in under:
                for z in xrange(0,106):
                            all_bindings[k][z][1] = all_bindings[k][z][1]*all_scales[k]/alphabet[m][1][2]
                for l in all_frames_rec[k]:
                    for i in [0,1,2]:
                        for j in [0,1,2]:
                            error_rot[m] = error_rot[m] + (all_bindings[k][l][0][i,j] - alphabet[m][l][0][i,j])**2
                              
                    for i in [0,1,2]:
                        error_trans[m] = error_trans[m] + (all_bindings[k][l][1][i] - alphabet[m][l][1][i])**2

                error[m] = error_trans[m] + error_rot[m]

            for i in letters:
                if error[i] == 0:
                    error[i] = 100000000
            phrase_out.append(min(error, key=error.get))
            print("Chose " + (min(error, key=error.get) + "from UNDER."))

    #Print the resulting signed phrase to the user
    if phrase_out == []:
        phrase_out = 'YOU DID NOT SIGN ANYTHING'
    else:
        print(phrase_out)

    #Convert underscore to space
    phrase_txt = []
    for x in xrange(0,len(phrase_out)):
        if phrase_out[x] == '_':
            phrase_txt.append(' ')
        else:
            phrase_txt.append(phrase_out[x])

    phrase_txt = ''.join(phrase_txt)
    print(phrase_txt)

    #Create a text file containing the signed phrase
    # file = open("/var/local/home/team23/Downloads/interpreted.txt","w")
    file = open("/home/cc/ee106a/fa16/class/ee106a-ada/Downloads/interpreted.txt","w")
    file.write(phrase_txt)
    file.close()

if __name__=='__main__':
	rospy.init_node('final_code')
	do_stuff()
	print('main function')