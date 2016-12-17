#!/usr/bin/python
# Load the required library
import pygame, time
from gtts import gTTS
import os
import spell

#This package uses Google's text to speech package

# os.system("scp ee106a-ada@ashby.cs:~/ros_workspaces/lab6/src/lab6/src/interpreted.txt interpreted.txt")
# file = "interpreted.txt"
response1 = ''
try: 
    while(True):
        ##If working over the internet, uncomment following line##
        # os.system("scp ee106a-ada@ashby.cs:Downloads/interpreted.txt interpreted.txt")
        ##If working over the ZUMY network, uncomment following line##
        os.system("scp team23@192.168.1.100:Downloads/interpreted.txt interpreted.txt")

        file = "interpreted.txt"
        # file = "testing.txt"
        response = open(file).read()
        print(response)
        #print(open(file).read())
        if os.path.exists(file) and os.path.getsize(file) > 0:
            # response = spell.correction(response)
            print("spell checked: " + response)
            if response != response1:
                tts = gTTS(text=response, lang='en')
                tts.save("talk.mp3")
                pygame.mixer.init()
                pygame.mixer.music.set_volume(1)
                pygame.mixer.music.load('/home/makani/Documents/ee106a/talk.mp3')
                print("Baxter is talking")
                pygame.mixer.music.play(0)
                print("wait...")
                time.sleep(3)
                response1 = response
            else:
                print('Waiting for new text file input')
        else:
            print("File is empty or does not exist")
        
except KeyboardInterrupt:
    pass 