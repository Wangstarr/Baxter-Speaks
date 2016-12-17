# #!/usr/bin/env python3
import speech_recognition as sr
import os

#This package uses Google's speech recognition package

r = sr.Recognizer()
m = sr.Microphone()
#set threhold level
with m as source: r.adjust_for_ambient_noise(source)
print("Please remain silent during calibration")
print("Set minimum energy threshold to {}".format(r.energy_threshold))
# obtain audio from the microphone

with sr.Microphone() as source:
    print("Say something!")
    audio = r.listen(source)
file = open("command.txt", "w")
recognized = r.recognize_google(audio)
file.write(recognized)
### If using internet connection ###
# os.system("scp ~/Documents/ee106a/command.txt ee106a-ada@ashby.cs:~/ros_workspaces/lab7_baxter/src/lab7/src/command.txt")
### If using local Zumy network ###
os.system("scp ~/Documents/ee106a/command.txt team23@192.168.1.100:~/ros_workspaces/Final_Project/src/Baxter/src/command.txt")
print(recognized)