#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import os
import rospy
import rospkg
from std_msgs.msg import String

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
from optparse import OptionParser


import numpy as np
import soundfile as sf
import requests
import re

import speech_recognition as ssr
from pydub import AudioSegment
from gtts import gTTS
import random


##Import all ros message files
from std_msgs.msg import String
from pepper_conversation.msg import rasa_question

DEFAULT_PEPPER_IP = "10.10.3.129"
#NAO_IP = "10.10.3.129"


class ConversationModule(ALModule):
    def __init__(self, name,language,pepper_ip,pepper_id):
        ALModule.__init__(self, name)
        self.tts = ALProxy("ALTextToSpeech")
        self.ats = ALProxy("ALAnimatedSpeech")

        self.bodyLanguageConfiguration = {"bodyLanguageMode":"contextual"}
        print("Name of the module:" ,self.getName())
        print("Language to be used : ",language)


        #self.tts.say("Hello, world!")
        
        self.audio_player = ALProxy("ALAudioPlayer")
        self.audio = ALProxy( "ALAudioDevice")
        self.audio.setClientPreferences(self.getName(),48000,0,0)
        self.audio.subscribe(self.getName())

        self.absolute_file_path = self.get_ros_package_path()
        self.RAW_FILE = self.absolute_file_path + '/audio_recordings/raw_file.raw'
        self.WAV_FILE = self.absolute_file_path + '/audio_recordings/pepper.wav'
        
        self.audioRecordingStarted = False
        self.answeringPreviousQuestion  =False

        self.outFile=None
        self.startTime=0
        self.r = ssr.Recognizer()
        self.uselanguage = language
        self.pepper_ip = pepper_ip
        self.listeningTimeout = 6.5
        self.pepper_id = pepper_id

        self.greetingPrefix=['Hi, ','Hello, ','Nuh.muh.stay']

        #####All Publisher and Subscriber initialization

        self.led_status_pub = rospy.Publisher('led_status_data_'+self.pepper_id, String, queue_size=10)
        self.question_pub_topic = rospy.Publisher('/rasa_ping_'+self.pepper_id, rasa_question, queue_size=10)
       
        self.namesToAppend=[]
        self.refreshNamesTimeout = 60*2 #every 2 mins the names already recognised will be refreshed.This is done to prevent continuous calling of names.
        self.lastRefreshTime = self.get_current_timestamp()

    def get_ros_package_path(self):

        rp = rospkg.RosPack()
        return(rp.get_path('pepper_conversation'))

    def get_current_timestamp(self):
        currTime = rospy.Time.now()
        currTime = 1.0*currTime.secs + 1.0*currTime.nsecs/pow(10,9)
        return(currTime)

    def processRemote(self, nbOfChannels, nbrOfSamplesByChannel, aTimeStamp, buffer ):
        
        # print(type(nbOfChannels),nbOfChannels)
        # print(type(nbrOfSamplesByChannel),nbrOfSamplesByChannel)
        # print(type(aTimeStamp),aTimeStamp)
        # print(type(buffer),len(buffer))
        
        self.int_buffer = np.fromstring(buffer, dtype=np.int16 )
        #print(type(self.int_buffer),len(self.int_buffer))
        if not self.audioRecordingStarted and not self.answeringPreviousQuestion:
            self.audioRecordingStarted=True
            
            self.outFile = open(self.RAW_FILE,"wb")
            self.startTime = aTimeStamp[0]


        
        if (not self.answeringPreviousQuestion and (aTimeStamp[0]-self.startTime < self.listeningTimeout)):

            
            self.int_buffer.tofile(self.outFile)
            
            

        else:

            if(not self.answeringPreviousQuestion):
                self.outFile.close()
                data, samplerate = sf.read(self.RAW_FILE, channels=4, samplerate=48000, subtype='PCM_16')
                #print(data,samplerate)
                sf.write(self.WAV_FILE, data, samplerate)
                
                ##change status of led to processing
                self.led_status_pub.publish('1')
                
                text_to_process = self.audio_recognition()
                print(text_to_process)
                if(text_to_process is not None):
                    newQuestion=rasa_question()
                    newQuestion.language='english'
                    newQuestion.question = text_to_process
                    self.answeringPreviousQuestion=True
                    self.question_pub_topic.publish(newQuestion)
                    ##change status of led to processing
                    self.led_status_pub.publish('2')
                
                else:
                    self.answeringPreviousQuestion=False
                    ##change status of led to listening
                    self.audioRecordingStarted=False
                    self.led_status_pub.publish('0')


    def audio_recognition(self):
        
        sound = AudioSegment.from_wav(self.WAV_FILE)
        sounds = sound.split_to_mono()
        print(len(sounds))
        sounds[0].export(self.absolute_file_path + "/audio_recordings/pepper1.wav", format="wav")
        sounds[1].export(self.absolute_file_path + "/audio_recordings/pepper2.wav", format="wav")
        sounds[2].export(self.absolute_file_path + "/audio_recordings/pepper3.wav", format="wav")
        sounds[3].export(self.absolute_file_path + "/audio_recordings/pepper4.wav", format="wav")
        max_length_of_statement=0
        final_text_obtained=None

        for i in range(0,4):
            file_name = self.absolute_file_path + "/audio_recordings/pepper"+str(i+1)+ ".wav"
            #file_name = "wav_output.wav"
            with ssr.AudioFile(file_name) as source:
                self.r.adjust_for_ambient_noise(source)
                audio_en = self.r.record(source)
            try:
                #text_obtained = r.recognize_google(audio_en,language = "en-IN")
                googleLang = None
                if(self.uselanguage == 'english'):
                    googleLang = "en-IN"
                else:
                    googleLang = "hi-IN"
                text_obtained = self.r.recognize_google(audio_en,language = googleLang)
                
                if(len(text_obtained)>max_length_of_statement):
                    final_text_obtained = text_obtained
                    max_length_of_statement = len(text_obtained)
                
                #return text_obtained
            except:

                print("Couldn't recognize speech")
                #return None 
        return final_text_obtained

    

    ## Callback function for answer obtained from RASA node
    def rasa_answer_callback(self,data):
        result = data.data
        print('[INFO]:  Text input obtained from audio conversion',result)
        try:
            if(self.uselanguage == 'english'):
                if 'pronounced ' in result:
                    split_res = result.split('pronounced ')
                    res2 = split_res[1].split(']')
                    result = split_res[0] + res2[1]
            elif (self.uselanguage == 'hindi'):
                
                prefix = 'इसे सुनेंरोकें'
                prefix = unicode(prefix.decode('utf-8'))
                
                if prefix in result:
                    split_result = result.split(prefix)
                    result = split_result[1]
                    
                else:
                    print('[INFO]:No glitch in result')
        except Exception as e: 
            print('[INFO]:Exception in finding glitch -->',e)
        
        
        if(self.uselanguage == 'hindi'):
            print("[INFO]: Converting text to Audio Using GTTS")

            try:
                text_to_speech_result_for_hind = gTTS(text=result, lang='hi', slow=False)
                text_to_speech_result_for_hind.save(self.absolute_file_path + "/audio_recordings/hindi_speech.mp3")
                text_to_speech_result_for_hind = AudioSegment.from_mp3(self.absolute_file_path + "/audio_recordings/hindi_speech.mp3")
                text_to_speech_result_for_hind = text_to_speech_result_for_hind + 6
                text_to_speech_result_for_hind.export(self.absolute_file_path + "/audio_recordings/hindi_speech.mp3", format='mp3')
                
                filePath =self.absolute_file_path + "/audio_recordings/hindi_speech.mp3"
                os_command = "sshpass -p nao scp -o StrictHostKeyChecking=no "+ filePath+" nao@" + self.pepper_ip + ":/home/nao/"
                os.system(os_command)
                self.audio_player.stopAll()
                #Loads a file and launchs the playing 5 seconds later
                print("[INFO]: Playing Hindi Audio from file")
                fileId = self.audio_player.loadFile("/home/nao/hindi_speech.mp3")
                self.audio_player.play(fileId)
            except Exception as e: 
                print('[INFO]: Error in converting audio to text-->',e)


        else:
            

            print(type(result))
            print("[INFO]: Passing English Audio Directly to Pepper")
            self.ats.say(str(result),self.bodyLanguageConfiguration)
        
        if(self.answeringPreviousQuestion):
            self.answeringPreviousQuestion=False
        self.audioRecordingStarted=False
        ##change status of led to listening
        self.led_status_pub.publish('0')
        
    
    # This function is no more used as this is from server client model
    # This has been replaced with answer_callback function which get data from the rasa_node over the rasa_ping topic

    # def rasa_answer_obtained_from_server(self,text):      
    #     #text = re.sub(r" ", "+", self.transcript)   
    #     lang = self.uselanguage
    #     URL = "http://10.10.1.109:4000/hindi-android-ai?text="+text+"&lang="+lang
    #     #print("---URL---", URL)

    #     # Get the response from the RASA server

    #      ##change status of led to processing
    #     self.led_status_pub.publish('2')

        
    #     response = requests.get(url=URL, timeout=13) # handle case for if RASA is not running
        
    #     if response.status_code == 200:
    #         #print(type((response.json())['response']))
    #         result = (response.json())['response']
    #         #result = unicode(result)
            
    #         print('[INFO]:  Text input obtained from audio conversion',result)
    #         try:
    #             if(self.uselanguage == 'english'):
    #                 if 'pronounced ' in result:
    #                     split_res = result.split('pronounced ')
    #                     res2 = split_res[1].split(']')
    #                     result = split_res[0] + res2[1]
    #             elif (self.uselanguage == 'hindi'):
                    
    #                 prefix = 'इसे सुनेंरोकें'
    #                 prefix = unicode(prefix.decode('utf-8'))
                    
    #                 if prefix in result:
    #                     split_result = result.split(prefix)
    #                     result = split_result[1]
                        
    #                 else:
    #                     print('[INFO]:No glitch in result')
    #         except:
    #             print('[INFO]:Exception in finding glitch')
            
            
    #         if(self.uselanguage == 'hindi'):
    #             print("[INFO]: Converting text to Audio Using GTTS")

    #             try:
    #                 text_to_speech_result_for_hind = gTTS(text=result, lang='hi', slow=False)
    #                 text_to_speech_result_for_hind.save(self.absolute_file_path + "/audio_recordings/hindi_speech.mp3")
    #                 text_to_speech_result_for_hind = AudioSegment.from_mp3(self.absolute_file_path + "/audio_recordings/hindi_speech.mp3")
    #                 text_to_speech_result_for_hind = text_to_speech_result_for_hind + 6
    #                 text_to_speech_result_for_hind.export(self.absolute_file_path + "/audio_recordings/hindi_speech.mp3", format='mp3')
                    
    #                 filePath =self.absolute_file_path + "/audio_recordings/hindi_speech.mp3"
    #                 os_command = "sshpass -p nao scp -o StrictHostKeyChecking=no "+ filePath+" nao@" + self.pepper_ip + ":/home/nao/"
    #                 os.system(os_command)
    #                 self.audio_player.stopAll()
    #                 #Loads a file and launchs the playing 5 seconds later
    #                 print("[INFO]: Playing Hindi Audio from file")
    #                 fileId = self.audio_player.loadFile("/home/nao/hindi_speech.mp3")
    #                 self.audio_player.play(fileId)
    #             except Exception as e: 
    #                 print('[INFO]: Error in converting audio to text-->',e)
                
    #         else:
                
    #             print("[INFO]: Passing English Audio Directly to Pepper")
    #             self.ats.say(str(result),self.bodyLanguageConfiguration)
            
         
    #     else:
    #         print('[WARNING]:No response obtained from Rasa Server')



    def resetNameArray(self):
        self.namesToAppend=[]
        self.lastRefreshTime = self.get_current_timestamp()


    def face_callback(self,data):
        

        if(data.data not in self.namesToAppend):
            self.namesToAppend.append(data.data)
            n = random.randint(0,2)
            toSpeak= self.greetingPrefix[n] + data.data
            self.tts.say(toSpeak)

def main():
    ## Main entry point


    parser = OptionParser()
    parser.add_option("-l","--lang",
        help="Pass Language of Operation: for hindi pass : hindi and for english pass : english",
        dest="lang")


    parser.add_option("-i","--ip",
        help="Pass ip of the robot",
        dest="pip")
    
    parser.add_option("-d","--pepperID",
        help="Pass id of the robot",
        dest="pepperID")

    parser.set_defaults(
        lang='english',
        pip = DEFAULT_PEPPER_IP,
        pepperID = '1')

    (opts, args_) = parser.parse_args()
    language   = opts.lang
    pip = opts.pip
    pepperID = opts.pepperID



    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    #The broker must stay alive until the program exists
    myBroker = ALBroker("myBroker",
    "0.0.0.0",   # listen to anyone
    0,           # find a free port and use it
    pip,         # parent broker IP
    9559)       # parent broker port


    # Warning: HumanGreeter must be a global variable
    # The name given to the constructor must be the name of the
    # variable\
    #sst.say_something(myBroker)
    

    rospy.init_node('conversation_node', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    global NewConversation
    NewConversation = ConversationModule("NewConversation",language,pip,pepperID)


    rospy.Subscriber("faces_topic_"+pepperID, String,NewConversation.face_callback)
    rospy.Subscriber("/rasa_answer_"+pepperID,String,NewConversation.rasa_answer_callback)
    

    try:
        while not rospy.is_shutdown():
            
            if(NewConversation.get_current_timestamp()- NewConversation.lastRefreshTime> NewConversation.refreshNamesTimeout):
                NewConversation.resetNameArray()

            rate.sleep()

    except KeyboardInterrupt:

        print ("Interrupted by user, shutting down")
        myBroker.shutdown()
        sys.exit(0)



if __name__ == "__main__":
    main()