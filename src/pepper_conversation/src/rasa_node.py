#!/usr/bin/env python3

import rospy
import rospkg
import os
import time
import asyncio
import re
import config as cf
import random
from optparse import OptionParser

#Google search requirements
import requests
import json
from serpapi import GoogleSearch

#Rasa libraries
from rasa.core.agent import Agent
from rasa.core.interpreter import RasaNLUInterpreter
from rasa.utils.endpoints import EndpointConfig    # required to launch the action server

# user defined python modules kept in same directory as this code
# -----------------------

##Import all ros message files
from std_msgs.msg import String
from pepper_conversation.msg import rasa_question

DEFAULT_PEPPER_IP='10.10.3.129'



class RASA_ANSWER_NODE:
    def __init__(self,opts):

        self.opts=opts
        #asyncio event handler loop
        self.loop = asyncio.get_event_loop()
        self.englishRasaAgent = Agent.load(self.get_ros_package_path()+ cf.rasa_paths["EnglishModelPath"] + cf.rasa_paths["EnglishModel"])
        self.hindiRasaAgent = Agent.load(self.get_ros_package_path()+ cf.rasa_paths["HindiModelPath"] + cf.rasa_paths["HindiModel"])
        self.action_endpoint = EndpointConfig(url=cf.rasa_paths["ActionEndPointURL"])
        self.pub1 = rospy.Publisher("/rasa_answer_"+self.opts.pepperID,String,queue_size=10)

    def get_ros_package_path(self):

        # get an instance of RosPack with the default search paths
        rospack = rospkg.RosPack()
        # return the file path for pepper conversation package
        return(rospack.get_path('pepper_conversation'))
    
    def checkKey(self,dict, key):
        if key in dict.keys():
            return True

        else:
            return False

    def searchInternet(self,question,language):
        try:
            if(language == 'english'):
                ln = 'en'
            elif(language == 'hindi'):
                ln = 'hi'
                
            search = GoogleSearch({"q":question, "google_domain": "google.com",
                                "gl":"in", "hl" : ln,"location": "Gujarat,India", 
                                "api_key": "6776ddfb5649758e93e57d85842748fea0c5e08fdd0976b71b308d02e9cff7a0"})
                    
            result = search.get_dict()
            answer = ""

            if self.checkKey(result,"answer_box"):
                ## check for answer/result/snippet and title keys in answer box

                
                if(self.checkKey(result["answer_box"],"answer")):
                    answer=result["answer_box"]["answer"]
                elif self.checkKey(result["answer_box"],"result"):
                    answer=result["answer_box"]["result"]
                elif self.checkKey(result["answer_box"],"snippet"):
                    answer=result["answer_box"]["snippet"]
                elif self.checkKey(result["answer_box"],"title"):
                    answer=result["answer_box"]["title"]           
                else:
                    answer=""

            if self.checkKey(result,"knowledge_graph") and answer=="":

                print('[INFO] :No answerbox')
                if self.checkKey(result["knowledge_graph"],"description"):
                    answer=result["knowledge_graph"]["description"]
                else:
                    answer=""

            if self.checkKey(result,"organic_results") and answer=="":

                print('[INFO] :No knowledgeGraph')
                if self.checkKey(result["organic_results"][0],"snippet"):
                    answer = result["organic_results"][0]["snippet"]
                else:
                    print('[INFO] :No Organic Results')
                    answer=""        
            
            return answer
            
        except:
            print('[Warning] :Exception')
            return ""

    def getAnswerCallback(self,data):
        rasaAnswer=None
        if data.language == "english" :
            rasaAnswer  = self.loop.run_until_complete(self.englishRasaAgent.handle_text(data.question))
            
        elif data.language == "hindi" :
            rasaAnswer  = self.loop.run_until_complete(self.hindiRasaAgent.handle_text(data.question))
        

        #[{'recipient_id': 'default', 'text': 'We have 3 washrooms on each floor in the Roboseum. You will find them towards your right next to the elevator.'}]
        #~Sorry could you speak more slowly please ?
        if rasaAnswer==None or rasaAnswer == []:
            ##Rasa has given default answer_box..go ahead and search in internet
            googleAnswer = self.searchInternet(data.question,data.language)
            
            if googleAnswer =="":
                defaultAnswerNumber = random.randint(0,2)
                ret = cf.default_answers[defaultAnswerNumber]
                self.pub1.publish(ret)
            else:
                self.pub1.publish(googleAnswer)

        else:
            returnAnswer = rasaAnswer[0]['text']
            if(returnAnswer[0]) =='~':
            ##Rasa has given default answer_box..go ahead and search in internet
                googleAnswer = self.searchInternet(data.question,data.language)
                
                if googleAnswer =="":
                    defaultAnswerNumber = random.randint(0,2)
                    ret = cf.default_answers[defaultAnswerNumber]
                    self.pub1.publish(ret)
                else:
                    self.pub1.publish(googleAnswer)
            else:
                self.pub1.publish(returnAnswer)

                    

    

if __name__=="__main__":


    parser = OptionParser()


    parser.add_option("-i","--ip",
        help="Pass ip of the robot",
        dest="pip")
    
    parser.add_option("-d","--pepperID",
        help="Pass id of the robot",
        dest="pepperID")

    parser.set_defaults(
        pip = DEFAULT_PEPPER_IP,
        pepperID='1')

    (opts, args_) = parser.parse_args()
    
    pip = opts.pip
    pepperID = opts.pepperID


    rasaInstance = RASA_ANSWER_NODE(opts)
    rospy.init_node('rasa_node', anonymous=True)
    rate = rospy.Rate(5) # 30hz

    rospy.Subscriber("/rasa_ping_"+pepperID, rasa_question, rasaInstance.getAnswerCallback)
    
    try:
        while not rospy.is_shutdown():
            # data1 = rasa_question()
            # data1.language = 'english'
            # data1.question = 'hello'
            # pub1.publish(data1)
            rate.sleep()
    except:
        print('Error running Node')