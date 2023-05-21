#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import sys
import time
import vision_definitions
import dlib
import face_recognition
from PIL import Image
import cv2
import numpy as np

import rospy
import rospkg
import sys

# import the necessary packages
from imutils.video import VideoStream
import imutils
import pickle

from naoqi import ALProxy
from naoqi import ALBroker
from naoqi import ALModule
from optparse import OptionParser
import argparse
from std_msgs.msg import String




DEFAULT_PEPPER_IP ='10.10.3.129'

print("[INFO]: USING DLIB WITH CUDA ENABLED->",dlib.DLIB_USE_CUDA)


class face_recog():

    def __init__(self,opts):

        self.opts=opts
        print(self.opts.detection_method)
        print(self.opts.encodings)

        self.face_encodings_file_path = self.get_ros_package_path() + self.opts.encodings
        # load the known faces and embeddings
        print("[INFO] loading encodings...")
        self.data = pickle.loads(open(self.face_encodings_file_path, "rb").read())
        # initialize the video stream and pointer to output video file, then
        # allow the camera sensor to warm up
        print("[INFO] starting video stream service...")
        
        self.video_service = ALProxy("ALVideoDevice",self.opts.pip,self.opts.pport)

        self.resolution = 1    # VGA
        self.colorSpace = 11   # RGB

        # resolution = vision_definitions.kQQVGA
        # colorSpace = vision_definitions.kYUVColorSpace
        self.fps = 30

        print("[INFO] subscribing to video stream service...")
        self.videoClient = self.video_service.subscribe("python_GVM", self.resolution, self.colorSpace, self.fps)

        #####All Publisher and Subscriber initialization

        self.face_recog_pub = rospy.Publisher('faces_topic_'+self.opts.pepperID, String, queue_size=10)

        

    def get_ros_package_path(self):

        rp = rospkg.RosPack()
        return(rp.get_path('pepper_conversation'))

    def image_grabber(self):
        

        # while True:
        # grab the frame from the pepper 
        imageFromPepper = self.video_service.getImageRemote(self.videoClient)
        # Get the image size and pixel array.
        if(imageFromPepper is not None):
            imageWidth = imageFromPepper[0]
            imageHeight = imageFromPepper[1]
            array = imageFromPepper[6]
            #print(imageFromPepper)
            image_string = str(bytearray(array))

            # # Create a PIL Image from our pixel array.
            im = Image.frombytes("RGB", (imageWidth, imageHeight), image_string)
            frame = np.array(im)

                # convert the input frame from BGR to RGB then resize it to have
            # a width of 750px (to speedup processing)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb1 = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb = imutils.resize(frame, width=750)
            r = frame.shape[1] / float(rgb.shape[1])
            # detect the (x, y)-coordinates of the bounding boxes
            # corresponding to each face in the input frame, then compute
            # the facial embeddings for each face
            boxes = face_recognition.face_locations(rgb,
                model=self.opts.detection_method)
            encodings = face_recognition.face_encodings(rgb, boxes)
            names = []

            # loop over the facial embeddings
            for encoding in encodings:
                # attempt to match each face in the input image to our known
                # encodings
                matches = face_recognition.compare_faces(self.data["encodings"],
                    encoding,tolerance=0.45)
                name = "Unknown"
                # check to see if we have found a match
                if True in matches:
                    # find the indexes of all matched faces then initialize a
                    # dictionary to count the total number of times each face
                    # was matched
                    matchedIdxs = [i for (i, b) in enumerate(matches) if b]
                    counts = {}
                    # loop over the matched indexes and maintain a count for
                    # each recognized face face
                    for i in matchedIdxs:
                        name = self.data["names"][i]
                        counts[name] = counts.get(name, 0) + 1
                    # determine the recognized face with the largest number
                    # of votes (note: in the event of an unlikely tie Python
                    # will select first entry in the dictionary)
                    name = max(counts, key=counts.get)
                
                # update the list of names
                names.append(name)
            
            for eachName in names:
                if(eachName is not 'Unknown'):
                    self.face_recog_pub.publish(eachName)

                # loop over the recognized faces
            for ((top, right, bottom, left), name) in zip(boxes, names):
                # rescale the face coordinates
                top = int(top * r)
                right = int(right * r)
                bottom = int(bottom * r)
                left = int(left * r)
                # draw the predicted face name on the image
                cv2.rectangle(rgb1, (left, top), (right, bottom),
                    (0, 255, 0), 2)
                y = top - 15 if top - 15 > 15 else top + 15
                cv2.putText(rgb1, name, (left, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.75, (0, 255, 0), 2)

            # check to see if we are supposed to display the output frame to
            # the screen
            if self.opts.display > 0:
                cv2.imshow("Frame", rgb1)
                key = cv2.waitKey(1) & 0xFF
                # if the `q` key was pressed, break from the loop
                if key == ord("q"):
                    self.video_service.unsubscribe(self.videoClient)
                    # do a bit of cleanup
                    cv2.destroyAllWindows()





def main():
    ## Main entry point

    time.sleep(20)
    parser = OptionParser()
    parser.add_option("-i","--pip",
        help="Parent broker port. The IP address or your robot",
        dest="pip")
    parser.add_option("-p","--pport",
        help="Parent broker port. The port NAOqi is listening to",
        dest="pport",
        type="int")

    parser.add_option("-y", "--display", 
        type=int, 
	    help="whether or not to display output frame to screen")
    
    parser.add_option("-e","--encodings",
        help="path to serialized db of facial encodings",
        dest="encodings")

    parser.add_option("-m", "--detection_method", 
        type=str, 	
        help="face detection model to use: either `hog` or `cnn`")

    parser.add_option("-d","--pepperID",
        help="Pass id of the robot",
        dest="pepperID")

    parser.set_defaults(
        pip=DEFAULT_PEPPER_IP,
        pport=9559,
        encodings='/face_recognition_database/face_encodings.pickle',
        display = 1,
        detection_method = 'cnn',
        pepperID='1')

    (opts, args_) = parser.parse_args()

    # We need this broker to be able to construct
    # NAOqi modules and subscribe to other modules
    #The broker must stay alive until the program exists
    # No need for a broker in this code
    #     # myBroker = ALBroker("myBroker",
    #     "0.0.0.0",   # listen to anyone
    #     0,           # find a free port and use it
    #     pip,         # parent broker IP
    #     9559)       # parent broker port


    # Warning: HumanGreeter must be a global variable
    # The name given to the constructor must be the name of the
    # variable\

   
    newFaceRecog = face_recog(opts)

    rospy.init_node('face_recognition_node', anonymous=True)
    rate = rospy.Rate(40) # 10hz
    

    try:
        while not rospy.is_shutdown():
            
            newFaceRecog.image_grabber()         
            rate.sleep()

    except KeyboardInterrupt:
        
        print ("Interrupted by user, shutting down")
        cv2.destroyAllWindows()
        sys.exit(0)



if __name__ == "__main__":
    main()