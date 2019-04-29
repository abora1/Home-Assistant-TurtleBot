#!/usr/bin/env python 
import argparse
import roslib
import rospy
import urllib
from geometry_msgs.msg import Twist

from pocketsphinx.pocketsphinx import *
from sphinxbase.sphinxbase import *
import pyaudio
from geometry_msgs.msg import Twist	
from sound_play.libsoundplay import SoundClient
import rospy 
import actionlib 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal 
import cv2,os
import numpy as np
from PIL import Image 
import pickle
 

def face():
	recognizer = cv2.face.LBPHFaceRecognizer_create() 
	recognizer.read('/home/patila/catkin_ws/src/PatilA/src/trainer/trainer.yml')
	cascadePath = "/home/patila/catkin_ws/src/PatilA/src/Classifiers/face.xml"
	faceCascade = cv2.CascadeClassifier(cascadePath);
	path = 'dataSet'
	url='http://149.125.37.190:8080/shot.jpg'
	#cam = cv2.VideoCapture(0)
	#font = cv2.cv.InitFont(cv2.cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1, 0, 1, 1) #Creates a font
	#font = cv2.FONT_HERSHEY_SIMPLEX
	#fontscale = 1
	#fontcolor = (255, 0, 0)
	soundhandle = SoundClient()

        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)
	
        stream.start_stream()

	while True:
		imgResp = urllib.urlopen(url)
		imgNp = np.array(bytearray(imgResp.read()),dtype=np.uint8)
		img = cv2.imdecode(imgNp,-1)
		cv2.imshow('IPWebcam',img)
		if cv2.waitKey(1) & 0xFF == ord('q'):
		   break
	 #   ret, im =cam.read()
		gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
		faces=faceCascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5, minSize=(100, 100), flags=cv2.CASCADE_SCALE_IMAGE)
		for(x,y,w,h) in faces:
		  nbr_predicted, conf = recognizer.predict(gray[y:y+h,x:x+w])
		  cv2.rectangle(img,(x-50,y-50),(x+w+50,y+h+50),(225,0,0),2)
		  if(conf<50):
			if(nbr_predicted==1):
		     		nbr_predicted='Akshay'
			elif(nbr_predicted==2):
		     		nbr_predicted='Aaditya'
				soundhandle.say('should i open the door?')
		else:
		     nbr_predicted='UNKNOWN'
		     soundhandle.say('Hi visitor')
		     
	       # cv2.cv.PutText(cv2.cv.fromarray(im),str(nbr_predicted)+"--"+str(conf), (x,y+h),font, 255) #Draw the text
		print nbr_predicted        
		cv2.imshow('im',img)
		cv2.waitKey(10)


def voice_control(): 

        vel_msg = Twist()
  
        publisher = rospy.Publisher('mobile_base/commands/velocity', Twist, queue_size=10)

        config = Decoder.default_config()

	model='/usr/share/pocketsphinx/model/hmm/en_US/hub4wsj_sc_8k'
        config.set_string('-hmm', model)

	lexicon=rospy.get_param('~lex')
        config.set_string('-dict', lexicon)

	kwlist=rospy.get_param('~kwl')
        config.set_string('-kws', kwlist)

	soundhandle = SoundClient()

        stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1,
                        rate=16000, input=True, frames_per_buffer=1024)
	
        stream.start_stream()

        decoder = Decoder(config)
        decoder.start_utt()

        while not rospy.is_shutdown():
            buf = stream.read(1024)
            if buf:
                decoder.process_raw(buf, False, False)
            else:
                break 
            action(decoder,vel_msg,publisher,soundhandle,goal)


def action(decoder,vel_msg, publisher,soundhandle,goal):
	
	global flag
	if decoder.hyp() != None:
	    
		print ([(seg.word, seg.prob, seg.start_frame, seg.end_frame)
                for seg in decoder.seg()])
				
		seg.word = seg.word.upper()
		decoder.end_utt()
		decoder.start_utt()

		if seg.word.find("AHEAD") > -1:
			flag =0
			goal(flag)
			vel_msg.linear.x = 0.25
                	vel_msg.angular.z = 0
			soundhandle.say('moving forward')
		
		elif	seg.word.find("STOP") > -1:
			flag =0	
			goal(flag)
			vel_msg.linear.x = 0
                	vel_msg.angular.z = 0
			soundhandle.say('OK')

		elif	seg.word.find("COME BACK") > -1:
			flag =0
			goal(flag)
			vel_msg.linear.x = -0.25
                	vel_msg.angular.z = 0
			soundhandle.say('going backwards')	

		elif	seg.word.find("LEFT") > -1 or seg.word.find("GO LEFT") > -1:
			flag =0
			goal(flag)
			vel_msg.linear.x = 0
                	vel_msg.angular.z = 0.2
			soundhandle.say('I am turning')
	
		elif	seg.word.find("RIGHT") > -1 or seg.word.find("GO RIGHT") > -1:
			flag =0
			goal(flag)
			vel_msg.linear.x = 0
                	vel_msg.angular.z = -0.2
			soundhandle.say('I am turning')
		
		elif	seg.word.find("POINT ONE") > -1:
			soundhandle.say('going to the corner')
			flag=1
		 	goal(flag)    	

		elif	seg.word.find("SLOW DOWN") > -1 or seg.word.find("REDUCE SPEED") > -1:
			flag=0
		 	goal(flag) 
			if(vel_msg.linear.x==0):
				vel_msg.linear.x=0			
			else:
				vel_msg.linear.x = vel_msg.linear.x-0.05
		        	vel_msg.angular.z = 0
				soundhandle.say('reducing speed')

		elif	seg.word.find("FASTER") > -1:
			flag=0
		 	goal(flag) 
			if(vel_msg.linear.x==0):
				vel_msg.linear.x=0			
			else:
				vel_msg.linear.x = vel_msg.linear.x+0.05
		        	vel_msg.angular.z = 0
				soundhandle.say('incerasing speed')

 	publisher.publish(vel_msg)


def goal(flag):
	
	if flag==1:
		client = actionlib.SimpleActionClient('move_base',MoveBaseAction) 
		client.wait_for_server()  
		goal = MoveBaseGoal() 
		goal.target_pose.header.frame_id = "map" 
		goal.target_pose.header.stamp = rospy.Time.now() 
		goal.target_pose.pose.position.x = 7.68
		goal.target_pose.pose.position.y = 3.48
		goal.target_pose.pose.orientation.w = 1.0 
		client.send_goal(goal)
		client.wait_for_result() 

	elif flag==0:
		client = actionlib.SimpleActionClient('move_base',MoveBaseAction) 
		client.wait_for_server()  
		goal = MoveBaseGoal() 
		goal.target_pose.header.frame_id = "map" 
		goal.target_pose.header.stamp = rospy.Time.now() 
		client.send_goal(goal)

if __name__ == '__main__': 
           try: 
         	global flag
            	rospy.init_node('voice', anonymous=True)  
            	#rate=rospy.Rate(2) 
                #voice_control()
		face() 
           except rospy.ROSInterruptException: pass
