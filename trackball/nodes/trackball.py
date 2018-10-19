#!/usr/bin/env python
import rospy
import cv2
import sys
import math
import argparse
import imutils
import time
import numpy as np
import thread

#from Tkinter import *
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
from collections import deque
from contour_color.shapedetector import ShapeDetector

(major_ver, minor_ver, subminor_ver) = (cv2.__version__).split('.')
tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN']
tracker_type = tracker_types[2]
         
class TrackBall(object):
    def __init__(self, node_name):	      
        self.node_name = node_name
        rospy.init_node(node_name)
        rospy.loginfo("Starting node " + str(node_name))
        rospy.on_shutdown(self.cleanup)
        self.show_text = rospy.get_param("~show_text", True)
        self.ROI = RegionOfInterest()
        self.roi_pub = rospy.Publisher("/roi", RegionOfInterest, queue_size=1)
        self.frame_size = None
        self.frame_width = None
        self.frame_height = None
        self.cps = 0 
        self.cps_values = list()
        self.cps_n_values = 20      
        self.bridge = CvBridge()
        self.track_box = None
        self.has_tracked = False
        self.redLower = None
        self.redUpper = None
        self.blueLower = None
        self.blueUpper = None
        self.track_color = None
        self.track_count = 0
        self.track_count_thread = 90  ###to publish roi, thread of track count
        self.sd = ShapeDetector()
        self.red_h_min = rospy.get_param("~red_h_min", 0)
        self.red_h_max = rospy.get_param("~red_h_max", 10)	
        self.blue_h_min = rospy.get_param("~blue_h_min", 100)
        self.blue_h_max = rospy.get_param("~blue_h_max", 124)
        self.s_min = rospy.get_param("~s_min", 43)	
        self.s_max = rospy.get_param("~s_max", 255)	
        self.v_min = rospy.get_param("~v_min", 46)	
        self.v_max = rospy.get_param("~v_max", 255)
        self.red_area_percent = rospy.get_param("~red_area_percent", 75)
        self.blue_area_percent = rospy.get_param("~blue_area_percent", 85)	
        self.image_pub = rospy.Publisher("camera/image",Image,queue_size=10)
        self.image_sub = rospy.Subscriber("camera/bgr/image_raw", Image, self.image_callback, queue_size=1)
        if int(minor_ver) < 3:
            self.tracker = cv2.Tracker_create(tracker_type)
        else:
       	    if tracker_type == 'BOOSTING':
            	self.tracker = cv2.TrackerBoosting_create()
            if tracker_type == 'MIL':
            	self.tracker = cv2.TrackerMIL_create()
            if tracker_type == 'KCF':
            	self.tracker = cv2.TrackerKCF_create()
            if tracker_type == 'TLD':
            	self.tracker = cv2.TrackerTLD_create()
            if tracker_type == 'MEDIANFLOW':
            	self.tracker = cv2.TrackerMedianFlow_create()
            if tracker_type == 'GOTURN':
            	self.tracker = cv2.TrackerGOTURN_create()
            if tracker_type == 'MOSSE':
            	self.tracker = cv2.TrackerMOSSE_create()
            if tracker_type == "CSRT":
            	self.tracker = cv2.TrackerCSRT_create()
    # These are the callbacks for the slider controls
    def set_red_h_min(self, pos):
        self.red_h_min = pos
    def set_red_h_max(self, pos):
        self.red_h_max = pos
    def set_blue_h_min(self, pos):
        self.blue_h_min = pos
    def set_blue_h_max(self, pos):
        self.blue_h_max = pos
    def set_s_min(self, pos):
        self.s_min = pos
    def set_s_max(self, pos):
        self.s_max = pos   
    def set_v_min(self, pos):
        self.v_min = pos       
    def set_v_max(self, pos):
       self.v_max = pos
    def set_red_area_percent(self, pos):
        self.red_area_percent = pos
    def set_blue_area_percent(self, pos):
        self.blue_area_percent = pos
    
    def image_callback(self, data):
        timer = cv2.getTickCount()

        cv2.namedWindow("Parameters_1")#CV_WINDOW_NORMAL,cv2.WINDOW_AUTOSIZE   
        cv2.createTrackbar("s_min", "Parameters_1", self.s_min, 255, self.set_s_min)
        cv2.createTrackbar("s_max", "Parameters_1", self.s_max, 255, self.set_s_max)
        cv2.createTrackbar("v_min", "Parameters_1", self.v_min, 255, self.set_v_min)
        cv2.createTrackbar("v_max", "Parameters_1", self.v_max, 255, self.set_v_max)
        
        # Convert the ROS image to OpenCV format using a cv_bridge helper function
        frame = self.convert_image(data)	    
        if self.frame_width is None:
            self.frame_size = (frame.shape[1], frame.shape[0])
            self.frame_width, self.frame_height = self.frame_size
        frame = cv2.GaussianBlur(frame, (5, 5), 0)
        redmask_image = self.image_to_redmask(frame)
        bluemask_image = self.image_to_bluemask(frame)
        cv2.namedWindow("red_mask")
        cv2.createTrackbar("red_h_min", "red_mask", self.red_h_min, 255, self.set_red_h_min)
        cv2.createTrackbar("red_h_max", "red_mask", self.red_h_max, 255, self.set_red_h_max)
        cv2.createTrackbar("red_area_percent", "red_mask", self.red_area_percent, 100,self.set_red_area_percent)
        cv2.namedWindow("blue_mask")
        cv2.createTrackbar("blue_h_min", "blue_mask", self.blue_h_min, 255, self.set_blue_h_min)
        cv2.createTrackbar("blue_h_max", "blue_mask", self.blue_h_max, 255, self.set_blue_h_max) 
        cv2.createTrackbar("blue_area_percent", "blue_mask", self.blue_area_percent, 100,self.set_blue_area_percent) 
        cv2.imshow("red_mask",redmask_image)
        cv2.imshow("blue_mask",bluemask_image)
        cv2.waitKey(1)

        if self.has_tracked == True:
            if self.track_color == (0,0,255): 
            #now track is red,try to find a bigger one
                color = (0,0,255)
                bigger_red_find = self.find_bigger_red(redmask_image,color,frame,timer)
                if bigger_red_find == True:
                    rospy.loginfo("find a bigger red to track")
                else:
                    result = self.track_ball(frame)
                    if result == True:
                        self.track_count += 1
                        if self.track_count >  self.track_count_thread : 
                        #track so many times than publish red roi
                            x,y,w,h = self.track_box
                            roi_x = x + 0.5 * w
                            roi_y = y + 0.5 * h
                            roi_width = w
                            roi_height = h
                            self.publish_track_roi()   
                            rospy.loginfo("track red count:%d",self.track_count)
                            rospy.loginfo("publish red roi:(%d,%d,%d,%d)",roi_x,roi_y,roi_width,roi_height)              
                        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
                        self.publish_frame1(frame, fps)              
                    else:
                        rospy.loginfo("track red failed")
                        self.find_in_order(redmask_image,bluemask_image,frame,timer) 
            else:  
            #now track is blue,try to find red 
                color = (0,0,255)
                find_red = self.find_red_before_track(redmask_image,color,frame,timer)
                if find_red == True: 
                    rospy.loginfo("change color to track")
                else:
                    result = self.track_ball(frame)
                    if result == True:
                        self.track_count += 1
                        if self.track_count >  self.track_count_thread : 
                        #track so many times than publish blue roi
                            x,y,w,h = self.track_box
                            roi_x = x + 0.5 * w
                            roi_y = y + 0.5 * h
                            roi_width = w
                            roi_height = h
                            rospy.loginfo("publish blue roi:(%d,%d,%d,%d)",roi_x,roi_y,roi_width,roi_height)  
                            rospy.loginfo("track blue count:%d",self.track_count)
                            self.publish_track_roi()               
                        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer);
                        self.publish_frame1(frame, fps)              
                    else:
                        rospy.loginfo("track  blue failed")
                        self.find_in_order(redmask_image,bluemask_image,frame,timer)             
        else:
            rospy.loginfo("no tracked before")
            self.find_in_order(redmask_image,bluemask_image,frame,timer)          
        
    def find_in_order(self,red_mask,blue_mask,frame,timer):
        color = (0,0,255)
        find_red = self.find_ball(red_mask,color,frame,timer)
        if find_red == False:
            rospy.loginfo("find_in_order:no red ball")
            color = (255,0,0)
            find_blue = self.find_ball(blue_mask,color,frame,timer)
            if find_blue == True:
                rospy.loginfo("find_in_order:find blue ball")
            else:
                rospy.loginfo("find_in_order:no blue ball")
        else:
            rospy.loginfo("find_in_order:find red ball") 
 
    def find_ball(self,image,color,frame,timer):
        cnts = cv2.findContours(image,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        flag = False
        contours = []
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            for c in cnts:
                shape = self.sd.detect(c)
                ((cx, cy), cradius) = cv2.minEnclosingCircle(c)
                area_per = cv2.contourArea(c) / (math.pi * math.pow(cradius,2))
                #list conditions all here(cv2.HoughCircles)
                area_percent = 75
                if color == (0,0,255):
                    area_percent = self.red_area_percent
                elif color == (255,0,0):
                    area_percent = self.blue_area_percent
                else:
                    pass  #TODO
                if shape == "circle" and area_per * 100 >= area_percent:
                    flag = True
                    contours.append(c)
            if flag == True:
                rospy.loginfo("find_in_order: num_contours: %d", len(contours));
                cc = max(contours,key=cv2.contourArea)
                #((self.x, self.y), self.radius) = cv2.minEnclosingCircle(cc)
                self.track_box = cv2.boundingRect(cc) ###
                self.has_tracked = True
                self.track_color = color
                try:
                    self.tracker = cv2.TrackerKCF_create()
                    init_ok = self.tracker.init(frame,self.track_box)
                    self.track_count = 0
                except:
                     rospy.loginfo("find_in_order: tracker init failed")
                rospy.loginfo("find_in_order: find a new box to track")
                return True
            else:
                rospy.loginfo("find_in_order: no circle,wait to detect")
                self.has_tracked = False
                self.track_box = None
                self.track_color = None
                self.publish_roi()
                self.publish_frame(frame)
                return False
        else:
            rospy.loginfo("find_in_order: no contours,wait to detect")
            self.has_tracked = False
            self.track_box = None
            self.track_color = None
            self.publish_roi()
            self.publish_frame(frame)
            return False
        cnts = None
        contours = []
    
    def find_red_before_track(self,image,color,frame,timer):
        cnts = cv2.findContours(image,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        flag = False
        contours = []
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            for c in cnts:
                shape = self.sd.detect(c)
                ((cx, cy), cradius) = cv2.minEnclosingCircle(c)
                area_per = cv2.contourArea(c) / (math.pi * math.pow(cradius,2))
                #list conditions all here(cv2.HoughCircles)
                area_percent = self.red_area_percent
                if shape == "circle" and area_per * 100 >= area_percent:
                    flag = True
                    contours.append(c)
            if flag == True:
                rospy.loginfo("find_red_before_track: num_contours: %d", len(contours));
                cc = max(contours,key=cv2.contourArea)
                self.track_box = cv2.boundingRect(cc) ###
                self.has_tracked = True
                self.track_color = color
                try:
                    self.tracker = cv2.TrackerKCF_create()
                    init_ok = self.tracker.init(frame,self.track_box)
                    self.track_count = 0
                    return True
                except:
                     rospy.loginfo("find_red_before_track: tracker init failed")
                     return False
            else:
                return False
        else:
            return False
        cnts = None
        contours = []

    def find_bigger_red(self,image,color,frame,timer):
        cnts = cv2.findContours(image,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if imutils.is_cv2() else cnts[1]
        flag = False
        contours = []
        if len(cnts) > 0:
            for c in cnts:
                shape = self.sd.detect(c)
                ((cx, cy), cradius) = cv2.minEnclosingCircle(c)
                area_per = cv2.contourArea(c) / (math.pi * math.pow(cradius,2))
                area_percent = self.red_area_percent - 5
                if shape == "circle" and area_per * 100 >= area_percent:
                    flag = True
                    contours.append(c)
            if flag == True:
                cc = max(contours,key=cv2.contourArea)
                track_box = cv2.boundingRect(cc)
                x1,y1,w1,h1 = track_box
                x,y,w,h = self.track_box
                if w1 > w + 2 and h1 > h + 2 and abs(x1-x) > 5 and abs(y1-y) > 5:
                    self.track_box = cv2.boundingRect(cc) 
                    self.has_tracked = True
                    self.track_color = color
                    try:
                        self.tracker = cv2.TrackerKCF_create()
                        init_ok = self.tracker.init(frame,self.track_box)
                        self.track_count = 0
                    except:
                        rospy.loginfo("find_bigger_red: tracker init failed")
                    return True
                else:
                    return False
            else:
                return False
        else:
            return False
        cnts = None
        contours = []

    def track_ball(self,frame):  #KCF tracker
        try:
            timer = cv2.getTickCount()
            ok,bbox = self.tracker.update(frame)
            track_fps = int(cv2.getTickFrequency() / (cv2.getTickCount() - timer)); 
            cv2.putText(frame, tracker_type + " Tracker", (300,50), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),1);
            if ok:  
                self.has_tracked = True
                self.track_box = bbox
                return True
            else:
                #self.tracker.release()
                self.has_tracked = False
                self.track_box = None
                self.track_color = None
                return False
        except:
            rospy.loginfo("tracker update failed") 
            self.has_tracked = False
            self.track_box = None
            self.track_color = None
            return False

    def publish_track_roi(self):
        try:
            roi = RegionOfInterest()
            x,y,w,h = self.track_box
            roi.x_offset = x + 0.5 * w
            roi.y_offset = y + 0.5 * h
            roi.width = w
            roi.height = h
            self.roi_pub.publish(roi)
        except:
            rospy.loginfo("Publishing ROI failed")

    def publish_roi(self):
        try:
            roi = RegionOfInterest()
            roi.x_offset = 0
            roi.y_offset = 0
            roi.width = 0
            roi.height = 0
            self.roi_pub.publish(roi)
        except:
            rospy.loginfo("Publishing ROI failed")

    def publish_frame(self,frame):
        cv2.putText(frame, "No tracking object", (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1,(0, 0, 255))
        frame2 = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(frame2)

    def publish_frame1(self,frame,fps):
        x,y,w,h = self.track_box
        color = self.track_color
        cv2.rectangle(frame,(int(x),int(y)),(int(x+w),int(y+h)),color,2)
        # Compute the time for this loop and estimate CPS as a running average
        self.cps_values.append(fps)
        if len(self.cps_values) > self.cps_n_values:
            self.cps_values.pop(0)
        self.cps = int(sum(self.cps_values) / len(self.cps_values))
        # Display CPS and image resolution if asked to
        if self.show_text:
            font_face = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            if self.frame_size[0] >= 640:
                vstart = 25
                voffset = int(50 + self.frame_size[1] / 120.)
            elif self.frame_size[0] == 320:
                vstart = 15
                voffset = int(35 + self.frame_size[1] / 120.)
            else:
                vstart = 10
                voffset = int(20 + self.frame_size[1] / 120.)
            cv2.putText(frame, "CPS: " + str(self.cps), (10, vstart), font_face, font_scale,(0, 0, 255))
            cv2.putText(frame, "RES: " + str(self.frame_size[0]) + "X" + str(self.frame_size[1]), (10, voffset), font_face, font_scale, (0, 0, 255))
        frame2 = self.bridge.cv2_to_imgmsg(frame, "bgr8")
        self.image_pub.publish(frame2)

    def convert_image(self,image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            return np.array(cv_image, dtype=np.uint8)
        except CvBridgeError(e):
            print(e)
    
    def image_to_redmask(self, image):
        frame = imutils.resize(image, width=640)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.redLower = (self.red_h_min, self.s_min, self.v_min)
        self.redUpper = (self.red_h_max, self.s_max, self.v_max)
        mask_red = cv2.inRange(hsv, self.redLower, self.redUpper)
        mask_red = cv2.erode(mask_red, None, iterations=2)
        mask_red = cv2.dilate(mask_red, None, iterations=2)  
        return mask_red
    
    def image_to_bluemask(self, image):
        frame = imutils.resize(image, width=640)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        self.blueLower = (self.blue_h_min, self.s_min, self.v_min)
        self.blueUpper = (self.blue_h_max, self.s_max, self.v_max)
        mask_blue = cv2.inRange(hsv, self.blueLower, self.blueUpper)
        mask_blue = cv2.erode(mask_blue, None, iterations=2)
        mask_blue = cv2.dilate(mask_blue, None, iterations=2)
        return mask_blue

    def cleanup(self):
        print("Shutting down vision node.")
        cv2.destroyAllWindows()       

def main(args):
    try:
        node_name = "trackball"
        TrackBall(node_name)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down trackball node.")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
