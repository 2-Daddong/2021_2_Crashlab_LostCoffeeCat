#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import sys
import cv2

from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#===========================================================================


class tracker:
	
	def __init__(self):

		#set center_x & distance pub node
		self.x_pub = rospy.Publisher('tracking', Int64, queue_size=1)
		self.depth_pub = rospy.Publisher('distance', Int64, queue_size=1)
		self.rate = rospy.Rate(10)

		#set subscribe
		self.bbox_sub = rospy.Subscriber('/tracked_boxes', BoundingBoxes, self.boxcallback)
		self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depthcallback)
		#self.gui_sub = rospy.Subscriber('/GUI_end', bool, self.guicallback)
		#self.cycle_sub = rospy.Subscriber('/cycle_reset', bool, self.cyclecallback)
		#self.rfid_sub = rospy.Subscriber('/RFID', bool, self.RFIDcallback)
		self.RFID_check = False #remove after

		self.tracking_id = 0
		self.tracking_mode = 0
		self.count = 0
		self.depth = np.empty((480,640))
	
#======================================================================================================

	def depthcallback(self, Image):
		self.bridge = CvBridge()

		depth_image = self.bridge.imgmsg_to_cv2(Image, desired_encoding="passthrough")
		self.depth = np.array(depth_image, dtype=np.float32)


	def RFIDcallback(self, data1):
		self.RFID_check = data1.data


	def guicallback(self, data2):
		self.gui_check = data2.data


	def cyclecallback(self, data3):
		self.cycle_check = data3.data

#======================================================================================================

	def boxcallback(self, detectbox):

		#tracking_mode==0: roaming
		if self.tracking_mode == 0:
			
			depths=[]
			for box in range(len(detectbox.bounding_boxes)):
				bbox = detectbox.bounding_boxes[box]
				center_x = round((bbox.xmin + bbox.xmax)/2)
				center_y = round((bbox.ymin + bbox.ymax)/2)
				distance = round(self.depth[int(center_y), int(center_x)]/10)
				depths.append([distance, bbox.id])

			#choose the closest person if detected is more than one
			if len(depths) > 0:
				#sort array in ascending - 1)distance 2)id
				depths.sort(key=lambda x: (x[0], x[1]))
	
				#decide the closest person for tracking target (depth boundary = 160cm)
				if depths[0][0] < 160:
					self.tracking_id = depths[0][1]
		
					self.tracking_mode = 1
					rospy.loginfo('target person')
					rospy.loginfo(self.tracking_id)
		
		#tracking_mode==1: check target
		elif self.tracking_mode == 1:
			
			self.tracking_mode = 2

		#tracking_mode==2: tracksing
		elif self.tracking_mode == 2:
			
			if self.RFID_check == False:
				
         			for box in range(len(detectbox.bounding_boxes)):
            
					bbox = detectbox.bounding_boxes[box]
					if self.tracking_id == bbox.id:
						center_x = round((bbox.xmin + bbox.xmax)/2)
						center_y = round((bbox.ymin + bbox.ymax)/2)
						distance = round(self.depth[int(center_y), int(center_x)]/10)

						if distance <= 400:
							self.x_pub.publish(center_x)
							self.depth_pub.publish(distance)
							rospy.loginfo('tracking target person')
							rospy.loginfo('x: %d' % center_x)
							rospy.loginfo('depth: %d' % distance)
							#self.rate.sleep()
							break
        
					else:
						if bbox == len(detectbox.bounding_boxes):
							self.count += 1
							if self.count > 3:
								self.count=0
								self.tracking_mode = 0:
							else:
								pass
          
			else:
				self.tracking_mode = 3
				rospy.loginfo('end')

		#tracking_mode==3: waiting for end of cat's motion
		elif self.tracking_mode == 3:
			
			if self.gui_check == False:
				pass
			else:
				self.tracking_mode = 4
		
		#tracking_mode==4: return to start position
		else:
	
			if self.cycle_check == False:

				#get obstacle distance array about division pixel
				Image_width = self.depth.shape[1]
				Image_height = self.depth.shape[0]
				obstacle_depth = []

				for x in range(4):
 					for y in range(3):
						self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depthcallback)						
						self.distance = round(self.depth[Image_height/3/2+(Image_height/3)*y, Image_width/4/2+(Image_width/4)*x]/10)
						obstacle_depth.append(self.distance)
			
				obstacle_depth.sort()
				obstacle_distance = obstacle_depth[0]
				self.depth_pub.publish(obstacle_distance)
				
				self.rate.sleep()
				

			else:
				self.tracking_mode == 0 


#=========================================================================================

if __name__ == '__main__':
	
	rospy.init_node('start', anonymous=True)
	
	tracker()

	try:
		rospy.spin()
	except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
		sys.exit(0)
