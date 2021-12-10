#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import sys
import cv2

from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import Int32, Int64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


#===================================================================================


class tracker:
	
	def __init__(self):

		#set center_x & distance pub node
		self.x_pub = rospy.Publisher('tracking', Int64, queue_size=1)
		self.depth_pub = rospy.Publisher('distance', Int64, queue_size=1)
		self.rate = rospy.Rate(10)

		#set subscribe
		self.bbox_sub = rospy.Subscriber('/tracked_boxes', BoundingBoxes, self.boxcallback)
		self.depth_sub = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depthcallback)
		#self.gui_sub = rospy.Subscriber('/gui_mode', Float32, self.guicallback)
		#gui_mode = 2 or 5: tracking, 9: return start point (after arrived), 99: retrun start point (before start)
		#self.cycle_sub = rospy.Subscriber('/cycle', Int32, self.cyclecallback)
		#cycle = 1: 1 cycle end, 0: 1 cycle continue 
		#self.rfid_sub = rospy.Subscriber('/finish', Int32, self.RFIDcallback)
		#RFID = true: arrvied at destination, false: not arrived

		self.RFID_check = False #remove after

		self.tracking_id = 0
		self.tracking_mode = 0
		self.person_check = False
		self.distance1 = 0
		self.count = 0
		self.depth = np.empty((480,640))
	
#======================================================================================

	def depthcallback(self, Image):
		self.bridge = CvBridge()

		depth_image = self.bridge.imgmsg_to_cv2(Image, desired_encoding="passthrough")
		self.depth = np.array(depth_image, dtype=np.float32)


	def RFIDcallback(self, data1):
		self.RFID_check = data1.data


	def guicallback(self, data2):
		self.gui_check = data2.data


	#def cyclecallback(self, data3):
		self.cycle_check = data3.data

#======================================================================================================

	def boxcallback(self, detectbox):
		
		self.person_check = True

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
				if depths[0][0] < 180:
					self.tracking_id = depths[0][1]
					self.distance1 = depths[0][0]
					self.tracking_mode = 1
					rospy.loginfo(self.tracking_id)
		
		#tracking_mode==1: check target
		elif self.tracking_mode == 1:
			
			self.tracking_mode = 2

		#tracking_mode==2: tracking
		elif self.tracking_mode == 2:

			if self.RFID_check == False:
				if gui_check == 1 or gui_check == 5:
          					
					for box in range(len(detectbox.bounding_boxes)):
            
						bbox = detectbox.bounding_boxes[box]
						if self.tracking_id == bbox.id:
							center_x = round((bbox.xmin + bbox.xmax)/2)
							center_y = round((bbox.ymin + bbox.ymax)/2)
							distance2 = self.distance1
							distance = round(self.depth[int(center_y), int(center_x)]/10)
							self.distance1 = distance

							if abs(distance2 - self.distance1) <= 120 and self.distance1 != 0:
								self.x_pub.publish(center_x)
								self.depth_pub.publish(distance)
								rospy.loginfo('tracking target person')
								rospy.loginfo('x: %d' % center_x)
								rospy.loginfo('depth: %d' % distance)
								break
							else:
								rospy.loginfo('assume outlier value!')
        
						else:
							self.count += 1 

							if self.count > 3:
								self.tracking_mode = 0
				    				self.count = 0
								rospy.loginfo('return mode 0')
							else:
								rospy.loginfo('watiting target person')

				elif gui_check == 99:
					tracking_mode = 4
					rospy.loginfo('refuse')

				else:
					pass


			else:
				self.tracking_mode = 3
				rospy.loginfo('Arrived!')

		#tracking_mode==3: waiting for end of cat's motion
		elif self.tracking_mode == 3:
			
			if self.gui_check != 9:
				pass
			else:
				self.tracking_mode = 4
				rospy.loginfo('return mode 4')
		
		#tracking_mode==4: return to start position
		else:
	
			if self.cycle_check == 0:

				#get obstacle distance array about division pixel
				Image_width = self.depth.shape[1]
				Image_height = self.depth.shape[0]
				obstacle_depth = []

				for x in range(4):
 					for y in range(3):
					
						self.distance = round(self.depth[Image_height/3/2+(Image_height/3)*y, Image_width/4/2+(Image_width/4)*x]/10)
						obstacle_depth.append(self.distance)
			
				obstacle_depth.sort()
				obstacle_distance = obstacle_depth[0]
				self.depth_pub.publish(obstacle_distance)
				

			else:
				self.tracking_mode == 0 
				rospy.loginfo('restart scenario')

	#case that didn't sub /tracked_boxes
	def no_sub_bbox(self):

		rospy.loginfo(self.person_check)

		if self.person_check == True:
			pass
		else:
			trash_x = -1
			trash_dist = 0
			self.x_pub.publish(trash_x)
			self.depth_pub.publish(trash_dist)
			rospy.loginfo('No sub bbox: pub trash value')
		
		self.person_check = False

#=========================================================================================

if __name__ == '__main__':
	
	rospy.init_node('vision', anonymous=True)
	
	tracker()

	try:
		rospin()

	except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
		sys.exit(0)
