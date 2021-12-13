#!/usr/bin/env python3
#-*- coding:utf-8 -*-

import rospy
import numpy as np
import sys
import cv2

from darknet_ros_msgs.msg import BoundingBoxes
from darknet_ros_msgs.msg import BoundingBox
from std_msgs.msg import Int64, Int32
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
		self.gui_sub = rospy.Subscriber('/gui_mode', Int32, self.guicallback)
		#gui_mode:1 or 5 => tracking gui, 9 => return start pose, 99 = > ignore start so return start pose
		self.rfid_sub = rospy.Subscriber('/finish', Int32, self.RFIDcallback)
		#finish: 1 => RFID check, 2 => go to start position
		self.cycle_sub = rospy.Subscriber('/go_back', Int32, self.cyclecallback)
		#go_back: 3 => arrived at start position

		#variable initial
		self.tracking_id = 0
		self.tracking_mode = 0
		self.distance1 = 0
		self.count = 0
		self.gui_check=0
		self.RFID_check=0
		self.cycle_check=0
		self.depth = np.empty((480,640))

		
#======================================================================================================

	def depthcallback(self, Image):
		self.bridge = CvBridge()

		depth_image = self.bridge.imgmsg_to_cv2(Image, desired_encoding="passthrough")
		self.depth = np.array(depth_image, dtype=np.float32)


	def RFIDcallback(self, data1):
		self.RFID_check = data1
		
	def cyclecallback(self, data2):
		self.cycle_check = data2

	def guicallback(self, data3):
		self.gui_check = data3
		

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
					self.distance1 = depths[0][0]
					self.tracking_mode = 1
					rospy.loginfo(self.tracking_id)
		
		#tracking_mode==1: check target
		elif self.tracking_mode == 1:
			
			self.tracking_mode = 2

		#tracking_mode==2: tracksing
		elif self.tracking_mode == 2:
			
			if self.RFID_check != 1:
				if self.gui_check == 1 or self.gui_check == 5:

					for box in range(len(detectbox.bounding_boxes)):

						bbox = detectbox.bounding_boxes[box]

						if self.tracking_id == bbox.id:
							center_x = round((bbox.xmin + bbox.xmax)/2)
							center_y = round((bbox.ymin + bbox.ymax)/2)
							distance2 = self.distance1
							distance = round(self.depth[int(center_y), int(center_x)]/10)					
							self.distance1 = distance
							
							#ignor huge or 0 current distance
							if abs(self.distance1- distance2) <= 180 and self.distance1 != 0:
							
								self.x_pub.publish(center_x)
								self.depth_pub.publish(distance)

                old_x = center_x
                old_y = center_y
                old_d = distance
                
								rospy.loginfo(self.tracking_id)
								rospy.loginfo('x: %d' % center_x)
								rospy.loginfo('depth: %d' % distance)
								rospy.loginfo('gui: %d' % self.gui_check)
								break

							else:
								rospy.loginfo('large moving points')
        
						else:

							if box+1 == len(detectbox.bounding_boxes):
								self.count += 1 

								if self.count > 3:
									self.tracking_mode = 5
									self.count = 0
									rospy.loginfo('return mode 5')				
								else:
									rospy.loginfo('waiting target person')

				#target person ignores start => return start position
				elif self.gui_check == 99:
					tracking_mode = 4
					rospy.loginfo('refuse help')
					rospy.loginfo(self.gui_check)

				else:
					pass	
          
			else:
				self.tracking_mode = 3
				rospy.loginfo('Arrived!')
				rospy.loginfo(self.gui_check)

		#tracking_mode==3: waiting for end of cat's motion
		elif self.tracking_mode == 3:
			
			if self.gui_check != 9:
				rospy.loginfo(self.gui_check)
			else:
				self.tracking_mode = 4
				rospy.loginfo('return mode')
				rospy.loginfo(self.gui_check)
		
    elif self.tracking_mode == 5:
			
			targets=[]
			for box in range(len(detectbox.bounding_boxes)):
				bbox = detectbox.bounding_boxes[box]
				center_x = round((bbox.xmin + bbox.xmax)/2)
				center_y = round((bbox.ymin + bbox.ymax)/2)
				distance = round(self.depth[int(center_y), int(center_x)]/10)

        if abs(old_x - center_x) < 100 and abs(old_y - center_y) < 80 and abs(old_d - distance) < 160:
          similar = abs(old_x - center_x) + abs(old_y - center_y) + abs(old_d - distance)
          targets.append([similar, bbox.id])
      
      #decide the most similar person for checking pre_tracking target
      targets.sort(key=lambda x: (x[0], x[1]))
    
			self.tracking_id = targets[0][1]
			self.tracking_mode = 1
			rospy.loginfo(self.tracking_id)
    
		#tracking_mode==4: return to start position
		else:
	
			if self.cycle_check != 3:

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
	
				#self.rate.sleep()
				
			else:
				self.tracking_mode == 0
				rospy.loginfo('restart scenario~')


#=========================================================================================

if __name__ == '__main__':
	
	rospy.init_node('vision', anonymous=True)
	
	tracker()

	#while not rospy.is_shutdown():	
	try:			
			#a.no_sub_bbox()
			#a.rate.sleep()
		rospy.spin()
	except (rospy.ROSInterruptException, SystemExit, KeyboardInterrupt):
		sys.exit(0)
