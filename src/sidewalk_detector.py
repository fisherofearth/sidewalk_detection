#!/usr/bin/env python

#######################################################################
# Auther: Xi Yu (Fisher)
# Date: 04/30/2016
#
# This node provides an openCV-based algorithm which detects sidewalk 
#	from a real video record. This algorithm assume that the bottom 
# 	center of an imageframe is always capturing sidewalk. This area is 
#	set as reference of sidewalk. Similaritiy is computed between
# 	different ROI and ref_ROI. The finnal detection result is computed
#	by searching algorith based on similarities and Canny edges.
# This 2-day programming challenge is set to output point cloud as 
#	depth description for sidewalk and non-sidewalk. However, Python
#	does not contain PCL API. So this task has not be done yet.

#######################################################################

import rospy
import numpy as np

import message_filters
from sensor_msgs.msg import Image, PointCloud2

from cv_bridge import CvBridge, CvBridgeError
import cv2

#import sensor_msgs.point_cloud2 as pc2


step_number = 20 	# 10 : 2 : 30
T_edge = -0.08 	# -1.5 : 0.1 : -0.5
T_similar = 0.7 # 0.20 : 0.01 : 1.00

kernel_sharpen = np.array([[-1,-1,-1,-1,-1],
							[-1,2,2,2,-1],
							[-1,2,8,2,-1],
							[-1,2,2,2,-1],
							[-1,-1,-1,-1,-1]]) / 8.0

similarities_history = [] # used for mean filter
color_sidewalk = (0,0, 255)
bridge = CvBridge() # convert between openCV image and ImageMsgs

### Define publisher
pub_color_img_raw = rospy.Publisher('/sidewalk_detector/color/image_raw',  Image, queue_size=10)
#pub_depth_img_raw = rospy.Publisher('/sidewalk_detector/depth/image_raw',  Image, queue_size=10)
pub_canny_edge = rospy.Publisher('/sidewalk_detector/color/canny_edge',  Image, queue_size=10)
#pub_marker = rospy.Publisher('/sidewalk_detector/color/marker',  Image, queue_size=10)
pub_depth_pts_in  = rospy.Publisher('/sidewalk_detector/depth/points_in',  PointCloud2, queue_size=10)
pub_depth_pts_out = rospy.Publisher('/sidewalk_detector/depth/points_out', PointCloud2, queue_size=10)

'''
def Seperate_Cloud(cloud_points, mask_in):
	pts_in, pts_out= [], []
	for p in pc2.read_points(cloud_points, field_names = ("x", "y", "z"), skip_nans=True):
		if p[2] > 0:
			if mask_in[p[1], p[0]] == color_sidewalk:
				pts_in.append([p[0], p[1], p[2]])	
			else:
				pts_out.append([p[0], p[1], p[2]])			
	return create_cloud_xyz32(cloud_points.header, pts_in), create_cloud_xyz32(cloud_points.header, pts_out)
'''

class Sidewalk_detector:
	def __init__(self, step_number):
		global T_edge
		global T_similar
		self.counter = 0
		self.step_number = step_number

		sub_color_img_raw = message_filters.Subscriber('/camera/color/image_raw',Image)
		sub_depth_img_raw = message_filters.Subscriber('/camera/depth/image_raw',Image)
		sub_depth_pts = message_filters.Subscriber('/camera/depth/points',PointCloud2)

		ts = message_filters.ApproximateTimeSynchronizer([sub_color_img_raw, sub_depth_pts], 10, 1)
		ts.registerCallback(self.callback)

	def Fill_gap(self, img_edge, w_box, h_box, x_box, y_box, y_up):
	# Fill gap  between high-similarity ROI and closest edge
	# Param 	img_edge 	: Canny edge image 
	# Param 	w_box		: Width of ROI
	# Param 	h_box		: height of ROI
	# Param 	x_box		: x position of ROI
	# Param 	y_box		: y position of ROI
	# Param 	y_up		: search limit
		lines_edge_gap = []
		for x in xrange(x_box, x_box+w_box):
			idx_edge = np.nonzero(img_edge[y_up:y_box+h_box, x])[0]
			if len(idx_edge) > 0:
				y = idx_edge[-1]+y_up
			else:
				y = y_box+h_box
			lines_edge_gap.append([(x,y_box+h_box), (x, y)])
		return lines_edge_gap
	
	def Compare_similarity(self, refBox, cropBox):
	# Compute the similarity between two ROI
	# Param 	refBox 	: ROI for reference
	# Param 	cropBox : ROI for comparing
		H1 = cv2.normalize(cv2.calcHist(refBox,  [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])).flatten()
		H2 = cv2.normalize(cv2.calcHist(cropBox, [0, 1, 2], None, [8, 8, 8], [0, 256, 0, 256, 0, 256])).flatten()
		return cv2.compareHist(H1, H2, cv2.cv.CV_COMP_CORREL)


	def callback(self, sub_color_img_raw, sub_depth_pts):
		T_edge = rospy.get_param('T_edge')
		T_similar = rospy.get_param('T_similar')

		#### convert Image_msg to ndarry
		try:
			cv_color_img_raw = bridge.imgmsg_to_cv2(sub_color_img_raw, "bgr8")
		except CvBridgeError as e:
			print(e)

		### 180 degree rotatation
		h,w = cv_color_img_raw.shape[:2]
		M = cv2.getRotationMatrix2D((w/2,h/2),180,1)
		cv_color_img_raw = cv2.warpAffine(cv_color_img_raw,M,(w,h))

		#### sharpen
		cv_color_img_raw = cv2.filter2D(cv_color_img_raw, -1, kernel_sharpen)

		### Canny edge detection
		img_edge = cv2.Canny(cv_color_img_raw,100,250)

		#### Create a marker
		cv_img_marker = cv_color_img_raw.copy()

		#### Compute similarities
		## Define ROI size
		w_box, h_box = w/self.step_number, h/self.step_number

		## Define reference ROI 
		wr, hr = 200, 150
		xr, yr = (w-wr)/2, h-hr,
		
		## Compute similarity for each ROI
		similarities = []
		c = 0
		for j in xrange(self.step_number):
			for i in xrange(self.step_number):
				xc, yc, wc, hc = i*w_box, j*h_box, w_box, h_box
				d = self.Compare_similarity(cv_color_img_raw[yr:yr+hr, xr:xr+wr], cv_color_img_raw[yc:yc+hc, xc:xc+wc])
	
				# mean filter
				history = []
				if self.counter > 10:
					for idx in range(-10,0):
						history.append(similarities_history[idx][c][4])
				elif self.counter > 1:
					for idx in range(-self.counter,0):
						history.append(similarities_history[idx][c][4])		
				history.append(d)
				d = np.mean(history)

				similarities.append([xc, yc, wc, hc, d])
				c += 1
		similarities_history.append(similarities)

		### Search side walk edge
		boxex_array = np.zeros(shape=(self.step_number,self.step_number,2), dtype=int)
		similarities_array = np.zeros(shape=(self.step_number,self.step_number), dtype=float)
		is_sidewalk = np.zeros(shape=(self.step_number,self.step_number), dtype=bool)
		for y in xrange(self.step_number):
			for x in xrange(self.step_number):
				boxex_array[x,y] = similarities[(self.step_number*y) + x][0:2]
				similarities_array[x,y] = similarities[(self.step_number*y) + x][4]

		##  Leftward search
		is_sidewalk[self.step_number/2-1, self.step_number-1] = True
		for left in xrange(self.step_number/2-1):
			idx_x, idx_y = self.step_number/2-2-left, self.step_number-1
			if (similarities_array[idx_x, idx_y] - similarities_array[idx_x+1, idx_y]) > T_edge: # if not an edge
				is_sidewalk[idx_x, idx_y] = True
			elif similarities_array[idx_x, idx_y] > T_similar/2:
				is_sidewalk[idx_x, idx_y] = True
			else:
				break # if edge, break

		##  Rightward search
		is_sidewalk[self.step_number/2, self.step_number-1] = True
		for right in xrange(self.step_number/2-1):
			idx_x, idx_y = self.step_number/2+right+1, self.step_number-1
			if (similarities_array[idx_x, idx_y] - similarities_array[idx_x-1, idx_y]) > T_edge: # if not an edge
				is_sidewalk[idx_x, idx_y] = True
			elif similarities_array[idx_x, idx_y] > T_similar/2:
				is_sidewalk[idx_x, idx_y] = True
			else:
				break # if edge, break

		## Rpward search
		lines_edge_gap = []
		for idx_x in xrange(self.step_number):
			if is_sidewalk[idx_x, self.step_number-1] == True:
				for up in xrange(self.step_number-1):
					idx_y = self.step_number-2-up
					if (similarities_array[idx_x, idx_y] - similarities_array[idx_x, idx_y+1]) > T_edge: # if not an edge
						is_sidewalk[idx_x, idx_y] = True
					elif similarities_array[idx_x, idx_y] > T_similar:
						is_sidewalk[idx_x, idx_y] = True
					else:
						y = idx_y
						while y > 0 and similarities_array[idx_x, y] > T_similar/3:
							y -=1
						y_up = boxex_array[idx_x, y][1]
						x_box, y_box = boxex_array[idx_x, idx_y][0], boxex_array[idx_x, idx_y][1]
						lines_edge_gap += self.Fill_gap(img_edge, w_box, h_box, x_box, y_box, y_up)
						break

		# draw mask
		img_mask_sidewalk = cv_color_img_raw.copy()
		for x in xrange(is_sidewalk.shape[0]):
			for y in xrange(is_sidewalk.shape[1]):
				if is_sidewalk[x,y] == True:
					box = boxex_array[x,y]
					img_mask_sidewalk[box[1]:box[1]+h_box, box[0]:box[0]+w_box]=color_sidewalk
		
		for pts in lines_edge_gap:
			cv2.line(img_mask_sidewalk, pts[0], pts[1], color_sidewalk, thickness=1)

		cv_img_marker = cv2.addWeighted(img_mask_sidewalk, 0.5, cv_color_img_raw, 0.5, 0)
		#cv2.rectangle(cv_img_marker, (xr,yr),(xr+wr,yr+hr),(255,0,0),1)
		#cv2.rectangle(cv_img_marker, (xc,yc),(xc+wc,yc+hc),color_sidewalk,1)
		#cv2.putText(cv_img_marker,str("%.2f"%d),(xc,yc+12), cv2.FONT_HERSHEY_SIMPLEX, 0.4,color_sidewalk,1)

		### seperate point cloud
		# Since Python does not have PCL API, fail to seperate 
		# point cloud in an efficient way. Should use C++
		pts_in, pts_out = sub_depth_pts, sub_depth_pts 

		'''
		h,w = img_mask_sidewalk.shape[:2]
		M = cv2.getRotationMatrix2D((w/2,h/2),180,1)
		mask_in = cv2.warpAffine(img_mask_sidewalk,M,(w,h))
		mask_in = cv2.resize(mask_in,(480,360))
		#pts_in, pts_out = Seperate_Cloud(sub_depth_pts, mask_in)
		'''

		### convert ndarry to Image_msg
		try:
			imgmsg_image = bridge.cv2_to_imgmsg(cv_img_marker, "bgr8")
			imgmsg_canny_edge = bridge.cv2_to_imgmsg(img_edge, "mono8")
		except CvBridgeError as e:
			print(e)


		### publish topics
		pub_color_img_raw.publish(imgmsg_image)
		pub_depth_pts_in.publish(pts_in)  
		pub_depth_pts_out.publish(pts_out) 
		pub_canny_edge.publish(imgmsg_canny_edge)

		self.counter += 1

if __name__ == '__main__':
	rospy.init_node('sidewalk_detector')

	detector = Sidewalk_detector(step_number)

	rospy.spin()