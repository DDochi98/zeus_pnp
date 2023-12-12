##################################################################################################
##       License: Apache 2.0. See LICENSE file in root directory.		                      ####
##################################################################################################
##                  Box Dimensioner with multiple cameras: Helper files 					  ####
##################################################################################################

import pyrealsense2 as rs
import numpy as np
import cv2
from realsense_device_manager import post_process_depth_frame
from helper_functions import convert_depth_frame_to_pointcloud, get_clipped_pointcloud
import math

def calculate_cumulative_pointcloud(frames_devices, calibration_info_devices, roi_2d, depth_threshold = 0.01):
	"""
 Calculate the cumulative pointcloud from the multiple devices
	Parameters:
	-----------
	frames_devices : dict
		The frames from the different devices
		keys: Tuple of (serial, product-line)
			Serial number and product line of the device
		values: [frame]
			frame: rs.frame()
				The frameset obtained over the active pipeline from the realsense device
				
	calibration_info_devices : dict
		keys: str
			Serial number of the device
		values: [transformation_devices, intrinsics_devices]
			transformation_devices: Transformation object
					The transformation object containing the transformation information between the device and the world coordinate systems
			intrinsics_devices: rs.intrinscs
					The intrinsics of the depth_frame of the realsense device
					
	roi_2d : array
		The region of interest given in the following order [minX, maxX, minY, maxY]
		
	depth_threshold : double
		The threshold for the depth value (meters) in world-coordinates beyond which the point cloud information will not be used.
		Following the right-hand coordinate system, if the object is placed on the chessboard plane, the height of the object will increase along the negative Z-axis
	
	Return:
	----------
	point_cloud_cumulative : array
		The cumulative pointcloud from the multiple devices
	"""
	# Use a threshold of 5 centimeters from the chessboard as the area where useful points are found
	point_cloud_cumulative = np.array([-1, -1, -1]).transpose()
	for (device_info, frame) in frames_devices.items() :
		device = device_info[0]
		# Filter the depth_frame using the Temporal filter and get the corresponding pointcloud for each frame
		filtered_depth_frame = post_process_depth_frame(frame[rs.stream.depth], temporal_smooth_alpha=0.1, temporal_smooth_delta=80)	
		point_cloud = convert_depth_frame_to_pointcloud( np.asarray( filtered_depth_frame.get_data()), calibration_info_devices[device][1][rs.stream.depth])
		point_cloud = np.asanyarray(point_cloud)

		# Get the point cloud in the world-coordinates using the transformation
		point_cloud = calibration_info_devices[device][0].apply_transformation(point_cloud)

		# Filter the point cloud based on the depth of the object
		# The object placed has its height in the negative direction of z-axis due to the right-hand coordinate system
		point_cloud = get_clipped_pointcloud(point_cloud, roi_2d)
		point_cloud = point_cloud[:,point_cloud[2,:]<-depth_threshold]
		point_cloud_cumulative = np.column_stack( ( point_cloud_cumulative, point_cloud ) )

	point_cloud_cumulative = np.delete(point_cloud_cumulative, 0, 1)
	return point_cloud_cumulative

def distance_cal(x1,y1,x2,y2):

	distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
	return distance

def calculate_boundingbox_points(point_cloud, calibration_info_devices, mm_roi_2d,depth_threshold = 0.01):
	"""
	Calculate the top and bottom bounding box corner points for the point cloud in the image coordinates of the color imager of the realsense device
	
	Parameters:
	-----------
	point_cloud : ndarray
		The (3 x N) array containing the pointcloud information
		
	calibration_info_devices : dict
		keys: str
			Serial number of the device
		values: [transformation_devices, intrinsics_devices, extrinsics_devices]
			transformation_devices: Transformation object
					The transformation object containing the transformation information between the device and the world coordinate systems
			intrinsics_devices: rs.intrinscs
					The intrinsics of the depth_frame of the realsense device
			extrinsics_devices: rs.extrinsics
					The extrinsics between the depth imager 1 and the color imager of the realsense device
					
	depth_threshold : double
		The threshold for the depth value (meters) in world-coordinates beyond which the point cloud information will not be used
		Following the right-hand coordinate system, if the object is placed on the chessboard plane, the height of the object will increase along the negative Z-axis
		
	Return:
	----------
	bounding_box_points_color_image : dict
		The bounding box corner points in the image coordinate system for the color imager
		keys: str
				Serial number of the device
			values: [points]
				points: list
					The (8x2) list of the upper corner points stacked above the lower corner points 
					
	length : double
		The length of the bounding box calculated in the world coordinates of the pointcloud
		
	width : double
		The width of the bounding box calculated in the world coordinates of the pointcloud
		
	height : double
		The height of the bounding box calculated in the world coordinates of the pointcloud
	"""
	# Calculate the dimensions of the filtered and summed up point cloud
	# Some dirty array manipulations are gonna follow
	if point_cloud.shape[1] > 500:
		# Get the bounding box in 2D using the X and Y coordinates
		coord = np.c_[point_cloud[0,:], point_cloud[1,:]].astype('float32')

		min_x = mm_roi_2d[0]
		max_x = mm_roi_2d[1]
		min_y = mm_roi_2d[2]
		max_y = mm_roi_2d[3]

		ma = distance_cal(min_x,min_y,min_x,max_y)	
		mb = distance_cal(min_x,max_y,max_x,min_y)

		#print("ma : ",ma)
		#print("mb : ",mb)
		#227.7
		#151.8

		min_max_2d = np.array([[min_x, min_y], [min_x, max_y], [max_x, max_y], [max_x, min_y]])

		min_area_rectangle = cv2.minAreaRect(coord)	
	
		bounding_box_world_2d = cv2.boxPoints(min_area_rectangle)
		p0_x = bounding_box_world_2d[0][0]
		p0_y = bounding_box_world_2d[0][1]
		
		p1_x = bounding_box_world_2d[1][1]
		p1_y = bounding_box_world_2d[1][1]

		p2_x = bounding_box_world_2d[2][0]
		p2_y = bounding_box_world_2d[2][1]
		
		da = distance_cal(p0_x,p0_y,p1_x,p1_y)	
		db = distance_cal(p1_x,p1_y,p2_x,p2_y)	

		#print("distance 1 : ", da)
		#print("distance 2 : ", db)
		#print(bounding_box_world_2d)

		# Caculate the height of the pointcloud
		height = max(point_cloud[2,:]) - min(point_cloud[2,:]) + depth_threshold
		# Get the upper and lower bounding box corner points in 3D
		height_array = np.array([[-height], [-height], [-height], [-height], [0], [0], [0], [0]])
		bounding_box_world_3d = np.column_stack((np.row_stack((bounding_box_world_2d,bounding_box_world_2d)), height_array))
		minmax_box_world_3d = np.column_stack((np.row_stack((min_max_2d,min_max_2d)), height_array))

		# Get the bounding box points in the image coordinates
		bounding_box_points_color_image={}
		mm_box_points_color_image={}
		for (device, calibration_info) in calibration_info_devices.items():
			# Transform the bounding box corner points to the device coordinates
			bounding_box_device_3d = calibration_info[0].inverse().apply_transformation(bounding_box_world_3d.transpose())
			minmax_device_3d = calibration_info[0].inverse().apply_transformation(minmax_box_world_3d.transpose())
			
			# Obtain the image coordinates in the color imager using the bounding box 3D corner points in the device coordinates
			color_pixel=[]
			mm_color_pixel=[]
			bounding_box_device_3d = bounding_box_device_3d.transpose().tolist()
			minmax_device_3d = minmax_device_3d.transpose().tolist()

			for bounding_box_point in bounding_box_device_3d: 
				bounding_box_color_image_point = rs.rs2_transform_point_to_point(calibration_info[2], bounding_box_point)			
				color_pixel.append(rs.rs2_project_point_to_pixel(calibration_info[1][rs.stream.color], bounding_box_color_image_point))
			
			for mixmax_box_point in minmax_device_3d: 
				minmax_box_color_image_point = rs.rs2_transform_point_to_point(calibration_info[2], mixmax_box_point)			
				mm_color_pixel.append(rs.rs2_project_point_to_pixel(calibration_info[1][rs.stream.color], minmax_box_color_image_point))

			bounding_box_points_color_image[device] = np.row_stack( color_pixel )
			mm_box_points_color_image[device] = np.row_stack( mm_color_pixel )
		#return bounding_box_points_color_image, min_area_rectangle[1][0], min_area_rectangle[1][1], height
		return bounding_box_points_color_image,mm_box_points_color_image, min_area_rectangle, height
	else : 
		return {},{},0,0

def calculate_line_equation_and_distance(x1, y1, x2, y2, x3, y3):
    # 두 점을 지나는 직선의 방정식 계산
    # 기울기 (m) 계산
    if x1 != x2:
        m = (y2 - y1) / (x2 - x1)
    else:
        m = math.inf  # 수직선의 경우 기울기가 무한대

    # y 절편 (b) 계산
    b = y1 - m * x1

    # 직선과 다른 점 C와의 거리 계산
    distance = abs(m * x3 - y3 + b) / math.sqrt(m**2 + 1)

    return distance


#def visualise_measurements(frames_devices, bounding_box_points_devices, length, width, height):
def visualise_measurements(frames_devices, bounding_box_points_devices,mm_box_points_color_image, area_rect, height):
	"""
 Calculate the cumulative pointcloud from the multiple devices
	
	Parameters:
	-----------
	frames_devices : dict
		The frames from the different devices
		keys: Tuple of (serial, product-line)
			Serial number and product line of the device
		values: [frame]
			frame: rs.frame()
				The frameset obtained over the active pipeline from the realsense device
				
	bounding_box_points_color_image : dict
		The bounding box corner points in the image coordinate system for the color imager
		keys: str
				Serial number of the device
			values: [points]
				points: list
					The (8x2) list of the upper corner points stacked above the lower corner points 
					
	length : double
		The length of the bounding box calculated in the world coordinates of the pointcloud
		
	width : double
		The width of the bounding box calculated in the world coordinates of the pointcloud
		
	height : double
		The height of the bounding box calculated in the world coordinates of the pointcloud
	"""
	for (device_info, frame) in frames_devices.items():
		device = device_info[0] #serial number

		if area_rect != 0:
			length = area_rect[1][0]
			width = area_rect[1][1]

			center_x = area_rect[0][0]
			center_y = area_rect[0][1]
		else :
			length = 0
			width = 0
		
		color_image = np.asarray(frame[rs.stream.color].get_data())
		if (length != 0 and width !=0 and height != 0):
			bounding_box_points_device_upper = bounding_box_points_devices[device][0:4,:]
			bounding_box_points_device_lower = bounding_box_points_devices[device][4:8,:]
			mm_box_points_color_image_lower = mm_box_points_color_image[device][4:8,:]
			
			box_info = "Length, Width, Height (mm): " + str(int(length*1000)) + ", " + str(int(width*1000)) + ", " + str(int(height*1000))

			# Draw the boundry 
			mm_box_points_color_image_lower = tuple(map(tuple,mm_box_points_color_image_lower.astype(int)))
			for i in range(len(mm_box_points_color_image_lower)):	
				test_txts = str(int(mm_box_points_color_image_lower[i][0])) + "," + str(int(mm_box_points_color_image_lower[i][1])) 
				cv2.putText(color_image, test_txts, mm_box_points_color_image_lower[i], cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255) )
				cv2.line(color_image, mm_box_points_color_image_lower[i], mm_box_points_color_image_lower[(i+1)%4], (0,0,255), 2)
			
			# Draw the box as an overlay on the color image		
			bounding_box_points_device_upper = tuple(map(tuple,bounding_box_points_device_upper.astype(int)))
			for i in range(len(bounding_box_points_device_upper)):	
				cv2.line(color_image, bounding_box_points_device_upper[i], bounding_box_points_device_upper[(i+1)%4], (0,255,0), 1)
				cv2.line(color_image, bounding_box_points_device_upper[i], bounding_box_points_device_upper[i], (0,255,0), 1)

			bounding_box_points_device_lower = tuple(map(tuple,bounding_box_points_device_lower.astype(int)))
			for i in range(len(bounding_box_points_device_upper)):	
				cv2.line(color_image, bounding_box_points_device_lower[i], bounding_box_points_device_lower[(i+1)%4], (0,255,0), 1)
				test_txt = str(int(bounding_box_points_device_lower[i][0])) + "," + str(int(bounding_box_points_device_lower[i][1])) 
				cv2.putText(color_image, test_txt, bounding_box_points_device_lower[i], cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0) )

			angle = int(area_rect[2])
			#print("length : ", length*1000)
			#print("width : ", width*1000)
			#print("angle : " ,angle
			box_cx = ( bounding_box_points_device_lower[0][0] + bounding_box_points_device_lower[2][0] ) / 2
			box_cy = ( bounding_box_points_device_lower[1][1] + bounding_box_points_device_lower[3][1] ) / 2
			
			cbox_info = "x, y: " + str(box_cx) + ", " + str(box_cy)
			
			cv2.putText(color_image, cbox_info, (int(box_cx),int(box_cy)) ,cv2.FONT_HERSHEY_PLAIN, 2, (0,0,255) )

			cv2.line(color_image, bounding_box_points_device_upper[1], bounding_box_points_device_lower[1], (0,255,0), 1)
			cv2.line(color_image, bounding_box_points_device_upper[2], bounding_box_points_device_lower[2], (0,255,0), 1)
			cv2.line(color_image, bounding_box_points_device_upper[3], bounding_box_points_device_lower[3], (0,255,0), 1)
			cv2.line(color_image, bounding_box_points_device_upper[0], bounding_box_points_device_lower[0], (0,255,0), 1)

			cv2.putText(color_image, box_info, (50,50), cv2.FONT_HERSHEY_PLAIN, 2, (0,255,0) )

			default_x = 585
			default_y = 400
			default_z = 158

			#dx = (int(mm_box_points_color_image_lower[0][0]) - int(box_cx) )/2
			#dy = (int(mm_box_points_color_image_lower[0][1]) - int(box_cy) ) /2
			dz = int(height*1000) / 2

			#print("dx : ", dx)
			#print("dy : ", dy)
			#print("dy : ", dz)

			#print("move x : ", default_x - abs(dx) )
			#print("move y : ", default_y - abs(dy) )
			#print("move z : ", default_z + dz)

			
			# 예제 사용법
			x1, y1 = int(mm_box_points_color_image_lower[0][0]), int(mm_box_points_color_image_lower[0][1])
			x2, y2 = int(mm_box_points_color_image_lower[3][0]), int(mm_box_points_color_image_lower[3][1])
			x2_2, y2_2 = int(mm_box_points_color_image_lower[1][0]), int(mm_box_points_color_image_lower[1][1])
			x3, y3 = box_cx, box_cy

			dx = calculate_line_equation_and_distance(x1, y1, x2, y2, x3, y3)
			print("x1,y1",x1,y1)
			print("x2,y2",x2,y2)
			print("x3,y3",x3,y3)

			print("dx 거리:", dx)	

			dy = calculate_line_equation_and_distance(x1, y1, x2_2, y2_2, x3, y3)
			print("x1,y1",x1,y1)
			print("x2,y2",x2_2,y2_2)
			print("x3,y3",x3,y3)

			print("dy 거리:", dy)

			print("move x : ", default_x - dx )
			print("move y : ", default_y - dy )
			print("move z : ", default_z + dz )
			print("angle  : ", angle)
		
		# Visualise the results
		cv2.imshow('Color image from RealSense Device Nr: ' + device, color_image)
		cv2.waitKey(1)