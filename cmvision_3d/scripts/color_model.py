#!/usr/bin/env python
import rospy
import tf

from image_geometry import PinholeCameraModel
from geometry_msgs.msg import TransformStamped, PointStamped
from cmvision.msg import Blobs, Blob
from cmvision_3d.msg import Blob3d

import numpy as np
from math import isnan

#The "model" part of our model-view-controller. 
#Each color_model represents a color that we're tracking. When calling update(), you present new information.
class color_model():
	def __init__(self, blob, camera_info, parent_frame, depth_image, cam_model, listener, broadcaster):
		
		#Our initial blob information.
		self.blob = blob

		#The frame we wish our blobs to have as a parent. E.g, this is "/map" if I'm localizing to the /map frame.
		self.parent_frame = parent_frame
		
		#Depth image is important for projecting the blob to 3D.
		self.depth_image = depth_image
		
		#Our projected color blob exists in this frame.
		self.camera_frame = camera_info.header.frame_id
		
		#Cam_model is important for projections to and from 2d/3d.
		self.cam_model = cam_model

		#Listener is necessary to transform from camera_frame to parent_frame.
		self.listener = listener

		#Broadcaster to publish the transforms.
		self.broadcaster = broadcaster
	
	# Error checking for blob to see if it's worthwhile to even publish. Returns true if so.
	def validate(self):
		return ( self._validateDepthAt(self.blob.x, self.blob.y) and 
				self._validateDepthAt(self.blob.left, self.blob.top) and 
				self._validateDepthAt(self.blob.right, self.blob.bottom) )
		
		return False

	#Publishes to our view, color_broadcaster, if the model updates. 
	def publish(self):
		transform = self._toTransform(self.blob.x, self.blob.y)
		pos = (transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z)
		rot = (transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w)

		self.broadcaster.sendTransform(pos, rot, rospy.Time.now(), transform.child_frame_id, transform.header.frame_id)

		return transform
	def toBlob3d(self):
		blob3d = Blob3d()
		
		blob3d.name = self.blob.name
		blob3d.red = self.blob.red
		blob3d.green = self.blob.green
		blob3d.blue = self.blob.blue
		blob3d.area = self.blob.area

		transform = self._toTransform(self.blob.x, self.blob.y)
		blob3d.center.x = transform.transform.translation.x
		blob3d.center.y = transform.transform.translation.y
		blob3d.center.z = transform.transform.translation.z

		transform = self._toTransform(self.blob.left, self.blob.top)
		blob3d.top_left.x = transform.transform.translation.x
		blob3d.top_left.y = transform.transform.translation.y
		blob3d.top_left.z = transform.transform.translation.z

		transform = self._toTransform(self.blob.right, self.blob.bottom)
		blob3d.bottom_right.x = transform.transform.translation.x
		blob3d.bottom_right.y = transform.transform.translation.y
		blob3d.bottom_right.z = transform.transform.translation.z

		return blob3d

## Private functions
## ^^^^^^^^^^^^^^^^^
	
	#Takes our data and makes a tf2 transform message.
	def _toTransform(self, my_x, my_y):
		transform = TransformStamped()
		transform.header.stamp = rospy.Time.now()
		transform.header.frame_id = self.camera_frame
		transform.child_frame_id = self.blob.name

		(x,y,z) = self._projectTo3d(my_x, my_y)
		transform.transform.translation.x = x
		transform.transform.translation.y = y
		transform.transform.translation.z = z

		transform.transform.rotation.w = 1.0

		#If our parent frame is not the camera frame then an additional transformation is required.
		if self.parent_frame != self.camera_frame:
			point = PointStamped()
			point.header.frame_id = self.camera_frame
			point.header.stamp = rospy.Time(0)
			point.point.x = transform.transform.translation.x
			point.point.y = transform.transform.translation.y
			point.point.z = transform.transform.translation.z

			#Now we've gone from the regular camera frame to the correct parent_frame.
			point_transformed = self.listener.transformPoint(self.parent_frame, point)
			
			transform.header.frame_id = self.parent_frame
			transform.transform.translation.x = point_transformed.point.x
			transform.transform.translation.y = point_transformed.point.y
			transform.transform.translation.z = point_transformed.point.z

		return transform

	def _projectTo3d(self, x, y):
		[vx,vy,vz] = self.cam_model.projectPixelTo3dRay((x,y))
		blob_z = self._getDepthAt(x,y)
		blob_x = vx * blob_z
		blob_y = vy * blob_z

		return (blob_x, blob_y, blob_z)

	def _getDepthAt(self, x,y):
		return self.depth_image[y][x]/1000

	def _validateDepthAt(self, x, y):
		depth = self._getDepthAt(x, y)
		if isnan(depth) or depth == 0:
			return False
		if self.blob.area == 0:
			return False
		return True		

