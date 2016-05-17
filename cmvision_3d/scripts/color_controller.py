#!/usr/bin/env python

import rospy
import tf

from sensor_msgs.msg import Image, CameraInfo 
from image_geometry import PinholeCameraModel
from cmvision.msg import Blobs
from cmvision_3d.msg import Blobs3d, Blob3d

from cv_bridge import CvBridge

import cv, cv2
import numpy as np
from color_model import color_model

#This package integrates cmvision with tf and localization; now we can track color in 3D.
class color_controller():
	def __init__(self, publish_tf):
		#To take our Ros Image into a cv message and subsequently a numpy array.
		self.bridge = CvBridge()        

		# To make the pixel to vector projection
		self.cam_model = PinholeCameraModel()

		#We need CameraInfo in order to use PinholeCameraModel below.
		rospy.Subscriber("camera_topic", CameraInfo, self.camera_callback)
		self.hasCameraInfo = False

		while not self.hasCameraInfo:
			print "waiting on camera info."
			rospy.sleep(0.5)

		#We are using a depth image to get depth information of what we're tracking.
		rospy.Subscriber("depth_image", Image, self.depth_callback)

		#This package is just an extension of cmvision to provide tf tracking of the blobs provided by cmvision. 
		rospy.Subscriber('blobs', Blobs, self.blob_callback)

		#Subscribe to image for debugging.
		# rospy.Subscriber('thing', Image, self.image_callback)
		self.listener = tf.TransformListener()
		self.broadcaster = tf.TransformBroadcaster()

		#Republish each blob as part of a blob.
		self.blob_pub = rospy.Publisher('/blobs_3d', Blobs3d)

		self.publish_tf = publish_tf



		#blobs is received from running cmvision. It's color blobs as defined by our color file.
	def blob_callback(self, blobs):
		#array of our color_models.
		self.colors = {}
		blobs3d = Blobs3d()

		for blob in blobs.blobs:
			#If we have multiple catches of a single color, we only want to publish the one with the maximum area. Fortunately they are sorted as such.
			already_published = False
			if blob.name in self.colors:
				already_published = True

			self.colors[blob.name] = color_model(blob, self.camera_info, self.parent_frame, self.depth_image, self.cam_model, self.listener, self.broadcaster)
			
			#Make sure this blob isn't shitty.
			if self.colors[blob.name].validate():

				#Publish to tf if master wishes it upon Dobby the programming elf.
				if self.publish_tf and not already_published:
					self.colors[blob.name].publish()

				#3d blobs that are in this blobs3d list are published to the /blobs_3d topic. 
				blobs3d.blobs.append(self.colors[blob.name].toBlob3d())

		blobs3d.header.frame_id = self.parent_frame
		blobs3d.header.stamp = rospy.Time.now() 
		blobs3d.blob_count = len(blobs3d.blobs) 
		self.blob_pub.publish(blobs3d) 


	def depth_callback(self, image):
		image_cv = self.bridge.imgmsg_to_cv2(image, image.encoding)
		image_cv2 = np.squeeze(np.array(image_cv, dtype=np.float32))
		self.depth_image = image_cv2


	def camera_callback(self, camera_info):
		if not self.hasCameraInfo:
			self.cam_model.fromCameraInfo(camera_info)
			self.camera_info = camera_info
			self.parent_frame = self.camera_info.header.frame_id
		self.hasCameraInfo = True
		
if __name__ == '__main__':
	rospy.init_node('color_controller')
	
	publish_tf = rospy.get_param('color_controller/publish_tf', True)
	my_controller = color_controller(publish_tf)

	rospy.spin()
