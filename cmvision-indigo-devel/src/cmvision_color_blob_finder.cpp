/*
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *     Andrew Martignoni III
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*
 * Uses CMVision to retrieve the blob data
 */
// author Andy Martignoni III, Brian Gerkey, Brendan Burns, Ben Grocholsky, Brad Kratochvil

#include <opencv/highgui.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>

#include "cmvision_color_blob_finder.h"

using namespace color_blob_track;

CMVisionColorBlobFinder::CMVisionColorBlobFinder() :
	debug_on_(false), width_(0), height_(0), color_filename_(""), uyvy_image_(NULL), blob_count_(0), vision_(NULL), mean_shift_on_(false), spatial_radius_(0),
			color_radius_(0)
{
}

CMVisionColorBlobFinder::~CMVisionColorBlobFinder()
{
	if (vision_)
	{
		delete vision_;
	}
}

bool CMVisionColorBlobFinder::initialize(ros::NodeHandle &node_handle)
{

	uyvy_image_ = NULL;
	width_ = 0;
	height_ = 0;

	vision_ = new CMVision();

	blob_count_ = 0;
	blob_message_.blob_count = 0;

	// Get the color file. This defines what colors to track
	if (!node_handle.getParam("/cmvision/color_file", color_filename_))
	{
		ROS_ERROR("Could not find color calibration file name \"/cmvision/color_file\" in namespace: %s.", node_handle.getNamespace().c_str());
		return false;
	}

	// Get the level of debug output
	node_handle.param("/cmvision/debug_on", debug_on_, false);

	// check whether mean shift is turned on
	if (!node_handle.getParam("/cmvision/mean_shift_on", mean_shift_on_))
	{
		ROS_ERROR("Could not find mean shift flag \"/cmvision/mean_shift_on\" in namespace: %s.", node_handle.getNamespace().c_str());
		return false;
	}
	else
	{
		if (!node_handle.getParam("/cmvision/spatial_radius_pix", spatial_radius_))
		{
			ROS_ERROR("Could not get spatial_radius_pix from param server \"/cmvision/spatial_radius_pix\" in namespace: %s.", node_handle.getNamespace().c_str());
			return false;
		}

		if (!node_handle.getParam("/cmvision/color_radius_pix", color_radius_))
		{
			ROS_ERROR("Could not get color_radius_pix from param server \"/cmvision/color_radius_pix\" in namespace: %s.", node_handle.getNamespace().c_str());
			return false;
		}
	}

	// Subscribe to an image stream
	image_subscriber_ = node_handle.subscribe("image", 1, &CMVisionColorBlobFinder::imageCB, this);

	// Advertise our blobs
	blob_publisher_ = node_handle.advertise<cmvision::Blobs> ("blobs", 1);

	if (debug_on_)
	{
		cvNamedWindow("Image");
	}

	return true;
}

void CMVisionColorBlobFinder::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
	IplImage cvImageRef, *cvImage;
	CvSize size;

	const sensor_msgs::Image img = *msg;

	// Timing
	// struct timeval timeofday;
	// gettimeofday(&timeofday,NULL);
	// ros::Time startt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);
	ros::WallTime startt = ros::WallTime::now();

	// Get the image as and RGB image
        cv_bridge::CvImagePtr image = cv_bridge::toCvCopy(msg, "rgb8");
        cvImageRef = IplImage(image->image);
        cvImage = &cvImageRef;
        
	size = cvGetSize(cvImage);

	// this shouldn't change often
	if ((size.width != width_) || (size.height != height_))
	{
		if (!(vision_->initialize(size.width, size.height)))
		{
			width_ = height_ = 0;
			ROS_ERROR("Vision init failed.");
			return;
		}

		if (!color_filename_.empty())
		{
			if (!vision_->loadOptions(color_filename_.c_str()))
			{
				ROS_ERROR("Error loading color file");
				return;
			}
		}
		else
		{
			ROS_ERROR("No color file given.  Use the \"mColorFile\" "
				"option in the configuration file.");
			return;
		}

		width_ = size.width;
		height_ = size.height;

		blob_message_.image_width = size.width;
		blob_message_.image_height = size.height;

		if (uyvy_image_)
		{
			delete[] uyvy_image_;
		}
		uyvy_image_ = new uint8_t[width_ * height_ * 2];

	}

	// Smooth the image, if turned on
	if (mean_shift_on_)
	{
		cvPyrMeanShiftFiltering(cvImage, cvImage, spatial_radius_, color_radius_);
	}

	// Convert image to YUV color space
	rgb2uyvy((unsigned char *) cvImage->imageData, uyvy_image_, width_ * height_);

	if (debug_on_)
	{
		cvCvtColor(cvImage, cvImage, CV_RGB2BGR);
	}

	// Find the color blobs
	if (!vision_->processFrame(reinterpret_cast<image_pixel*> (uyvy_image_)))
	{
		ROS_ERROR("Frame error.");
		return;
	}

  //image_bridge_.fromImage(*msg, "bgr8");
  //cvImage = image_bridge_.toIpl();

	// Get all the blobs
	blob_count_ = 0;
	for (int ch = 0; ch < CMV_MAX_COLORS; ++ch)
	{
		// Get the descriptive color
		rgb c = vision_->getColorVisual(ch);
		char* name = vision_->getColorName(ch);

		// Grab the regions for this color
		CMVision::region* r = NULL;

		for (r = vision_->getRegions(ch); r != NULL; r = r->next)
		{
			// Resize the blob message
			if (blob_count_ >= blob_message_.blobs.size())
			{
				blob_message_.blobs.resize(blob_message_.blobs.size() + 1);
			}

			if (debug_on_)
			{
				cvRectangle(cvImage, cvPoint(r->x1, r->y1), cvPoint(r->x2, r->y2), CV_RGB(c.red, c.green, c.blue));
			}

			blob_message_.blobs[blob_count_].name = name;
			blob_message_.blobs[blob_count_].red = c.red;
			blob_message_.blobs[blob_count_].green = c.green;
			blob_message_.blobs[blob_count_].blue = c.blue;
			blob_message_.blobs[blob_count_].area = r->area;
			blob_message_.blobs[blob_count_].x = rint(r->cen_x + .5);
			blob_message_.blobs[blob_count_].y = rint(r->cen_y + .5);
			blob_message_.blobs[blob_count_].left = r->x1;
			blob_message_.blobs[blob_count_].right = r->x2;
			blob_message_.blobs[blob_count_].top = r->y1;
			blob_message_.blobs[blob_count_].bottom = r->y2;

			blob_count_++;
		}
	}

  if (blob_count_ < blob_message_.blobs.size())
    blob_message_.blobs.resize(blob_count_);

	if (debug_on_)
	{
		cvShowImage("Image", cvImage);
		cvWaitKey(3);
	}

	blob_message_.blob_count = blob_count_;

	blob_message_.header.stamp = ros::Time::now();

	blob_publisher_.publish(blob_message_);

	// Timing
	// gettimeofday(&timeofday,NULL);
	// ros::Time endt = ros::Time().fromNSec(1e9*timeofday.tv_sec + 1e3*timeofday.tv_usec);
	ros::WallTime endt = ros::WallTime::now();
	ros::WallDuration diff = endt - startt;
	ROS_DEBUG_STREAM_NAMED("cmvision", "Color Blob Detection duration " << diff.toSec());
}
