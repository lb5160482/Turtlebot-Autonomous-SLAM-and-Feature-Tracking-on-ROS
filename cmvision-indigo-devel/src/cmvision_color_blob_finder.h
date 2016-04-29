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

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <opencv/cv.h>

#include <cmvision/Blobs.h>
#include "conversions.h"
#include "cmvision.h"
#include "capture.h"

#define CMV_NUM_CHANNELS CMV_MAX_COLORS
#define CMV_HEADER_SIZE 4*CMV_NUM_CHANNELS
#define CMV_BLOB_SIZE 16
#define CMV_MAX_BLOBS_PER_CHANNEL 10

#define DEFAULT_CMV_WIDTH CMV_DEFAULT_WIDTH
#define DEFAULT_CMV_HEIGHT CMV_DEFAULT_HEIGHT

namespace color_blob_track
{

class CMVisionColorBlobFinder
{
public:

	/// \brief Constructor
	CMVisionColorBlobFinder();

	/// \brief Destructor
	virtual ~CMVisionColorBlobFinder();

	/*! \brief initialization function
	 * @param node_handle
	 * @return true if initialization successful, false else.
	 */
	bool initialize(ros::NodeHandle &node_handle);

	/// \brief Image callback
	void imageCB(const sensor_msgs::ImageConstPtr& msg);

private:

	ros::Publisher blob_publisher_;
	ros::Subscriber image_subscriber_;

	// sensor_msgs::Image image;

	bool debug_on_;
	uint16_t width_;
	uint16_t height_;
	std::string color_filename_;
	uint8_t *uyvy_image_;

	unsigned int blob_count_;

	CMVision *vision_;

	cmvision::Blobs blob_message_;

	bool mean_shift_on_;
	double spatial_radius_;
	double color_radius_;

};

}
