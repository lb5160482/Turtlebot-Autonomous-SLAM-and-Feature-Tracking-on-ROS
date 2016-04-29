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

#include <ros/ros.h>
#include "cmvision_color_blob_finder.h"

int main( int argc, char **argv)
{
  ros::init(argc, argv, "cmvision");
  ros::NodeHandle node_handle;

  color_blob_track::CMVisionColorBlobFinder cmvision;
  if(!cmvision.initialize(node_handle))
  {
  	ROS_ERROR_NAMED("cmvision", "could not initialize cmvision color blob finder.");
  	return -1;
  }
  ros::spin();
  return 0;
}
