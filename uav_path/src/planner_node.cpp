/*
    This file is part of LEDVO - learning dynamic factor for visual odometry

    LEDVO is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    LEDVO is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with LEDVO.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file planner_node.cpp
 * \date 26/10/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for uav_path using airo_control_interface
 */

#include "include/planner_server.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner_node");
    ros::NodeHandle nh("~");

    planner_server planner_server(nh);
    
    ros::spin();

    return 0;
}