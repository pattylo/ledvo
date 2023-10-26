/*
    This file is part of ALan - the non-robocentric dynamic landing system for quadrotor

    ALan is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    ALan is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with ALan.  If not, see <http://www.gnu.org/licenses/>.
*/

/**
 * \file planner_server.cpp
 * \date 26/10/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for uav_path using airo_control_interface
 */

#include "include/planner_server.h"

planner_server::planner_server(ros::NodeHandle& _nh)
: nh(_nh)
{
    // subscribe
    uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose", 1, &planner_server::poseCallback, this);
    fsm_info_sub = nh.subscribe<airo_message::FSMInfo>
            ("/airo_control/fsm_info", 1, &planner_server::fsmInfoCallback, this);
    
    // publish            
    command_pub = nh.advertise<airo_message::Reference>
            ("/airo_control/setpoint",1, true);
    takeoff_land_pub = nh.advertise<airo_message::TakeoffLandTrigger>
            ("/airo_control/takeoff_land_trigger",1, true);
    
    config(nh);
}

planner_server::~planner_server()
{

}

void planner_server::config(ros::NodeHandle& _nh)
{
    // traj type
    
    std::string TrajType;
    _nh.getParam("/uav_path/TrajType",TrajType);
    
    Eigen::Vector3d center;
    _nh.getParam("/uav_path/x0", center.x());
    _nh.getParam("/uav_path/y0", center.y());
    _nh.getParam("/uav_path/z0", center.z());

    _nh.getParam("/uav_path/pub_freq", pub_freq);

    int lap_no;
    _nh.getParam("/uav_path/lap_no", lap_no);

    bool ccw;
    _nh.getParam("/uav_path/ccw", ccw);


    bool z_moving;
    _nh.getParam("/uav_path/z_moving", z_moving);
    double z_moving_amp;
    _nh.getParam("/uav_path/z_moving_amp", z_moving_amp);
    double z_moving_period;
    _nh.getParam("/uav_path/z_moving_perioud", z_moving_period);

    std::unique_ptr<uav::uavpath> uavpath_ptr;
    if(TrajType == CIRCLE_TRAJ)
    {
        double velo;
        _nh.getParam("/uav_path/cirle_velo",velo);
        double radius;
        _nh.getParam("/uav_path/circle_radius", radius);

        uavpath_ptr = std::make_unique<uav::uavpath>(
            center,
            pub_freq,
            lap_no,
            ccw,
            velo,
            radius,
            z_moving,
            z_moving_amp,
            z_moving_period,
            CIRCLE_TRAJ
        );
    }
    else if (TrajType == BLOCK_TRAJ)
    {
        double velo;
        _nh.getParam("/uav_path/block_velo",velo);
        double length;
        _nh.getParam("/uav_path/block_length", length);
        double aspect_ratio;
        _nh.getParam("/uav_path/block_AR", aspect_ratio);
        double T_angle; // dg
        _nh.getParam("/uav_path/block_T_angle", T_angle);

        uavpath_ptr = std::make_unique<uav::uavpath>(
            center,
            pub_freq,
            lap_no,
            ccw,
            velo,
            length,
            aspect_ratio,
            T_angle,
            z_moving,
            z_moving_amp,
            z_moving_period,
            BLOCK_TRAJ
        );
    }
    else if (TrajType == LEMNISCATE_TRAJ)
    {
        double velo;
        _nh.getParam("/uav_path/block_velo",velo);
        double length;
        _nh.getParam("/uav_path/block_length", length);
        double aspect_ratio;
        _nh.getParam("/uav_path/block_AR", aspect_ratio);
        double T_angle; // dg
        _nh.getParam("/uav_path/block_T_angle", T_angle);

        uavpath_ptr = std::make_unique<uav::uavpath>(
            center,
            pub_freq,
            lap_no,
            ccw,
            velo,
            length,
            aspect_ratio,
            T_angle,
            z_moving,
            z_moving_amp,
            z_moving_period,
            BLOCK_TRAJ
        );
    }
    
        
    
    

}

void planner_server::mainserver()
{

}
