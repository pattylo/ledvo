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
 * \file planner_server.h
 * \date 01/11/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for alan_landing_planning FSM
 */

#ifndef PLANNER_SERVER_H
#define PLANNER_SERVER_H

#include "essential.h"

#include "airo_message/FSMInfo.h"
#include "airo_message/TakeoffLandTrigger.h"
#include "airo_message/Reference.h"

#include "uavpath_lib.hpp"

// #define REPLAN "IDLE"

class planner_server
{
    typedef struct waypts
    {
        double x;
        double y;
        double z;
        double yaw;
    }waypts;

private:
    ros::NodeHandle nh;

//ros related
    //subscriber
    ros::Subscriber uav_pose_sub, fsm_info_sub;

    //publisher
    ros::Publisher command_pub, takeoff_land_pub;
    
    geometry_msgs::PoseStamped uav_pose;
    
    inline void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        uav_pose = *msg;
    }

    airo_message::FSMInfo fsm_info;
    inline void fsmInfoCallback(const airo_message::FSMInfo::ConstPtr& msg)
    {
        fsm_info = *msg;
    }

//rotation function
    Eigen::Vector3d q2rpy(Eigen::Quaterniond q) {
        return q.toRotationMatrix().eulerAngles(1,0,2);
    };

    Eigen::Quaterniond rpy2q(Eigen::Vector3d rpy){
        Eigen::AngleAxisd rollAngle(rpy(0), Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd pitchAngle(rpy(1), Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd yawAngle(rpy(2), Eigen::Vector3d::UnitZ());

        Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;

        return q;

    };

    Eigen::Vector3d q_rotate_vector(Eigen::Quaterniond q, Eigen::Vector3d v){
        return q * v;
    }

// config
    void config(ros::NodeHandle& _nh);
    int pub_freq;
    std::vector<geometry_msgs::Point> traj;

public:
    planner_server(ros::NodeHandle& _nh);
    ~planner_server();

    void mainserver();

};

#endif