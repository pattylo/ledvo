/*
    This file is part of  LEDVO - learning dynamic factor for visual odometry

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
 * \file planner_server.h
 * \date 01/11/2022
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for uav_path using airo_control_interface
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
    enum State{
        TAKEOFF,
        COMMAND,
        LAND
    };

private:
    ros::NodeHandle nh;

    State state = TAKEOFF;

//ros related
    // subscriber
    ros::Subscriber uav_pose_sub, fsm_info_sub;

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

    // publisher
    ros::Publisher command_pub, takeoff_land_pub;
    airo_message::TakeoffLandTrigger takeoff_land_trigger;

    // timer
    ros::Timer mainspin_timer;
    void mainspinCallback(const ros::TimerEvent &e);

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
    bool traj_predefined = false;
    void config(ros::NodeHandle& _nh);
    int pub_freq;
    std::vector<geometry_msgs::Point> TRAJECTORY;

// exec_traj
    double last_request = 0;
    double starting_error = 0;
    airo_message::Reference target_pose;

    void exec_predefined_traj();
    bool check_start_point();
    bool can_start = false;
    int traj_i = 0;

    void exec_online_traj();
    void check_collision();


public:
    planner_server(ros::NodeHandle& _nh);
    ~planner_server();

};

#endif