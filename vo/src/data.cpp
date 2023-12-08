/*
    This file is part of LEDVO - the non-robocentric dynamic landing system for quadrotor

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
 * \file data.cpp
 * \date 21/11/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief executable to record data
 */

#include "./include/essential.h"
#include "./include/cameraModel.hpp"

#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>


static std::string log_file_1;
static std::string log_file_2;
static double starting_time;
static geometry_msgs::PoseStamped ugv_pose;

static vision::cameraModel tool;

static bool log_start = false;
static Eigen::Vector3d ctrl_rpy_previous;

static geometry_msgs::PoseStamped gt_pose;

void gt_callback(
    const geometry_msgs::PoseStamped::ConstPtr& gt_msg
)
{
    gt_pose = *gt_msg;
}

void ctrl_msg_callback(
    const mavros_msgs::AttitudeTarget::ConstPtr& ctrl_msg
)
{
    std::cout<<ctrl_msg->header.stamp.now().toSec() - starting_time<<std::endl;
    if(!log_start)
    {
        log_start = true;
        starting_time = ros::Time::now().toSec();

        ctrl_rpy_previous.setZero();
    }
    
    std::ofstream save(log_file_1, std::ios::app);

    Eigen::Vector3d ctrl_rpy = tool.q2rpy(
        Eigen::Quaterniond(
            ctrl_msg->orientation.w,
            ctrl_msg->orientation.x,
            ctrl_msg->orientation.y,
            ctrl_msg->orientation.z
        )
    );

    Eigen::Vector3d gt_rpy = tool.q2rpy(
        Eigen::Quaterniond(
            gt_pose.pose.orientation.w,
            gt_pose.pose.orientation.x,
            gt_pose.pose.orientation.y,
            gt_pose.pose.orientation.z
        )
    );

    Eigen::Vector3d d_ctrl_rpy = ctrl_rpy - ctrl_rpy_previous;
    
    // "t, f,r,p,y, dr,dp,dy, gt_x,gt_y,gt_z, gt_r,gt_p,gt_y"
    save 
         << ctrl_msg->header.stamp.now().toSec() - starting_time << "," 

         << ctrl_msg->thrust << "," 
         << ctrl_rpy(0) << "," 
         << ctrl_rpy(1) << "," 
         << ctrl_rpy(2) << "," 

         << d_ctrl_rpy(0) << ","
         << d_ctrl_rpy(1) << ","
         << d_ctrl_rpy(2) << "," 

         << gt_pose.pose.position.x << ","
         << gt_pose.pose.position.y << ","
         << gt_pose.pose.position.z << ","

         << gt_rpy(0) << "," 
         << gt_rpy(1) << "," 
         << gt_rpy(2) << "," 

         << std::endl;

    save.close();

    ctrl_rpy_previous = ctrl_rpy;

}

void msg_callback(
    const mavros_msgs::AttitudeTarget::ConstPtr& ctrl_msg,
    const geometry_msgs::PoseStamped::ConstPtr& gt_msg
)
{    
    std::cout<<ctrl_msg->header.stamp.now().toSec() - starting_time<<std::endl;
    if(!log_start)
    {
        log_start = true;
        starting_time = ros::Time::now().toSec();

        ctrl_rpy_previous.setZero();
    }
    
    std::ofstream save(log_file_1, std::ios::app);

    Eigen::Vector3d ctrl_rpy = tool.q2rpy(
        Eigen::Quaterniond(
            ctrl_msg->orientation.w,
            ctrl_msg->orientation.x,
            ctrl_msg->orientation.y,
            ctrl_msg->orientation.z
        )
    );

    Eigen::Vector3d gt_rpy = tool.q2rpy(
        Eigen::Quaterniond(
            gt_msg->pose.orientation.w,
            gt_msg->pose.orientation.x,
            gt_msg->pose.orientation.y,
            gt_msg->pose.orientation.z
        )
    );

    Eigen::Vector3d d_ctrl_rpy = ctrl_rpy - ctrl_rpy_previous;
    
    // "t, f,r,p,y, dr,dp,dy, gt_x,gt_y,gt_z, gt_r,gt_p,gt_y"
    save 
         << ctrl_msg->header.stamp.now().toSec() - starting_time << "," 

         << ctrl_msg->thrust << "," 
         << ctrl_rpy(0) << "," 
         << ctrl_rpy(1) << "," 
         << ctrl_rpy(2) << "," 

         << d_ctrl_rpy(0) << ","
         << d_ctrl_rpy(1) << ","
         << d_ctrl_rpy(2) << "," 

         << gt_msg->pose.position.x << ","
         << gt_msg->pose.position.y << ","
         << gt_msg->pose.position.z << ","

         << gt_rpy(0) << "," 
         << gt_rpy(1) << "," 
         << gt_rpy(2) << "," 

         << std::endl;

    save.close();

    ctrl_rpy_previous = ctrl_rpy;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_log");
    ros::NodeHandle nh;

    message_filters::Subscriber<mavros_msgs::AttitudeTarget> subctrl;
    message_filters::Subscriber<geometry_msgs::PoseStamped> subgt;
    typedef message_filters::sync_policies::ApproximateTime<mavros_msgs::AttitudeTarget, geometry_msgs::PoseStamped> MySyncPolicy;
    typedef message_filters::Synchronizer<MySyncPolicy> sync;//(MySyncPolicy(10), subimage, subdepth);
    boost::shared_ptr<sync> sync_;   

    // subctrl.subscribe(nh, "/mavros/setpoint_raw/attitude", 1);                
    // subgt.subscribe(nh, "/mavros/vision_pose/pose", 1);                
    // sync_.reset(new sync( MySyncPolicy(10), subctrl, subgt));
    // sync_->registerCallback(boost::bind(&msg_callback, _1, _2));   

    ros::Subscriber gt_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 1, &gt_callback);

    ros::Subscriber ctrl_sub = nh.subscribe<mavros_msgs::AttitudeTarget>
            ("/mavros/setpoint_raw/attitude", 1, &ctrl_msg_callback);

    std::string path;
    std::string filename;

//// led path
    nh.getParam("/data/log_path", path);
    nh.getParam("/data/filename", filename);

    log_file_1 = path + filename;

    std::cout<<filename<<std::endl;
    std::cout<<log_file_1<<std::endl;

    remove(log_file_1.c_str());

    std::ofstream save(log_file_1, std::ios::app);
    save<<filename<<std::endl;
    save<<"t,f,r,p,y,dr,dp,dy,gt_x,gt_y,gt_z,gt_r,gt_p,gt_y"<<std::endl;
    save.close();

    starting_time = ros::Time::now().toSec();                        

    ros::spin();
    return 0;
}