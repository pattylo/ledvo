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
 * \file main_process.cpp
 * \date 08/11/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
 */

#include "include/ledvo_lib.h"

ledvo::LedvoLib::LedvoLib(ros::NodeHandle& nh)
{
    ROS_INFO("LED Nodelet Initiated...");

    doALOTofConfigs(nh);
}



void ledvo::LedvoLib::camera_callback(
    const sensor_msgs::CompressedImage::ConstPtr& rgbmsg, 
    const sensor_msgs::Image::ConstPtr& depthmsg
)
{
    // cout<<"camera"<<endl;

    cv_bridge::CvImageConstPtr depth_ptr;
    led_pose_header = rgbmsg->header;

    try
    {
        depth_ptr  = cv_bridge::toCvCopy(depthmsg, depthmsg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth = depth_ptr->image;

    try
    {
        frame = cv::imdecode(cv::Mat(rgbmsg->data), 1);
        display = frame.clone();
        hsv = frame.clone();
        frame_temp = frame.clone();
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    double tick = ros::Time::now().toSec(); 

    // if(tick - last_request > ros::Duration(0.1).toSec() && nodelet_activated)
    // {
    //     LED_tracker_initiated_or_tracked = false;
    //     printf("\033c");
    //     ROS_RED_STREAM("RESET TERMINAL!");
    // }           

    // if(keyframe_k < 100) 
    solve_pose_w_LED(frame, depth);


    
    double tock = ros::Time::now().toSec();  

    // cout<<1 / (tock - tick)<<endl;
    if(1 / (tock - tick) < 30)
    {
        cout<<1 / (tock - tick)<<endl;
    }

    // if(tracker_started)
    //     total_no++;

    // terminal_msg_display(1 / (tock - tick));

    // set_image_to_publish(1 / (tock - tick), rgbmsg);

    // if(LED_tracker_initiated_or_tracked)
    //     log(tock - tick);
                    
    // last_request = ros::Time::now().toSec();

    // if(!nodelet_activated)
    //     nodelet_activated = true;

    // led_pose_header_previous = led_pose_header;

} 

Sophus::SE3d ledvo::LedvoLib::posemsg_to_SE3(const geometry_msgs::PoseStamped pose)
{
    return Sophus::SE3d(
        Eigen::Quaterniond(
            pose.pose.orientation.w,
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z  
        ).normalized().toRotationMatrix(),
        Eigen::Translation3d(
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z
        ).translation()
    );
}

geometry_msgs::PoseStamped ledvo::LedvoLib::SE3_to_posemsg(
    const Sophus::SE3d pose_on_SE3, 
    const std_msgs::Header msgHeader
)
{
    geometry_msgs::PoseStamped returnPoseMsg;
    returnPoseMsg.header = msgHeader;

    returnPoseMsg.pose.position.x = pose_on_SE3.translation().x();
    returnPoseMsg.pose.position.y = pose_on_SE3.translation().y();
    returnPoseMsg.pose.position.z = pose_on_SE3.translation().z();

    returnPoseMsg.pose.orientation.w = pose_on_SE3.unit_quaternion().w();
    returnPoseMsg.pose.orientation.x = pose_on_SE3.unit_quaternion().x();
    returnPoseMsg.pose.orientation.y = pose_on_SE3.unit_quaternion().y();
    returnPoseMsg.pose.orientation.z = pose_on_SE3.unit_quaternion().z();

    return returnPoseMsg;
}

void ledvo::LedvoLib::map_SE3_to_pose(Sophus::SE3d pose_led_inCamera_SE3)
{   
    pose_cam_inGeneralBodySE3 * pose_led_inCamera_SE3; //now we in body frame

    pose_led_inWorld_SE3 = 
        pose_cam_inWorld_SE3 
        * pose_led_inUavBodyOffset_SE3 
        * pose_cam_inGeneralBodySE3 
        * pose_led_inCamera_SE3;
    
    led_pose_header.frame_id = "world";
    led_pose_estimated_msg = SE3_to_posemsg(pose_led_inWorld_SE3, led_pose_header);

    ledpose_pub.publish(led_pose_estimated_msg);
}

void ledvo::LedvoLib::ugv_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    pose_cam_inWorld_SE3 = pose_ugv_inWorld_SE3 * pose_cam_inUgvBody_SE3;

    geometry_msgs::PoseStamped cam_pose_msg = SE3_to_posemsg(
        pose_cam_inWorld_SE3, 
        pose->header
    );

    campose_pub.publish(cam_pose_msg);

    pose_ugv_inWorld_SE3 = posemsg_to_SE3(*pose); 
    
    ugv_pose_msg = *pose;
    ugv_pose_msg.header.frame_id = "world";
    ugvpose_pub.publish(ugv_pose_msg);
}

void ledvo::LedvoLib::uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    pose_uav_inWorld_SE3 = posemsg_to_SE3(*pose);

    pose_cam_inWorld_SE3 = Sophus::SE3d(
        pose_uav_inWorld_SE3.matrix() * pose_cam_inGeneralBodySE3.matrix()
    );
    
    uav_pose_msg = *pose;
    uav_pose_msg.header.frame_id = "world";
    uavpose_pub.publish(uav_pose_msg);


    geometry_msgs::PoseStamped cam_pose_msg = SE3_to_posemsg(
        pose_cam_inWorld_SE3, 
        pose->header
    );

    campose_pub.publish(cam_pose_msg);    
}

void ledvo::LedvoLib::uav_setpt_callback(const geometry_msgs::PoseStamped::ConstPtr& pose)
{
    uav_stpt_msg = *pose;
}

void ledvo::LedvoLib::calculate_msg_callback(const std_msgs::Bool::ConstPtr& msg)
{
    ROS_RED_STREAM("CALCULATE!");

    // ROS_BLUE_STREAM("NEW VALUES");
    // newFactors.print();
    // ROS_BLUE_STREAM("NEW VALUES");
    // newValues.print();

    cout<<endl<<endl<<endl<<endl;
    ROS_RED_STREAM("HERERERERE");
    cout<<"ERROR BEFORE HERE===> "<<newFactors.error(newValues)<<endl;;
    ROS_RED_STREAM("HERERERERE");

    using namespace gtsam;
    Values result = LevenbergMarquardtOptimizer(
        newFactors, 
        newValues, 
        batchLMparameters).optimize();

    double error_lala = 0;

    cout<<"ERROR AFTER HERE===> "<< sqrt(newFactors.error(result) / (keyframe_k + 1 + lm_dict.size())) <<endl;;

    cout<<endl<<endl;
    result.print();


    Pose3 xValue = result.at<Pose3>(Symbol('x',0));
    cout<<xValue<<endl;

    double error_ = 0;

    for (int i = 0; i < keyframe_ground_truth.size(); i++)
    {
        Pose3 x_se3_temp = result.at<Pose3>(Symbol('x',i));
        Sophus::SE3d x_gt = posemsg_to_SE3(keyframe_ground_truth[i]);

        error_ = error_ + pow((x_se3_temp.translation() - x_gt.translation()).norm(),2);
    }

    ROS_RED_STREAM("FINAL ERROR!");
    cout<<sqrt(error_ / keyframe_ground_truth.size())<<endl;


    // gtsam::FixedLagSmoother::Result lala = batchsmoother->update(
    //     newFactors,
    //     newValues,
    //     newTimestamps
    // );

    // lala.print();
    // batchsmoother->calculateEstimate().print();
    // batchsmoother->optimize().print();
    // batchsmoother->
    // batchsmoother->getFactors().print();
    // batchsmoother->

    // std::cout<<"current batch size: "<<batchsmoother->calculateEstimate()<<std::endl;

    // Single Step Optimization using Levenberg-Marquardt
    
    // result.print();


    ROS_YELLOW_STREAM("IT IS ALL YELLOW!");
}


void ledvo::LedvoLib::set_image_to_publish(double freq, const sensor_msgs::CompressedImageConstPtr & rgbmsg)
{    
    char hz[40];
    char fps[10] = " fps";
    sprintf(hz, "%.2f", freq);
    strcat(hz, fps);

    char BA[40] = "BA: ";
    char BA_error_display[10];
    sprintf(BA_error_display, "%.2f", BA_error);
    strcat(BA, BA_error_display);

    char depth[40] = "DPTH: ";
    char depth_display[10];
    sprintf(depth_display, "%.2f", depth_avg_of_all);
    strcat(depth, depth_display);
    
    cv::putText(display, hz, cv::Point(20,40), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));  
    cv::putText(display, std::to_string(detect_no), cv::Point(720,460), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
    cv::putText(display, BA, cv::Point(720,60), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));
    cv::putText(display, depth, cv::Point(20,460), cv::FONT_HERSHEY_PLAIN, 1.6, CV_RGB(255,0,0));

    cv::Mat imageoutput = display.clone();
    cv_bridge::CvImage for_visual;
    for_visual.header = rgbmsg->header;
    for_visual.encoding = sensor_msgs::image_encodings::BGR8;
    for_visual.image = imageoutput;
    this->pubimage.publish(for_visual.toImageMsg());


    cv_bridge::CvImage for_visual_input;
    for_visual_input.header = rgbmsg->header;
    for_visual_input.encoding = sensor_msgs::image_encodings::BGR8;
    for_visual_input.image = frame_input;
    this->pubimage_input.publish(for_visual_input.toImageMsg());   

}

void ledvo::LedvoLib::log(double ms)
{
    vdo::ledvo_log logdata_entry_led;
    
    logdata_entry_led.px = pose_led_inWorld_SE3.translation().x();
    logdata_entry_led.py = pose_led_inWorld_SE3.translation().y();
    logdata_entry_led.pz = pose_led_inWorld_SE3.translation().z();

    logdata_entry_led.set_px = uav_stpt_msg.pose.position.x;
    logdata_entry_led.set_py = uav_stpt_msg.pose.position.y;
    logdata_entry_led.set_pz = uav_stpt_msg.pose.position.z;
    
    Eigen::Vector3d rpy = q2rpy(
        Eigen::Quaterniond(pose_led_inWorld_SE3.rotationMatrix())
    );

    logdata_entry_led.roll  = rpy(0);
    logdata_entry_led.pitch = rpy(1);
    logdata_entry_led.yaw   = rpy(2);

    Eigen::AngleAxisd angle_axis_led = Eigen::AngleAxisd(pose_led_inWorld_SE3.rotationMatrix());
    logdata_entry_led.orientation = angle_axis_led.angle();

    logdata_entry_led.ms = ms;
    logdata_entry_led.depth = abs(
        sqrt(
            pow(
                pose_cam_inWorld_SE3.translation().x() - pose_uav_inWorld_SE3.translation().x(),
                2
            ) +
            pow(pose_cam_inWorld_SE3.translation().y() - pose_uav_inWorld_SE3.translation().y(),
                2
            ) +
            pow(pose_cam_inWorld_SE3.translation().z() - pose_uav_inWorld_SE3.translation().z(),
                2
            )
        )        
    );

    logdata_entry_led.header.stamp = led_pose_header.stamp;

    record_led_pub.publish(logdata_entry_led);

    ///////////////////////////////////////////////////////////

    vdo::ledvo_log logdata_entry_uav;
    
    logdata_entry_uav.px = pose_uav_inWorld_SE3.translation().x();
    logdata_entry_uav.py = pose_uav_inWorld_SE3.translation().y();
    logdata_entry_uav.pz = pose_uav_inWorld_SE3.translation().z();
    
    rpy = q2rpy(
        Eigen::Quaterniond(pose_uav_inWorld_SE3.rotationMatrix())
    );

    logdata_entry_uav.roll  = rpy(0);
    logdata_entry_uav.pitch = rpy(1);
    logdata_entry_uav.yaw   = rpy(2);

    Eigen::AngleAxisd angle_axis_uav = Eigen::AngleAxisd(pose_uav_inWorld_SE3.rotationMatrix());
    logdata_entry_uav.orientation = angle_axis_uav.angle();

    logdata_entry_uav.header.stamp = led_pose_header.stamp;

    record_uav_pub.publish(logdata_entry_uav);

    // std::cout<<"orientation: "<<abs(logdata_entry_led.orientation - logdata_entry_uav.orientation)<<std::endl;
    // std::ofstream save("/home/patty/ledvo_ws/src/ledvo/vdo/src/temp/lala.txt", std::ios::app);
    // save<<abs(logdata_entry_led.orientation - logdata_entry_uav.orientation)<<std::endl;
}

void ledvo::LedvoLib::terminal_msg_display(double hz)
{
    std::string LED_terminal_display = "DETECT_no: " + std::to_string(detect_no);

    std::ostringstream out1;
    out1.precision(2);
    out1<<std::fixed<<BA_error;
    std::string BA_terminal_display = " || BA_ERROR: " + out1.str();

    std::ostringstream out2;
    out2.precision(2);
    out2<<std::fixed<<depth_avg_of_all;
    std::string depth_terminal_display = " || depth: " + out2.str();

    std::ostringstream out3;
    out3.precision(2);
    out3<<std::fixed<<hz;
    std::string hz_terminal_display = " || hz: " + out3.str();

    std::string final_msg = LED_terminal_display 
        + BA_terminal_display 
        + depth_terminal_display
        + hz_terminal_display;

    std::string LED_tracker_status_display;

    if(LED_tracker_initiated_or_tracked)
    {
        final_msg = "LED GOOD || " + final_msg;
        ROS_GREEN_STREAM(final_msg);
    }
    else
    {
        final_msg = "LED BAD! || " + final_msg;
        ROS_RED_STREAM(final_msg);
        if(tracker_started)
            error_no ++;
    }
    std::cout<<"fail: "<< error_no<<" / "<<total_no<<std::endl;
}

