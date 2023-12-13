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
 * \file config.cpp
 * \date 08/12/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
 */

#include "include/ledvo_lib.h"

void ledvo::LedvoLib::doALOTofConfigs(ros::NodeHandle& nh)
{
    POI_config(nh);
    camIntrinsic_config(nh);
    camExtrinsic_config(nh);
    CamInGeneralBody_config(nh);
    LEDInBodyAndOutlierSetting_config(nh);
    GTSAM_config();

    registerRosCommunicate(nh); 
}

void ledvo::LedvoLib::registerRosCommunicate(ros::NodeHandle& nh)
{
    RosTopicConfigs configs(nh, "/ledvo");
    std::cout<<"hi here in ros"<<std::endl;

    
    //subscribe                
    subimage.subscribe(nh, configs.getTopicName(COLOR_SUB_TOPIC), 1);                
    std::cout<<configs.getTopicName(COLOR_SUB_TOPIC)<<std::endl;

    subdepth.subscribe(nh, configs.getTopicName(DEPTH_SUB_TOPIC), 1);                
    sync_.reset(new sync( MySyncPolicy(10), subimage, subdepth));            
    sync_->registerCallback(boost::bind(&LedvoLib::camera_callback, this, _1, _2));                                

    uav_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
    //only used for validation stage
        (configs.getTopicName(UAV_POSE_SUB_TOPIC), 1, &LedvoLib::uav_pose_callback, this);

    ugv_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
        (configs.getTopicName(UGV_POSE_SUB_TOPIC), 1, &LedvoLib::ugv_pose_callback, this);

    uav_setpt_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("/planner_server/traj/pose", 1, &LedvoLib::uav_setpt_callback, this);

    uav_ctrlmsg_sub = nh.subscribe<mavros_msgs::AttitudeTarget>
        ("/mavros/setpoint_raw/attitude", 1, &LedvoLib::uav_ctrl_msg_callback, this);
    
    //publish
    image_transport::ImageTransport image_transport_(nh);

    pubimage = image_transport_.advertise("/processed_image",1);
    pubimage_input = image_transport_.advertise("/input_image", 1);

    
    ledpose_pub = nh.advertise<geometry_msgs::PoseStamped>
    //only used for validation stage
                    (configs.getTopicName(LED_POSE_PUB_TOPIC), 1, true);
    
    ledodom_pub = nh.advertise<nav_msgs::Odometry>
                    (configs.getTopicName(LED_ODOM_PUB_TOPIC),1 , true);
    
    ugvpose_pub = nh.advertise<geometry_msgs::PoseStamped>
    //only used for validation stage
                    (configs.getTopicName(UGV_POSE_PUB_TOPIC), 1, true); 
    
    
    uavpose_pub = nh.advertise<geometry_msgs::PoseStamped>
    //only used for validation stage
                    (configs.getTopicName(UAV_POSE_PUB_TOPIC), 1, true);
    
    campose_pub = nh.advertise<geometry_msgs::PoseStamped>
    //only used for validation stage
                    (configs.getTopicName(CAM_POSE_PUB_TOPIC), 1, true);    
    
    record_led_pub = nh.advertise<vdo::ledvo_log>
                    ("/vdo/led/led_log", 1);
    
    record_uav_pub = nh.advertise<vdo::ledvo_log>
                    ("/vdo/led/uav_log", 1);  

    lm_pub = nh.advertise<visualization_msgs::Marker>("/gt_points/traj", 1, true);
}

void ledvo::LedvoLib::POI_config(ros::NodeHandle &nh)
{
    // load POI_extract config
    nh.getParam("/ledvo_master/LANDING_DISTANCE", LANDING_DISTANCE);     
    nh.getParam("/ledvo_master/BINARY_threshold", BINARY_THRES);     
    nh.getParam("/ledvo_master/frame_width", _width);
    nh.getParam("/ledvo_master/frame_height", _height);

    // #define CAR_POSE_TOPIC POSE_SUB_TOPIC_A
    // std::cout<<CAR_POSE_TOPIC<<std::endl;
    // nh.getParam("/ledvo_master/CAR_POSE_TOPIC", CAR_POSE_TOPIC);
    // nh.getParam("/ledvo_master/UAV_POSE_TOPIC", UAV_POSE_TOPIC);
}

void ledvo::LedvoLib::camIntrinsic_config(ros::NodeHandle& nh)
{
    // load camera intrinsics
    XmlRpc::XmlRpcValue intrinsics_list;
    nh.getParam("/ledvo_master/cam_intrinsics_455", intrinsics_list);

    for(int i = 0; i < 3; i++)
        for(int j = 0; j < 3; j++)
        {
            std::ostringstream ostr;
            ostr << intrinsics_list[3 * i+ j];
            std::istringstream istr(ostr.str());
            istr >> cameraMat(i, j);
        }
    
    std::cout<<cameraMat<<std::endl;    
}

void ledvo::LedvoLib::camExtrinsic_config(ros::NodeHandle& nh)
{
    cameraEX.resize(6);
    XmlRpc::XmlRpcValue extrinsics_list;
    
    nh.getParam("/ledvo_master/cam_ugv_extrinsics", extrinsics_list);                
    
    for(int i = 0; i < 6; i++)
    {                    
        cameraEX(i) = extrinsics_list[i];                    
    }

    pose_cam_inUgvBody_SE3 = Sophus::SE3d(
        rpy2q(
            Eigen::Vector3d(
                cameraEX(3) / 180 * M_PI,
                cameraEX(4) / 180 * M_PI,
                cameraEX(5) / 180 * M_PI              
            )
        ),
        Eigen::Vector3d(
            cameraEX(0),
            cameraEX(1),
            cameraEX(2)
        )            
    );
}

void ledvo::LedvoLib::LEDExtrinsicUAV_config(ros::NodeHandle& nh)
{
    // load LED extrinsics
    LEDEX.resize(6);
    XmlRpc::XmlRpcValue extrinsics_list_led;

    nh.getParam("/ledvo_master/led_uav_extrinsics", extrinsics_list_led);

    for(int i = 0; i < 6; i++)
    {
        LEDEX(i) = extrinsics_list_led[i];
    }

    pose_led_inUavBodyOffset_SE3 = Sophus::SE3d(
        Eigen::Matrix3d::Identity(),
        Eigen::Vector3d(
            LEDEX(0),
            LEDEX(1),
            LEDEX(2)
        )
    );
}

void ledvo::LedvoLib::CamInGeneralBody_config(ros::NodeHandle& nh)
{
    // load cam in general body frame                
    Eigen::Matrix3d cam_to_body_rot;
    cam_to_body_rot << 
        0,0,1,
        -1,0,0,
        0,-1,0;

    pose_cam_inGeneralBodySE3 = Sophus::SE3d(
        cam_to_body_rot, 
        Eigen::Vector3d::Zero()
    );
}

void ledvo::LedvoLib::LEDInBodyAndOutlierSetting_config(ros::NodeHandle& nh)
{
    //load LED potisions in body frame
    XmlRpc::XmlRpcValue LED_list;
    nh.getParam("/ledvo_master/LED_positions", LED_list); 

    std::vector<double> norm_of_x_points, norm_of_y_points, norm_of_z_points;

    std::cout<<"\nPts on body frame (X Y Z):\n";
    for(int i = 0; i < LED_list.size(); i++)
    {
        Eigen::Vector3d temp(LED_list[i]["x"], LED_list[i]["y"], LED_list[i]["z"]);
        
        norm_of_x_points.push_back(temp.x());
        norm_of_y_points.push_back(temp.y());
        norm_of_z_points.push_back(temp.z());                    
        std::cout<<"-----"<<std::endl;
        std::cout<<temp.x()<<" "<<temp.y()<<" "<<temp.z()<<" "<<std::endl;                    
        pts_on_body_frame.push_back(temp);
    }   
    std::cout<<std::endl;

    LED_no = pts_on_body_frame.size();

    nh.getParam("/ledvo_master/LED_r_number", LED_r_no);
    nh.getParam("/ledvo_master/LED_g_number", LED_g_no);

    //load outlier rejection info
    nh.getParam("/ledvo_master/MAD_dilate", MAD_dilate);
    nh.getParam("/ledvo_master/MAD_max", MAD_max);


    LED_no = pts_on_body_frame.size();
}

void ledvo::LedvoLib::GTSAM_config()
{
    batchsmoother = std::make_unique<gtsam::BatchFixedLagSmoother>(
        batchLag,
        batchLMparameters
    );

    x_current = gtsam::Pose3().Identity();
    x_current.print();

    timeStart = ros::Time::now().toSec();
    
    K = std::make_shared<gtsam::Cal3_S2>(
        cameraMat(0,0),
        cameraMat(1,1),
        0.0,
        cameraMat(0,2),
        cameraMat(1,2)
    );

    std::cout<<cameraMat<<std::endl<<std::endl;;
    std::cout<<K->K()<<std::endl;

    traj_points.header.frame_id = "world";

    traj_points.ns = "GT_points";

    traj_points.id = 0;
    traj_points.action = visualization_msgs::Marker::ADD;
    traj_points.pose.orientation.w = 1.0;
    traj_points.type = visualization_msgs::Marker::SPHERE_LIST;
    traj_points.scale.x = traj_points.scale.y = traj_points.scale.z = 0.2;
    traj_points.color.a=1;
    traj_points.color.g=1;
    traj_points.color.r=0;
    traj_points.color.b=0;

}