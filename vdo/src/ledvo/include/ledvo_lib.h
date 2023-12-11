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
 * \file ledvo.h
 * \date 08/11/2022
 * \author pattylo
 * \copyright (c) RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
 */

#ifndef LED_H
#define LED_H

#include "essential.h"
#include "cameraModel.hpp"
#include "RosTopicConfigs.hpp"

#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <sophus/se3.hpp>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <pthread.h>
#include <tf/tf.h>

#include <gtsam/geometry/Point2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/ProjectionFactor.h>
#include <gtsam/nonlinear/FixedLagSmoother.h>
#include <gtsam/nonlinear/BatchFixedLagSmoother.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

#include "vdo/ledvo_log.h"


#include <visualization_msgs/Marker.h>

#include "torch/torch.h"


// map definition for convinience
#define COLOR_SUB_TOPIC CAMERA_SUB_TOPIC_A
#define DEPTH_SUB_TOPIC CAMERA_SUB_TOPIC_B
#define UAV_POSE_SUB_TOPIC POSE_SUB_TOPIC_A
#define UGV_POSE_SUB_TOPIC POSE_SUB_TOPIC_B

#define LED_POSE_PUB_TOPIC POSE_PUB_TOPIC_A
#define UGV_POSE_PUB_TOPIC POSE_PUB_TOPIC_B
#define UAV_POSE_PUB_TOPIC POSE_PUB_TOPIC_C
#define CAM_POSE_PUB_TOPIC POSE_PUB_TOPIC_D

#define LED_ODOM_PUB_TOPIC ODOM_PUB_TOPIC_A

namespace correspondence
{
    typedef struct matchid
        {
            int detected_indices; //
            bool detected_ornot = false;
            Eigen::Vector3d pts_3d_correspond;
            Eigen::Vector2d pts_2d_correspond;
        }matchid;
}

namespace ledvo
{
    typedef struct landmark
    {
        bool tracking = false;
        int tracked_no = 0;
        gtsam::Point3 pt3d;
        gtsam::Point2 pt2d_previous;
    }landmark;

    class LedvoLib : public vision::cameraModel
    {
        public:
        LedvoLib(ros::NodeHandle& nh);
        ~LedvoLib(){}

        private:
            ros::NodeHandle _nh;

            torch::Tensor tensor = torch::rand({2, 3});
        //primary objects
            //frames
            cv::Mat frame, display, hsv, frame_temp;
            cv::Mat frame_input;
            cv::Mat im_with_keypoints;
            cv::Mat frame_initial_thresholded;
            
            //time related
            std_msgs::Header led_pose_header, led_pose_header_previous;
            double last_request = 0;

            //camera related
            // Eigen::MatrixXd cameraMat = Eigen::MatrixXd::Zero(3,3);
            Eigen::VectorXd cameraEX;

            //LED config and correspondences
            std::vector<Eigen::Vector3d> pts_on_body_frame;

            // detect_no, correspondences(in order of 0->5)
            std::tuple<int, std::vector<correspondence::matchid>> corres_global_current;
            std::tuple<int, std::vector<correspondence::matchid>> corres_global_previous;

            Eigen::VectorXd LEDEX;

            //poses
            Sophus::SE3d pose_global_sophus;
            Sophus::SE3d pose_epnp_sophus, pose_depth_sophus;
            
            Sophus::SE3d pose_cam_inWorld_SE3;
            Sophus::SE3d pose_ugv_inWorld_SE3;
            Sophus::SE3d pose_uav_inWorld_SE3;
            Sophus::SE3d pose_led_inWorld_SE3;

            Sophus::SE3d pose_cam_inGeneralBodySE3;
            Sophus::SE3d pose_cam_inUgvBody_SE3;
            Sophus::SE3d pose_led_inUavBodyOffset_SE3;

            geometry_msgs::PoseStamped ugv_pose_msg, 
                                       uav_pose_msg,
                                       uav_stpt_msg;            
                       
        //secondary objects
            // double temp = 0;
            int i = 0;
            bool nodelet_activated = false;
            bool tracker_started = false;
            
            std::vector<cv::KeyPoint> blobs_for_initialize;
            int _width = 0, _height = 0;


        //subscribe                                    
            //objects
            message_filters::Subscriber<sensor_msgs::CompressedImage> subimage;
            message_filters::Subscriber<sensor_msgs::Image> subdepth;
            typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::Image> MySyncPolicy;
            typedef message_filters::Synchronizer<MySyncPolicy> sync;// (MySyncPolicy(10), subimage, subdepth);
            boost::shared_ptr<sync> sync_;                    
            ros::Subscriber ugv_pose_sub, uav_pose_sub;
            ros::Subscriber uav_setpt_sub;
            //functions
            void camera_callback(const sensor_msgs::CompressedImage::ConstPtr & rgbimage, const sensor_msgs::Image::ConstPtr & depth);            
            void ugv_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void uav_pose_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);
            void uav_setpt_callback(const geometry_msgs::PoseStamped::ConstPtr& pose);

            void uav_ctrl_msg_callback(const int lala);
        //publisher 
            //objects
            ros::Publisher ledpose_pub, ledodom_pub, 
                           campose_pub, ugvpose_pub, uavpose_pub,
                           record_led_pub, record_uav_pub;
            ros::Publisher lm_pub;
            image_transport::Publisher pubimage;
            image_transport::Publisher pubimage_input;
            //functions
        
        // ledvo
            void solve_pose_w_LED(cv::Mat& frame, cv::Mat depth);
            void initialization(cv::Mat& frame, cv::Mat depth);
            void recursive_filtering(cv::Mat& frame, cv::Mat depth);   
            std::vector<gtsam::Point2> LED_extract_POI_alter(cv::Mat& frame);
            std::vector<gtsam::Point3> pointcloud_generate(
                std::vector<gtsam::Point2>& pts_2d_detected, 
                cv::Mat depthimage
            );

            // batch fixedlag smoother
            gtsam::LevenbergMarquardtParams batchLMparameters;
            gtsam::NonlinearFactorGraph newFactors;
            gtsam::Values newValues;
            gtsam::Values landmarkValues;
            gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps;
        
            double batchLag = 4.0;
            // gtsam::BatchFixedLagSmoother batchsmoother(batchLag, batchLMparameters);
            std::unique_ptr<gtsam::BatchFixedLagSmoother> batchsmoother;

            gtsam::Values registeredLandmarks;
            bool firstFrame = true;
            int landmarkInitialTrackingCount = 0;
            gtsam::Pose3 x_current;
            gtsam::Pose3 x_previous;

            double timeStart;
            gtsam::Cal3_S2::shared_ptr K;

            int k = 0; // for x
            int m = 0; // for lm


            visualization_msgs::Marker traj_points;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
                
        //LED extraction tool
            //objects
            double LANDING_DISTANCE = 0;
            int BINARY_THRES = 0;

            std::vector<Eigen::Vector3d> pts_on_body_frame_in_corres_order;
            std::vector<Eigen::Vector2d> pts_detected_in_corres_order;

            

            bool process_landmarks(
                std::vector<gtsam::Point2> pts_2d_detect,
                std::vector<gtsam::Point3> pts_3d_detect,
                bool initialing
            );
            
            std::vector<Eigen::Vector2d> LED_extract_POI(cv::Mat& frame, cv::Mat depth);
            
            
        // correspondence search
            //objects
            int LED_no;
            int LED_r_no;
            int LED_g_no;
            // int last_frame_no;
            // int current_frame_no;
            std::vector<Eigen::Vector2d> pts_2d_detect_correct_order;
            cv::Point3f pcl_center_point_wo_outlier_previous;
            Eigen::Vector3d led_3d_posi_in_camera_frame_depth;

            //functions    
            void get_correspondence(
                std::vector<Eigen::Vector2d>& pts_2d_detected
            );
            std::vector<Eigen::Vector2d> shift2D(
                std::vector<Eigen::Vector2d>& pts_2D_previous,
                std::vector<Eigen::Vector2d>& pts_detect_current
            );
            void correspondence_search_2D2DCompare(
                std::vector<Eigen::Vector2d>& pts_2d_detected,
                std::vector<Eigen::Vector2d>& pts_2d_detected_previous
            );
                                  
        // initialization
            //objects
            bool LED_tracker_initiated_or_tracked = false;
            double MAD_dilate, MAD_max;
            double MAD_x_threshold = 0, MAD_y_threshold = 0, MAD_z_threshold = 0;
            double min_blob_size = 0;
            //functions
            
            void solve_pnp_initial_pose(
                std::vector<Eigen::Vector2d> pts_2d, 
                std::vector<Eigen::Vector3d> body_frame_pts
            );
            double calculate_MAD(std::vector<double> norm_of_points);
                    
        // publish
            //objects
            int error_no = 0;
            int total_no = 0;
            int detect_no = 0;
            double BA_error = 0;
            double depth_avg_of_all = 0;

            geometry_msgs::PoseStamped led_pose_estimated_msg;
            geometry_msgs::TwistStamped led_twist_estimated;
            nav_msgs::Odometry led_odom_estimated;            
            // functions
            void map_SE3_to_pose(Sophus::SE3d pose);                        
            double get_reprojection_error(
                std::vector<Eigen::Vector3d> pts_3d, 
                std::vector<Eigen::Vector2d> pts_2d, 
                Sophus::SE3d pose, 
                bool draw_reproject
            ) override;
               

            void set_image_to_publish(
                double hz, 
                const sensor_msgs::CompressedImageConstPtr & rgbmsg
            );
            void terminal_msg_display(double hz);
            void log(double ms);    
            
            Sophus::SE3d posemsg_to_SE3(const geometry_msgs::PoseStamped pose);
            geometry_msgs::PoseStamped SE3_to_posemsg(const Sophus::SE3d pose_on_SE3, const std_msgs::Header msgHeader);
            
            Eigen::VectorXd led_twist_current;    
            
            // configs.cpp
            void doALOTofConfigs(ros::NodeHandle& nh);

            void registerRosCommunicate(ros::NodeHandle& nh);
            void POI_config(ros::NodeHandle& nh);
            void camIntrinsic_config(ros::NodeHandle& nh);
            void camExtrinsic_config(ros::NodeHandle& nh);         
            void LEDExtrinsicUAV_config(ros::NodeHandle& nh);         
            void CamInGeneralBody_config(ros::NodeHandle& nh);            
            void LEDInBodyAndOutlierSetting_config(ros::NodeHandle& nh);
    };
}

#endif