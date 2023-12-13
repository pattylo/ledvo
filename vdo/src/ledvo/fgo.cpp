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
 * \file fgo.cpp
 * \date 08/11/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
 */

#include "include/ledvo_lib.h"

void ledvo::LedvoLib::solve_pose_w_LED(cv::Mat& frame, cv::Mat depth)
{   
    
    if(!LED_tracker_initiated_or_tracked)
    {
        initialization(frame, depth);
        // cout<<"end initialization"<<endl;
    }
    else
    {
        recursive_filtering(frame, depth);
    }
        
}

void ledvo::LedvoLib::initialization(cv::Mat& frame, cv::Mat depth)
{
    std::vector<gtsam::Point2> pts_2d_detect = LED_extract_POI_alter(frame);

    if(pts_2d_detect.size() < 4)
    {
        ROS_RED_STREAM("LESS THAN 4!!!!");
        return;
    }

    if(process_landmarks(pts_2d_detect, depth, true))
    {
        LED_tracker_initiated_or_tracked = true;
        ROS_CYAN_STREAM("INITIALIZED!");
        key_frame_manager(true);
    }
}

void ledvo::LedvoLib::recursive_filtering(cv::Mat& frame, cv::Mat depth)
{
    std::vector<gtsam::Point2> pts_2d_detect;

    pts_2d_detect = LED_extract_POI_alter(frame);
    set_correspondence_alter(pts_2d_detect);
    key_frame_manager(false);


    cv::imshow("test", im_with_keypoints);
    cv::waitKey(4);
}

void ledvo::LedvoLib::key_frame_manager(
    bool initializing
)
{
    using namespace gtsam;

    if(initializing)
    // at initialization
    {
        for(int i = 0; i < lm_dict.size(); i++)
        {
            add_visual_factor_wrapper(
                Symbol('x', keyframe_k),
                Pose3(pose_cam_inWorld_SE3.matrix()),
                lm_dict[i].pt2d,
                noiseModel::Isotropic::Sigma(2, 0.01),

                Symbol('l', i),
                lm_dict[i].pt3d
            );
            add_prior_factor_wrapper(
                Symbol('l', i),
                lm_dict[i].pt3d,
                noiseModel::Isotropic::Sigma(3, 0.1)
            );
        }

        add_prior_factor_wrapper(
            Symbol('x', keyframe_k),
            Pose3(pose_cam_inWorld_SE3.matrix()),
            noiseModel::Diagonal::Sigmas(
                (
                    Vector(6) << 
                        Vector3::Constant(0.3),Vector3::Constant(0.1)
                ).finished()
            )
        );

        return;
    }

    if(!select_keyframe())
        return;

    for(int i = 0; i < lm_dict.size(); i++)
    {
        if(lm_dict[i].tracking)
            continue;

        add_visual_factor_wrapper(
            Symbol('x', keyframe_k),
            Pose3(pose_cam_inWorld_SE3.matrix()),
            lm_dict[i].pt2d,
            noiseModel::Isotropic::Sigma(2, 0.01),

            Symbol('l', i),
            lm_dict[i].pt3d
        );
    }

    // // cout<<newValues.size
    // FixedLagSmoother::Result lala = batchsmoother->update(
    //     newFactors,
    //     newValues,
    //     newTimestamps
    // );
    cout<<keyframe_k<<endl;

    keyframe_k++;
}

bool ledvo::LedvoLib::select_keyframe()
{
    Eigen::Vector2d reproject;
    Sophus::SE3d pose_reproject = Sophus::SE3d(pose_cam_inWorld_SE3.inverse());

    std::vector<cv::Point> contour_temp;

    for(int i = 0; i < lm_dict.size(); i++)
    {
        reproject = reproject_3D_2D(lm_dict[i].pt3d, pose_reproject);
        cv::circle(im_with_keypoints, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);
        
        contour_temp.emplace_back(cv::Point(reproject(0), reproject(1)));
    }

    contour_current.clear();
    contour_current.emplace_back(contour_temp);

    std::vector< std::vector<cv::Point>> hull(contour_current.size());
    cv::convexHull(contour_current[0], hull[0]);
    
    contour_current = hull;

    if(!key_first_generate)
    {
        ROS_YELLOW_STREAM("FIRST KEY FRAME!");
        contour_previous = contour_current;

        key_first_generate = true;
        key_frame_last_request = ros::Time::now().toSec();

        // add_factor_wrapper();

        return true;
    }

    // Create binary masks for each contour
    cv::Mat mask_current = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Mat mask_previous = cv::Mat::zeros(frame.size(), CV_8UC1);

    cv::drawContours(mask_current, contour_current, -1, cv::Scalar(255), cv::FILLED);
    // cv::drawContours(im_with_keypoints, contour_current, 0, CV_RGB(0,255,0), cv::FILLED);

    cv::drawContours(mask_previous, contour_previous, -1, cv::Scalar(255), cv::FILLED);
    // cv::drawContours(im_with_keypoints, contour_previous, 0, cv::Scalar(255), cv::FILLED);

    // Calculate the overlapping area
    cv::Mat overlappingArea;
    cv::bitwise_and(mask_current, mask_previous, overlappingArea);

    double overlappingAreaValue = cv::countNonZero(overlappingArea);
    double previous_area = cv::contourArea(contour_previous[0]);

    double overlappingRate = overlappingAreaValue / previous_area;

    if (
        overlappingRate < 0.64 || 
        ros::Time::now().toSec() - key_frame_last_request > ros::Duration(2.0).toSec()
    )
    {
        contour_previous = contour_current;
        ROS_YELLOW_STREAM("NEW KEY FRAME!");
        key_frame_last_request = ros::Time::now().toSec();

        if(overlappingRate < 0.64)
        {
            ROS_RED_STREAM("KF becos of OVERLAPPING RATE!");
        }

        return true;
    }
    else 
        return false;

    // cout<<overlappingAreaValue / previous_area * 100 << " %\n"<<endl;
}


void ledvo::LedvoLib::add_prior_factor_wrapper(
    gtsam::Symbol factor_id,
    gtsam::Pose3 pose,
    gtsam::noiseModel::Diagonal::shared_ptr poseNoise
)
{
    // noiseModel::Diagonal::shared_ptr poseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << Vector3::Constant(0.3),Vector3::Constant(0.1)).finished()); 
    // 30cm std on x,y,z 0.1 rad on roll,pitch,yaw
    
    newFactors.addPrior(factor_id, pose, poseNoise);
    // newValues.insert(Symbol('x', 0), poses[0].compose(Pose3(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20))));
    newTimestamps[factor_id] = ros::Time::now().toSec() - starting_time;

}

void ledvo::LedvoLib::add_prior_factor_wrapper(
    gtsam::Symbol factor_id,
    gtsam::Point3 point,
    gtsam::noiseModel::Isotropic::shared_ptr pointNoise
)
{
    // noiseModel::Isotropic::shared_ptr pointNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    newFactors.addPrior(factor_id, point, pointNoise);
    // newValues.insert<Point3>(Symbol('l',0), points[0] + Point3(-0.25, 0.20, 0.15));
    newTimestamps[factor_id] = ros::Time::now().toSec() - starting_time;0.0;
}

void ledvo::LedvoLib::add_visual_factor_wrapper(
    gtsam::Symbol node_pose_id,
    gtsam::Pose3 node_pose,
    gtsam::Point2 pt_2d_measured,
    gtsam::noiseModel::Isotropic::shared_ptr measurementNoise,

    gtsam::Symbol node_lm_id,
    gtsam::Point3 node_lm_point
)
{
    using namespace gtsam;
    PinholeCamera<Cal3_S2> camera(node_pose, *K);
    // this will be the points that

    newFactors.push_back(
        GenericProjectionFactor<Pose3, Point3, Cal3_S2>(
            pt_2d_measured, 
            measurementNoise, 
            node_pose_id, 
            node_lm_id, 
            K
        )
    );

    if(!newValues.exists(node_pose_id))
    {
        newValues.insert(
            node_pose_id, //Symbol('x', k), 
            node_pose
        );
    }

    if(!landmarkValues.exists(node_lm_id))
    {
        landmarkValues.insert<Point3>(
            node_lm_id,
            node_lm_point
        );

        newValues.insert<Point3>(
            node_lm_id,
            node_lm_point
        );
    }

    newTimestamps[node_pose_id] = ros::Time::now().toSec() - starting_time;
    newTimestamps[node_lm_id] = ros::Time::now().toSec() - starting_time;
}
























































































void ledvo::LedvoLib::solve_pnp_initial_pose(std::vector<Eigen::Vector2d> pts_2d, std::vector<Eigen::Vector3d> pts_3d)
{
    Eigen::Matrix3d R;
    Eigen::Vector3d t;

    cv::Mat distCoeffs = cv::Mat::zeros(5, 1, CV_64F);
    // distCoeffs.at<double>(0) = -0.056986890733242035;
    // distCoeffs.at<double>(1) = 0.06356718391180038;
    // distCoeffs.at<double>(2) = -0.0012483829632401466;
    // distCoeffs.at<double>(3) = -0.00018130485841538757;
    // distCoeffs.at<double>(4) = -0.019809694960713387;

    cv::Mat no_ro_rmat = cv::Mat::eye(3,3,CV_64F);
    
    cv::Vec3d rvec, tvec;
    // cv::Rodrigues(no_ro_rmat, rvec);

    cv::Mat camMat = cv::Mat::eye(3,3,CV_64F);
    std::vector<cv::Point3f> pts_3d_;
    std::vector<cv::Point2f> pts_2d_;

    cv::Point3f temp3d;
    cv::Point2f temp2d;

    for(auto what : pts_3d)
    {
        temp3d.x = what(0);
        temp3d.y = what(1);
        temp3d.z = what(2);

        pts_3d_.push_back(temp3d);
    }

    for(auto what : pts_2d)
    {
        temp2d.x = what(0);
        temp2d.y = what(1);    

        pts_2d_.push_back(temp2d);
    }

    // camMat.at<double>(0,0) = cameraMat(0,0);
    // camMat.at<double>(0,2) = cameraMat(0,2);
    // camMat.at<double>(1,1) = cameraMat(1,1);
    // camMat.at<double>(1,2) = cameraMat(1,2);


    // cv::solvePnP(pts_3d_, pts_2d_ ,camMat, distCoeffs, rvec, tvec, cv::SOLVEPNP_EPNP);

    cv::solvePnP(pts_3d_, pts_2d_ ,camMat, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);
    //opt pnp algorithm
    //, cv::SOLVEPNP_EPNP
    //, cv::SOLVEPNP_IPPE
    //, cv::SOLVEPNP_P3P

    //return values
    cv::Mat rmat = cv::Mat::eye(3,3,CV_64F);
    cv::Rodrigues(rvec, rmat);

    R <<
        rmat.at<double>(0,0), rmat.at<double>(0,1), rmat.at<double>(0,2),
        rmat.at<double>(1,0), rmat.at<double>(1,1), rmat.at<double>(1,2),
        rmat.at<double>(2,0), rmat.at<double>(2,1), rmat.at<double>(2,2);

    Eigen::Matrix3d reverse_mat;
    reverse_mat <<
            1.0000000,  0.0000000,  0.0000000,
            0.0000000, -1.0000000, -0.0000000,
            0.0000000,  0.0000000, -1.0000000;

    

    t =  Eigen::Vector3d(
          tvec(0),
          tvec(1),
          tvec(2)  
        );

    if(tvec(2) < 0) //sometimes opencv yeilds reversed results, flip it 
    {
        R = R * reverse_mat;
        t = (-1) * t;
    }

    pose_epnp_sophus = Sophus::SE3d(R, t);
    
    // pose_depth_sophus = Sophus::SE3d(R, t);

    // if(LED_tracker_initiated_or_tracked)
    // {
    //     // cout<<"depth"<<endl;
    //     t = led_3d_posi_in_camera_frame_depth;
    //     pose_depth_sophus = Sophus::SE3d(R, t);
    // }

}



double ledvo::LedvoLib::get_reprojection_error(
    std::vector<Eigen::Vector3d> pts_3d, 
    std::vector<Eigen::Vector2d> pts_2d, 
    Sophus::SE3d pose, 
    bool draw_reproject
)
{
    double e = 0;

    Eigen::Vector2d reproject, error;

    for(int i = 0; i < pts_3d.size(); i++)
    {
        reproject = reproject_3D_2D(pts_3d[i], pose);
        error = pts_2d[i] - reproject;
        e = e + error.norm();

        if(draw_reproject)
            cv::circle(display, cv::Point(reproject(0), reproject(1)), 2.5, CV_RGB(0,255,0),-1);
        
    }

    return e;
};