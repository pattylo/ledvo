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

#include "include/ledvo.h"

double ledvo::LedNodelet::get_reprojection_error(
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



void ledvo::LedNodelet::solve_pose_w_LED(cv::Mat& frame, cv::Mat depth)
{   
    
    if(!LED_tracker_initiated_or_tracked)
        initialization(frame, depth);
    else
        recursive_filtering(frame, depth);
}

void ledvo::LedNodelet::initialization(cv::Mat& frame, cv::Mat depth)
{
    std::vector<gtsam::Point2> pts_2d_detect = LED_extract_POI_alter(frame);

    if(pts_2d_detect.size() < 4)
    {
        std::cout<<"LESS THAN 4"<<std::endl;
        return;
    }
    
    std::vector<gtsam::Point3> pts_3d_detect = pointcloud_generate(pts_2d_detect, depth);

    if(process_landmarks(pts_2d_detect, pts_3d_detect, true))
        LED_tracker_initiated_or_tracked = true;
    
    return;
}


void ledvo::LedNodelet::recursive_filtering(cv::Mat& frame, cv::Mat depth)
{
    std::vector<gtsam::Point2> pts_2d_detect;
    std::cout<<"gan"<<std::endl;
    pts_2d_detect = LED_extract_POI_alter(frame);
}


void ledvo::LedNodelet::solve_pnp_initial_pose(std::vector<Eigen::Vector2d> pts_2d, std::vector<Eigen::Vector3d> pts_3d)
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

    camMat.at<double>(0,0) = cameraMat(0,0);
    camMat.at<double>(0,2) = cameraMat(0,2);
    camMat.at<double>(1,1) = cameraMat(1,1);
    camMat.at<double>(1,2) = cameraMat(1,2);


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