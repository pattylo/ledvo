/*
    This file is part of  LEDVO - the relative state estimation package for UAV-UGV

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
 * \file fisheye.cpp
 * \date 25/02/2024
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for fisheye-based relative localization for UAV and UGV based on LED markers
 */

#include "include/ledvo_lib.h"
#include <opencv2/calib3d.hpp>

void ledvo::LedvoLib::fisheye_callback(const sensor_msgs::Image::ConstPtr& image)
{
    // main process here:
    cv_bridge::CvImageConstPtr fisheye_ptr;
    try
    {
        fisheye_ptr  = cv_bridge::toCvCopy(image, image->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    fisheye_frame = fisheye_ptr->image;

    // cv::imshow("fisheye", fisheye_frame);
    // cv::waitKey(10);

    cv::Mat undistorted_frame;
    
    // K
    cv::Mat K_matrix = (cv::Mat_<float>(3, 3) << 
            285.15679931640625, 0.0, 428.5877990722656, 
            0.0, 286.1261901855469, 392.0649108886719, 
            0.0, 0.0, 1.0
        );

    // fisheye1 
            // 284.18060302734375, 0.0, 425.24481201171875, 
            // 0.0, 285.1946105957031, 398.6476135253906, 
            // 0.0, 0.0, 1.0

    // fisheye2
            // 285.15679931640625, 0.0, 428.5877990722656, 
            // 0.0, 286.1261901855469, 392.0649108886719, 
            // 0.0, 0.0, 1.0

    // D
    std::vector<float> D_vec = {-0.000838623323943466, 0.03629162162542343, -0.03442243114113808, 0.0054010930471122265};
    // fisheye1 -0.0003750045143533498, 0.029878979548811913, -0.029044020920991898, 0.003844168037176132
    // fisheye2 -0.000838623323943466, 0.03629162162542343, -0.03442243114113808, 0.0054010930471122265

    cv::Mat D_matrix(D_vec, true);

    cv::Mat R_matrix = cv::Mat::eye(3, 3, CV_64F);

    cv::Mat KNew_matrix;

    // cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
    //     K_matrix,
    //     D_matrix,
    //     fisheye_frame.size(),
    //     R_matrix,
    //     KNew_matrix,
    //     0.0,
    //     fisheye_frame.size(),
    //     1.0
    // );

    // std::cout<<KNew_matrix<<std::endl;

    cv::fisheye::undistortImage(
        fisheye_frame, 
        undistorted_frame, 
        K_matrix, 
        D_matrix,
        K_matrix,
        fisheye_frame.size()
    );
    

    std:vector<Eigen::Vector2d> pts_2d;
    pose_w_aruco_pnp(undistorted_frame);
    // std::cout << aruco_detect(fisheye_frame, pts_2d) << std::endl;


    cv::imshow("un_fisheye", undistorted_frame);
    
    // cv::imshow("un_fisheye", fisheye_frame);
    cv::waitKey(10);


}

bool ledvo::LedvoLib::aruco_detect(
    cv::Mat& frame, 
    std::vector<Eigen::Vector2d>& pts_2d
)
{
    cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);

    // fisheye1 
            // 284.18060302734375, 0.0, 425.24481201171875, 
            // 0.0, 285.1946105957031, 398.6476135253906, 
            // 0.0, 0.0, 1.0

    cameraMatrix.at<double>(0,0) = 284.18060302734375;
    cameraMatrix.at<double>(1,1) = 285.1946105957031;
    cameraMatrix.at<double>(0,2) = 425.24481201171875;
    cameraMatrix.at<double>(1,2) = 398.6476135253906;

    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    
    // distCoeffs.at<double>(0) = -0.0003750045143533498;
    // distCoeffs.at<double>(1) = 0.029878979548811913;
    // distCoeffs.at<double>(2) = -0.029044020920991898;
    // distCoeffs.at<double>(3) = 0.003844168037176132;

    std::vector<cv::Vec3d> rvecs, tvecs;

    std::vector<int> markerids;
    std::vector<std::vector<cv::Point2f>> markercorners, rejected;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::detectMarkers(frame, dictionary, markercorners, markerids, parameters, rejected);

    if(markercorners.size() != 0)
    {
        if(markercorners.size() == 1)
        {
            double markerlength = 0.045;

            cv::aruco::estimatePoseSingleMarkers(markercorners, 0.08, cameraMatrix, distCoeffs, rvecs, tvecs);
      
            cv::aruco::drawAxis(frame, cameraMatrix, distCoeffs, rvecs[0], tvecs[0], 0.1);
            cv::Mat rmat = cv::Mat::eye(3,3,CV_64F);
            cv::Rodrigues(rvecs[0], rmat);

            Eigen::Matrix3d R;
            Eigen::Vector3d t;

            Eigen::Matrix<double, 4, 4> cam_to_body;
            cam_to_body << 0,0,1,0,
                -1,0,0,0,
                0,-1,0,0,
                0,0,0,1;

            R <<
                rmat.at<double>(0,0), rmat.at<double>(0,1), rmat.at<double>(0,2),
                rmat.at<double>(1,0), rmat.at<double>(1,1), rmat.at<double>(1,2),
                rmat.at<double>(2,0), rmat.at<double>(2,1), rmat.at<double>(2,2);
            // std::cout<<"R_aruco:"<<std::endl;
            // std::cout<<R<<std::endl;

            t <<
                tvecs[0](0),
                tvecs[0](1),
                tvecs[0](2);
        }
        
        for(auto& what : markercorners)
        {
            if(what.size() != 4)
                continue;

            for(auto& that : what)
            {
                // std::cout<<that<<std::endl;
                cv::circle(frame, cv::Point(that.x, that.y), 4, CV_RGB(0,0,255),-1);
                Eigen::Vector2d result;
                result << 
                    that.x,
                    that.y;
                pts_2d.push_back(result);
            }

            // std::cout<<"end aruco_detect"<<std::endl;
            // std::cout<<"------"<<std::endl;

            if(markercorners.size() > 1)
                pts_2d.clear();
        }

        if(pts_2d.size() == 0)
            return false;
        else
            return true;                                     
    }
    else
        return false;
}
