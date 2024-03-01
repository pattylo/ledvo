/*
    This file is part of LEDVO - the relative state estimation package for UAV-UGV

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
 * \file collect.cpp
 * \date 01/03/2024
 * \author pattylo
 * \copyright (c) RCUAS of Hong Kong Polytechnic University
 * \brief classes for fisheye-based relative localization for UAV and UGV based on LED markers
 */

#include "essential.h"
#include <opencv2/calib3d.hpp>

void fisheye_callback(const sensor_msgs::Image::ConstPtr& image)
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

    cv::Mat fisheye_frame = fisheye_ptr->image;

    cv::imshow("fisheye", fisheye_frame);
    cv::waitKey(10);

    cv::Mat undistorted_frame;
    
    // K
    cv::Mat K_matrix = (cv::Mat_<float>(3, 3) << 
            284.18060302734375, 0.0, 425.24481201171875, 
            0.0, 285.1946105957031, 398.6476135253906, 
            0.0, 0.0, 1.0
        );

    // D
    std::vector<float> D_vec = {-0.0003750045143533498, 0.029878979548811913, -0.029044020920991898, 0.003844168037176132};
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
    
    cv::imshow("un_fisheye", undistorted_frame);
    cv::waitKey(10);
}


int main(int argc, char** argv)
{
    return 0;
}