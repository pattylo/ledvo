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
 * \file dynamics.cpp
 * \date 08/12/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
*/

#include "include/ledvo_lib.h"

void ledvo::LedvoLib::uav_ctrl_msg_callback(const mavros_msgs::AttitudeTarget::ConstPtr &msg)
{
    // cout<<"ctrl"<<endl;
    uav_ctrl_msg = *msg;

    if(keyframe_k >= 0)
        stack_ctrl_msg_buffer(uav_ctrl_msg);

    std_msgs::Bool dummy;
    dummy_msg_pub.publish(dummy);
}

void ledvo::LedvoLib::dummy_callback(const std_msgs::Bool::ConstPtr& msg)
{
    // ROS_BLUE_STREAM("dummy");

    // double tic = ros::Time::now().toSec();
    // at::Tensor output = module.forward({input_data}).toTensor();
    // double tac = ros::Time::now().toSec();

    // std::cout << output << std::endl<<std::endl;;
    // std::cout << "INFERENCE TIME: " << tac - tic << std::endl;
}

void ledvo::LedvoLib::stack_ctrl_msg_buffer(mavros_msgs::AttitudeTarget ctrl_msg)
{
    Eigen::Vector3d ctrl_rpy = q2rpy(
        Eigen::Quaterniond(
            ctrl_msg.orientation.w,
            ctrl_msg.orientation.x,
            ctrl_msg.orientation.y,
            ctrl_msg.orientation.z
        )
    );

    if(dynamic_tensor_tk == 0)
    {
        ctrl_msg_tensor = torch::zeros({1, 4, 1}, torch::kFloat32);
        ctrl_msg_tensor[0][0][0] = ctrl_msg.thrust;
        ctrl_msg_tensor[0][1][0] = ctrl_rpy(0);
        ctrl_msg_tensor[0][2][0] = ctrl_rpy(1);
        ctrl_msg_tensor[0][3][0] = ctrl_rpy(2);
    }
    else
    {
        torch::Tensor new_ctrl_msg_tensor = torch::zeros({1, 4, 1}, torch::kFloat32);
        new_ctrl_msg_tensor[0][0][0] = ctrl_msg.thrust;
        new_ctrl_msg_tensor[0][1][0] = ctrl_rpy(0);
        new_ctrl_msg_tensor[0][2][0] = ctrl_rpy(1);
        new_ctrl_msg_tensor[0][3][0] = ctrl_rpy(2);

        ctrl_msg_tensor = torch::cat({ctrl_msg_tensor, new_ctrl_msg_tensor}, 2);
    }

    dynamic_tensor_tk++;
}

Eigen::Vector3d ledvo::LedvoLib::dynamic_factor_infer(torch::Tensor tensor_to_be_inferred)
{
    cout<<1<<endl;
    int target_size = 200;
    int current_size = tensor_to_be_inferred.sizes()[2];
    cout<<2<<endl;
    cout<<tensor_to_be_inferred.sizes()<<endl;;
    cout<<3<<endl;
    // if(current_size > 200)
    //     pc::pattyDebug("DEBUG DYNAMIC FACTOR");

    // padding
    if (current_size < target_size)
    {
        cout<<4<<endl;
        int padding_size = target_size - current_size;
        torch::Tensor padding_tensor = torch::zeros({1, 4, padding_size}, torch::kFloat32);
        cout<<5<<endl;
        tensor_to_be_inferred = torch::cat({padding_tensor, tensor_to_be_inferred}, 2);
        cout<<6<<endl;
    }
    cout<<7<<endl;

    at::Tensor output = module.forward({tensor_to_be_inferred}).toTensor();
    cout<<8<<endl;

    auto output_accessor = output.accessor<float, 2>();
    cout<<9<<endl;

    // Eigen Vector3d
    Eigen::Vector3d returnXYZ;

    for (int i = 0; i < 3; i++) 
        returnXYZ[i] = output_accessor[0][i];
    
    return returnXYZ;
}
