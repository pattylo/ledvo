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
 * \file uavpath_lib.hpp
 * \date 26/10/2023
 * \author pattylo
 * \copyright (c) AIRO-LAB, RCUAS of Hong Kong Polytechnic University
 * \brief classes of simple trajectories for UAV
 */

#ifndef UAVPATH_LIB_H
#define UAVPATH_LIB_H

#include "essential.h"

#define CIRCLE_TRAJ "CIRCLE_TRAJ"
#define BLOCK_TRAJ "BLOCK_TRAJ"
#define LEMNISCATE_TRAJ "LEMNISCATE_TRAJ"

namespace uav
{
    typedef struct uav_traj_info_circle
    {
        Eigen::Vector3d center; 
        int pub_freq; 
        int lap;
        bool ccw;
        
        double velo; // dg/s
        double radius; 
        
        bool z_moving;
        double z_moving_amp;
        double z_moving_period;
    }uav_traj_info_circle;

    typedef struct uav_traj_info_block
    {
        Eigen::Vector3d center; 
        int pub_freq;
        int lap;
        bool ccw;

        double velo; // m/s
        double length; 
        double aspect_ratio; 
        double T_angle;
        
        bool z_moving;
        double z_moving_amp;
        double z_moving_period;        
    }uav_traj_info_block;

    typedef struct uav_traj_info_lemi // leminiscate
    {
        Eigen::Vector3d center;
        double radius;
        double T_angle;
        int pub_freq;
        double velo; // dg/s
        int lap;
        bool ccw; // ∞ (RHS circle ccw true or not)
    }uav_traj_info_lemi;

    class uavpath
    {
    private:
        int traj_i = 0;
        bool start_trajectory = false;
        std::vector<geometry_msgs::Point> trajectory;
        std::string _traj_type;

        uav_traj_info_circle circle_traj_info;
        uav_traj_info_block block_traj_info;

        inline void set_traj();

    public:

        // define overload
        uavpath(
            Eigen::Vector3d center, 
            int pub_freq,
            int lap,
            bool ccw,
            //////////
            double velo,
            double radius,
            //////////
            bool z_moving,
            double z_moving_amp,
            double z_moving_period,
            //////////
            std::string traj_type = CIRCLE_TRAJ
        )  // circle
        : _traj_type(traj_type)
        {
            circle_traj_info.center = center;
            circle_traj_info.pub_freq = pub_freq;
            circle_traj_info.lap = lap;
            circle_traj_info.ccw = ccw;

            circle_traj_info.velo = velo;
            circle_traj_info.radius = radius;

            if(z_moving)
            {
                circle_traj_info.z_moving = z_moving;
                circle_traj_info.z_moving_amp = z_moving_amp;
                circle_traj_info.z_moving_period = z_moving_period;
            }

            std::cout<<CIRCLE_TRAJ<<std::endl;

            set_traj();
        };

        uavpath(            
            Eigen::Vector3d center, 
            int pub_freq,
            int lap,
            bool ccw,
            //////////
            double velo,
            double length,
            double aspect_ratio,
            double T_angle,
            //////////
            bool z_moving,
            double z_moving_amp,
            double z_moving_period,
            //////////
            std::string traj_type = BLOCK_TRAJ
        )  // block
        : _traj_type(traj_type)
        {
            block_traj_info.center = center;
            block_traj_info.pub_freq = pub_freq;
            block_traj_info.lap = lap;
            block_traj_info.ccw = ccw;

            block_traj_info.velo = velo;
            block_traj_info.length = length;
            block_traj_info.aspect_ratio = aspect_ratio;
            block_traj_info.T_angle = T_angle;

            if(z_moving)
            {
                block_traj_info.z_moving = z_moving;
                block_traj_info.z_moving_amp = z_moving_amp;
                block_traj_info.z_moving_period = z_moving_period;
            }

            std::cout<<BLOCK_TRAJ<<std::endl;

            set_traj();
        };                        
        ~uavpath(){};

        inline std::vector<geometry_msgs::Point> get_3Dtraj(){
            return trajectory;
        };
    };  

}


void uav::uavpath::set_traj()
{
    if(_traj_type == CIRCLE_TRAJ)
    {
        int traj_total_no_per_lap = (360 / circle_traj_info.velo) * circle_traj_info.pub_freq;

        std::cout<<traj_total_no_per_lap<<std::endl;

        for(int i = 0; i < circle_traj_info.lap; i++)
        {                
            geometry_msgs::Point traj_pt_temp;
            for(int j = 0; j < traj_total_no_per_lap; j++)
            {
                traj_pt_temp.x = circle_traj_info.center.x() + circle_traj_info.radius * cos(j / double(traj_total_no_per_lap) * 2.0 * M_PI);

                traj_pt_temp.y = circle_traj_info.center.y() + circle_traj_info.radius * sin(j / double(traj_total_no_per_lap) * 2.0 * M_PI);

                if(!circle_traj_info.z_moving)
                    traj_pt_temp.z = circle_traj_info.center.z();
                else
                    traj_pt_temp.z = circle_traj_info.center.z() + circle_traj_info.z_moving_amp * sin(j / double(traj_total_no_per_lap) * 2.0 * M_PI);

                trajectory.emplace_back(traj_pt_temp);                        
            }
        }     

        std::cout<<"final size: "<<trajectory.size()<<std::endl;  

        if(!circle_traj_info.ccw)
        {
            std::reverse(trajectory.begin(), trajectory.end());
        }

    }
    else if(_traj_type == BLOCK_TRAJ)
    {

    }
    else
    {
        std::printf("PLEASE CHECK CODE...");
    }
}

#endif