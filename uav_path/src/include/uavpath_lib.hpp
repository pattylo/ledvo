/*
    This file is part of LEDVO - learning dynamic factor for visual odometry

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
        int pub_freq;
        int lap;
        bool ccw;

        double amp;
        double velo; // dg / s
        double timePerLap;

        bool z_moving;
        double z_moving_amp;
        double z_moving_period; 
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
        uav_traj_info_lemi lemi_traj_info;

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

        uavpath(            
            Eigen::Vector3d center, 
            int pub_freq,
            int lap,
            bool ccw,
            //////////
            double amp,
            double velo,
            double timePerLap,
            //////////
            bool z_moving,
            double z_moving_amp,
            double z_moving_period,
            //////////
            std::string traj_type = LEMNISCATE_TRAJ
        )  // lemniscate
        : _traj_type(traj_type)
        {
            lemi_traj_info.center = center;
            lemi_traj_info.pub_freq = pub_freq;
            lemi_traj_info.lap = lap;
            lemi_traj_info.ccw = ccw;

            lemi_traj_info.amp = amp;
            lemi_traj_info.velo = velo;
            lemi_traj_info.timePerLap = timePerLap;

            if(z_moving)
            {
                lemi_traj_info.z_moving = z_moving;
                lemi_traj_info.z_moving_amp = z_moving_amp;
                lemi_traj_info.z_moving_period = z_moving_period;
            }

            std::cout<<LEMNISCATE_TRAJ<<std::endl;

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
        int traj_total_no_per_lap = (360.0 / circle_traj_info.velo) * circle_traj_info.pub_freq;

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
                {
                    traj_pt_temp.z = circle_traj_info.center.z() 
                        + circle_traj_info.z_moving_amp 
                        * sin(
                            2.0 * M_PI / circle_traj_info.z_moving_period 
                            * (1.0 / circle_traj_info.pub_freq) * double(i * j + j)
                        );
                }
                    
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
        int traj_total_no_per_lap = ((block_traj_info.length + block_traj_info.length * block_traj_info.aspect_ratio) * 2) / block_traj_info.velo * block_traj_info.pub_freq;
        
        Eigen::Vector2d starting2d;
        starting2d.x() = block_traj_info.center.x() + block_traj_info.length * block_traj_info.aspect_ratio / 2.0;
        starting2d.y() = block_traj_info.center.y() + block_traj_info.length / 2.0;
        int last_index;

        for(int i = 0; i < block_traj_info.lap; i++)
        {
            geometry_msgs::Point traj_pt_temp;

            for(int j = 0; j < traj_total_no_per_lap; j++)
            {
                if(j < 1.0 * traj_total_no_per_lap / 4.0)
                {
                    traj_pt_temp.x = starting2d.x() - block_traj_info.velo * (1.0 / block_traj_info.pub_freq * j);
                    traj_pt_temp.y = starting2d.y();
                }
                else if (1.0 * traj_total_no_per_lap / 4.0 < j && j < 2.0 * traj_total_no_per_lap / 4.0)
                {
                    traj_pt_temp.x = starting2d.x() - block_traj_info.length * block_traj_info.aspect_ratio;
                    traj_pt_temp.y = starting2d.y() - block_traj_info.velo * (1.0 / block_traj_info.pub_freq * (j - 1.0 * traj_total_no_per_lap / 4.0));
                }
                else if (2.0 * traj_total_no_per_lap / 4.0 < j && j < 3.0 * traj_total_no_per_lap / 4.0)
                {
                    traj_pt_temp.x = starting2d.x() - block_traj_info.length * block_traj_info.aspect_ratio +  block_traj_info.velo * (1.0 / block_traj_info.pub_freq * (j - 2.0 * traj_total_no_per_lap / 4.0));
                    traj_pt_temp.y = starting2d.y() - block_traj_info.length;

                }
                else if (3.0 * traj_total_no_per_lap / 4.0 < j )
                {
                    traj_pt_temp.x = starting2d.x();
                    traj_pt_temp.y = starting2d.y() - block_traj_info.length + block_traj_info.velo * (1.0 / block_traj_info.pub_freq * (j - 3.0 * traj_total_no_per_lap / 4.0));
                }

                if(!block_traj_info.z_moving)
                    traj_pt_temp.z = block_traj_info.center.z();
                else
                {
                    traj_pt_temp.z = block_traj_info.center.z() 
                        + block_traj_info.z_moving_amp 
                        * sin(
                            2.0 * M_PI / block_traj_info.z_moving_period 
                            * (1.0 / block_traj_info.pub_freq) * double(i * j + j)
                        );
                }

                trajectory.emplace_back(traj_pt_temp);
            }
        }

        if(!block_traj_info.ccw)
        {
            std::reverse(trajectory.begin(), trajectory.end());
        }


    }
    else if(_traj_type == LEMNISCATE_TRAJ)
    {
        int traj_total_no_per_lap = (360.0 / lemi_traj_info.velo) * lemi_traj_info.pub_freq;

        for(int i = 0; i < lemi_traj_info.lap; i++)
        {
            geometry_msgs::Point traj_pt_temp;

            for(int j = 0; j < traj_total_no_per_lap; j++)
            {
                traj_pt_temp.x = lemi_traj_info.center.x() + lemi_traj_info.amp * cos(j / double(traj_total_no_per_lap) * 2.0 * M_PI);
                traj_pt_temp.y = lemi_traj_info.center.y() + lemi_traj_info.amp * cos(j / double(traj_total_no_per_lap) * 2.0 * M_PI) * sin(j / double(traj_total_no_per_lap) * 2.0 * M_PI);
                
                if(!lemi_traj_info.z_moving)
                    traj_pt_temp.z = lemi_traj_info.center.z();
                else
                {
                    traj_pt_temp.z = lemi_traj_info.center.z() 
                        + lemi_traj_info.z_moving_amp 
                        * sin(
                            2.0 * M_PI / lemi_traj_info.z_moving_period 
                            * (1.0 / lemi_traj_info.pub_freq) * double(i * j + j)
                        );
                }

                trajectory.emplace_back(traj_pt_temp);     
            }
        }

        std::cout<<"final size: "<<trajectory.size()<<std::endl;  

        if(!lemi_traj_info.ccw)
        {
            std::reverse(trajectory.begin(), trajectory.end());
        }
    }
    else
    {
        std::printf("PLEASE CHECK CODE...");
    }
}

#endif