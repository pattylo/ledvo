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
 * \file ledvo_nodelet.h
 * \date 11/12/2022
 * \author pattylo
 * \copyright (c) RCUAS of Hong Kong Polytechnic University
 * \brief classes for vision-based relative localization for UAV and UGV based on LED markers
 */

#ifndef LEDVO_NODELET_H
#define LEDVO_NODELET_H

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <memory>

#include "ledvo_lib.h"

namespace ledvo
{
    class LedvoNodelet : public nodelet::Nodelet
    {
        public:
        virtual void onInit();
        std::shared_ptr<LedvoLib> ledvolib_ptr;
    };

    PLUGINLIB_EXPORT_CLASS(ledvo::LedvoNodelet, nodelet::Nodelet)
}

#endif