/******************************************************************************
 * Copyright (c) 2011
 * Locomotec
 *
 * Author:
 * Sebastian Blumenthal
 *
 *
 * This software is published under a dual-license: GNU Lesser General Public
 * License LGPL 2.1 and BSD license. The dual-license implies that users of this
 * code may choose which terms they prefer.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 * * Neither the name of Locomotec nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version or the BSD license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL and the BSD license for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL and BSD license along with this program.
 *
 ******************************************************************************/

#ifndef YOUBOTCONFIGURATION_H_
#define YOUBOTCONFIGURATION_H_

/* ROS includes */
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

/* OODL includes */
#include <youbot_driver/youbot/YouBotBase.hpp>

namespace youBot {

    class YouBotBaseConfiguration {
    public:
        /// Standard constructor
        YouBotBaseConfiguration();

        /// Standard destructor
        virtual ~YouBotBaseConfiguration();

        /// Handle to the base
        youbot::YouBotBase* youBotBase;

        /// "Name" of the base. Typically derived from name of youBot configuration file.
        std::string baseID;

        /// Joint names for the wheels
        std::vector<std::string> wheelNames;

        /// Receives Twist messages for the base.
        ros::Subscriber baseVelCommandSubscriber;

        /// Publishes Odometry messages
        ros::Publisher baseOdometryPublisher;

        /// Publishes JointState messages with angles/velocities for the wheels.
        ros::Publisher baseJointStatePublisher;

        /// Service to switch the motor off by setting the PWM value to zero
        ros::ServiceServer switchOffMotorsService;

        /// Service to switch the motor ON by setting the velocity to zero
        ros::ServiceServer switchONMotorsService;

        /// Service to move the base to the desired position
        ros::ServiceServer setPositionService;

        /// Service to displace the base
        ros::ServiceServer displaceService;

        /// Service to rotate the base
        ros::ServiceServer rotateService;

        /// Publishes tf frames as odometry
        tf::TransformBroadcaster odometryBroadcaster;
    };

    /**
 * @brief Aggregation class for instantiated parts of a youBot systems and all its name mapping between ROS and the youBot API.
 */
    class YouBotConfiguration {
    public:
        YouBotConfiguration();
        virtual ~YouBotConfiguration();

        ///Flag to indicate if youBot has a base (set after successful initialization)
        bool hasBase;

        /// Path to the configuration files, required by OODL (e.g. youbot-base.cfg)
        std::string configurationFilePath;

        /// A youbot system has one base
        YouBotBaseConfiguration baseConfiguration;

        /// Publishes diagnostic messages
        ros::Publisher diagnosticArrayPublisher;

        ros::Publisher dashboardMessagePublisher;
    };

} // namespace youBot

#endif /* YOUBOTCONFIGURATION_H_ */

/* EOF */
