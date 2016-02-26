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

#ifndef YOUBOTOODLWRAPPER_H_
#define YOUBOTOODLWRAPPER_H_

/* Stringification helper macros */
#define mkstr2(X) #X
#define mkstr(X) mkstr2(X)

/* BOOST includes */
#include <boost/units/io.hpp>

/* ROS includes */
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "std_srvs/Empty.h"
#include "youbot_driver_ros_interface/BaseSetPosition.h"
#include "youbot_driver_ros_interface/BaseDisplace.h"
#include "youbot_driver_ros_interface/BaseRotate.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include <diagnostic_msgs/DiagnosticArray.h>

#include <pr2_msgs/PowerBoardState.h>

#include "sensor_msgs/JointState.h"

/* OODL includes */
#include "YouBotConfiguration.h"
#include <youbot_driver/youbot/DataTrace.hpp>

namespace youBot {

    /**
 * @brief Wrapper class to map ROS messages to OODL method calls for the youBot platform.
 */
    class YouBotOODLWrapper {
    public:
        /**
     * @brief Constructor with a ROS handle.
     * @param n ROS handle
     */
        YouBotOODLWrapper(ros::NodeHandle n);

        /**
     * @brief DEfault constructor.
     */
        virtual ~YouBotOODLWrapper();

        /* Coordination: */

        /**
     * @brief Initializes a youBot base.
     * @param baseName Name of the base. Used to open the configuration file e.g. youbot-base.cfg
     */
        void initializeBase(std::string baseName);

        void setBaseVelocity(double youBotBaseJointVelocity);

        void setBaseAcceleration(double youBotBaseJointAcceleration);

        /**
     * @brief Stops all initialized elements.
     * Stops arm and/or base (if initialized).
     */
        void stop();

        /* Communication: */

        /**
     * @brief Callback that is executed when a cmd_vel for the base comes in.
     * @param youbotBaseVelCommand Message that contains the desired translational and rotational velocity for the base.
     */
        void baseVelCommandCallback(const geometry_msgs::Twist& youbotBaseVelCommand);

        /**
     * @brief Publishes all sensor measurements. Both for base and arm.
     *
     * Depending on what has been initialized before, either odometry and/or joint state valiues are published.
     * computeOODLSensorReadings needs to be executed before.
     */
        void publishOODLSensorReadings();

        /**
    * @brief Publishes status of base as diagnostic and dashboard messages continuously
    */
        void publishBaseDiagnostics(double publish_rate_in_secs);

        /* Computation: */

        /**
     * @brief Mapps OODL values to ROS messages
     */
        void computeOODLSensorReadings();

        bool switchOffBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        bool switchOnBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /**
     * @brief Callback that is executed when a setPosition service request for the base comes in.
     * @param request Message that contains the target position for the base. longitudinal is the forward or backward target position, transversal is the sideway target position, orientation is the rotation around the center of the YouBot
     */
        bool baseSetPositionCallback(youbot_driver_ros_interface::BaseSetPosition::Request& request, youbot_driver_ros_interface::BaseSetPosition::Response& response);

        /**
     * @brief Callback that is executed when a displace service request for the base comes in.
     * @param request Message that contains the displacementfor the base. longitudinal is the forward or backward displacement, transversal is the sideway displacement
     */
        bool baseDisplaceCallback(youbot_driver_ros_interface::BaseDisplace::Request& request, youbot_driver_ros_interface::BaseDisplace::Response& response);

        /**
     * @brief Callback that is executed when a rotate service request for the base comes in.
     * @param request Message that contains the rotation for the base. angle is the rotation around the center of the YouBot
     */
        bool baseRotateCallback(youbot_driver_ros_interface::BaseRotate::Request& request, youbot_driver_ros_interface::BaseRotate::Response& response);

        bool reconnectCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

        /* Configuration: */

        /// Handle the aggregates all parts of a youBot system
        YouBotConfiguration youBotConfiguration;

    private:
        YouBotOODLWrapper(); //forbid default constructor

        /// Number of wheels attached to the base.
        static const int youBotNumberOfWheels = 4;

        std::string youBotChildFrameID;
        std::string youBotOdometryFrameID;
        std::string youBotOdometryChildFrameID;

        /// The ROS node handle
        ros::NodeHandle node;

        /// ROS timestamp
        ros::Time currentTime;

        /// The published odometry message with distances in [m], angles in [RAD] and velocities in [m/s] and [RAD/s]
        nav_msgs::Odometry odometryMessage;

        /// The published odometry tf frame with distances in [m]
        geometry_msgs::TransformStamped odometryTransform;

        /// The quaternion inside the tf odometry frame with distances in [m]
        geometry_msgs::Quaternion odometryQuaternion;

        /// The published joint state of the base (wheels) with angles in [RAD] and velocities in [RAD/s]
        sensor_msgs::JointState baseJointStateMessage;

        double youBotDriverCycleFrequencyInHz;

        /// diagnostic msgs
        ros::Time lastDiagnosticPublishTime;

        ros::Publisher dashboardMessagePublisher;
        pr2_msgs::PowerBoardState platformStateMessage;

        ros::Publisher diagnosticArrayPublisher;
        diagnostic_msgs::DiagnosticArray diagnosticArrayMessage;
        diagnostic_msgs::DiagnosticStatus diagnosticStatusMessage;
        std::string diagnosticNameBase;

        bool areBaseMotorsSwitchedOn;
    };

} // namespace youBot

#endif /* YOUBOTOODLWRAPPER_H_ */

/* EOF */
