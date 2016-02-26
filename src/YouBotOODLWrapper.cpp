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

#include "youbot_driver_ros_interface/YouBotOODLWrapper.h"

#include <sstream>

namespace youBot {

    YouBotOODLWrapper::YouBotOODLWrapper()
    {
    }

    YouBotOODLWrapper::YouBotOODLWrapper(ros::NodeHandle n) : node(n)
    {
        youBotConfiguration.hasBase = false;
        areBaseMotorsSwitchedOn = false;

        youBotChildFrameID = "base_link"; //holds true for both: base and arm

        n.param("youBotDriverCycleFrequencyInHz", youBotDriverCycleFrequencyInHz, 50.0);

        diagnosticNameBase = "platform_Base";
        dashboardMessagePublisher = n.advertise<pr2_msgs::PowerBoardState>("/dashboard/platform_state", 1);
        diagnosticArrayPublisher = n.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 1);
    }

    YouBotOODLWrapper::~YouBotOODLWrapper()
    {
        this->stop();
        dashboardMessagePublisher.shutdown();
        diagnosticArrayPublisher.shutdown();
    }

    void YouBotOODLWrapper::initializeBase(std::string baseName)
    {

        try {
            youBotConfiguration.baseConfiguration.youBotBase = new youbot::YouBotBase(baseName, youBotConfiguration.configurationFilePath);
            youBotConfiguration.baseConfiguration.youBotBase->doJointCommutation();
        }
        catch (std::exception& e) {
            std::string errorMessage = e.what();
            ROS_FATAL("%s", errorMessage.c_str());
            ROS_ERROR("Base \"%s\" could not be initialized.", baseName.c_str());
            youBotConfiguration.hasBase = false;
            return;
        }

        /* setup input/output communication */
        youBotConfiguration.baseConfiguration.baseVelCommandSubscriber = node.subscribe("cmd_vel", 1000, &YouBotOODLWrapper::baseVelCommandCallback, this);
        youBotConfiguration.baseConfiguration.baseOdometryPublisher = node.advertise<nav_msgs::Odometry>("odom", 1);
        youBotConfiguration.baseConfiguration.baseJointStatePublisher = node.advertise<sensor_msgs::JointState>("base/joint_states", 1);

        /* setup services*/
        youBotConfiguration.baseConfiguration.switchOffMotorsService = node.advertiseService("base/switchOffMotors", &YouBotOODLWrapper::switchOffBaseMotorsCallback, this);
        youBotConfiguration.baseConfiguration.switchONMotorsService = node.advertiseService("base/switchOnMotors", &YouBotOODLWrapper::switchOnBaseMotorsCallback, this);
        youBotConfiguration.baseConfiguration.setPositionService = node.advertiseService("base/setPosition", &YouBotOODLWrapper::baseSetPositionCallback, this);
        youBotConfiguration.baseConfiguration.displaceService = node.advertiseService("base/displace", &YouBotOODLWrapper::baseDisplaceCallback, this);
        youBotConfiguration.baseConfiguration.rotateService = node.advertiseService("base/rotate", &YouBotOODLWrapper::baseRotateCallback, this);

        /* setup frame_ids */
        youBotOdometryFrameID = "odom";
        youBotOdometryChildFrameID = "base_footprint";

        ROS_INFO("Base is initialized.");
        youBotConfiguration.hasBase = true;
        areBaseMotorsSwitchedOn = true;
    }

    void YouBotOODLWrapper::setBaseVelocity(double youBotBaseJointVelocity)
    {
        for (int i = 1; i <= 4; i++) {
            youbot::MaximumPositioningVelocity maxPositioningVelocity;
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i).getConfigurationParameter(maxPositioningVelocity);
            maxPositioningVelocity.setParameter(youBotBaseJointVelocity * radian_per_second);
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i).setConfigurationParameter(maxPositioningVelocity);
        }
    }

    void YouBotOODLWrapper::setBaseAcceleration(double youBotBaseJointAcceleration)
    {
        for (int i = 1; i <= 4; i++) {
            youbot::MotorAcceleration motorAcceleration;
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i).getConfigurationParameter(motorAcceleration);
            motorAcceleration.setParameter(youBotBaseJointAcceleration * radians_per_second / boost::units::si::seconds);
            youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i).setConfigurationParameter(motorAcceleration);
        }
    }

    void YouBotOODLWrapper::stop()
    {

        if (youBotConfiguration.hasBase) {
            delete youBotConfiguration.baseConfiguration.youBotBase;
            youBotConfiguration.baseConfiguration.youBotBase = 0;
        }

        youBotConfiguration.baseConfiguration.baseVelCommandSubscriber.shutdown();
        youBotConfiguration.baseConfiguration.baseJointStatePublisher.shutdown();
        youBotConfiguration.baseConfiguration.baseOdometryPublisher.shutdown();
        youBotConfiguration.baseConfiguration.switchONMotorsService.shutdown();
        youBotConfiguration.baseConfiguration.switchOffMotorsService.shutdown();
        youBotConfiguration.baseConfiguration.setPositionService.shutdown();
        youBotConfiguration.baseConfiguration.displaceService.shutdown();
        youBotConfiguration.baseConfiguration.rotateService.shutdown();
        // youBotConfiguration.baseConfiguration.odometryBroadcaster.
        areBaseMotorsSwitchedOn = false;

        youbot::EthercatMaster::destroy();
    }

    void YouBotOODLWrapper::baseVelCommandCallback(const geometry_msgs::Twist& youbotBaseVelCommand)
    {

        if (youBotConfiguration.hasBase) { // in case stop has been invoked
            quantity<si::velocity> longitudinalVelocity;
            quantity<si::velocity> transversalVelocity;
            quantity<si::angular_velocity> angularVelocity;

            /*
         * Frame in OODL:
         *
         *		 FRONT
         *
         *         X
         *         ^
         *         |
         *         |
         *         |
         * Y <-----+
         *
         *        BACK
         *
         * Positive angular velocity means turning counterclockwise
         *
         */

            longitudinalVelocity = youbotBaseVelCommand.linear.x * meter_per_second;
            transversalVelocity = youbotBaseVelCommand.linear.y * meter_per_second;
            angularVelocity = youbotBaseVelCommand.angular.z * radian_per_second;

            try {
                youBotConfiguration.baseConfiguration.youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
            }
            catch (std::exception& e) {
                std::string errorMessage = e.what();
                ROS_WARN("Cannot set base velocities: %s", errorMessage.c_str());
            }
        }
        else {
            ROS_ERROR("No base initialized!");
        }
    }

    void YouBotOODLWrapper::computeOODLSensorReadings()
    {

        try {
            currentTime = ros::Time::now();
            youbot::JointSensedAngle currentAngle;
            youbot::JointSensedVelocity currentVelocity;

            youbot::EthercatMaster::getInstance().AutomaticReceiveOn(false); // ensure that all joint values will be received at the same time

            if (youBotConfiguration.hasBase) {
                double x = 0.0;
                double y = 0.0;
                double theta = 0.0;

                double vx = 0.0;
                double vy = 0.0;
                double vtheta = 0.0;

                quantity<si::length> longitudinalPosition;
                quantity<si::length> transversalPosition;
                quantity<plane_angle> orientation;

                quantity<si::velocity> longitudinalVelocity;
                quantity<si::velocity> transversalVelocity;
                quantity<si::angular_velocity> angularVelocity;

                youBotConfiguration.baseConfiguration.youBotBase->getBasePosition(longitudinalPosition, transversalPosition, orientation);
                x = longitudinalPosition.value();
                y = transversalPosition.value();
                theta = orientation.value();

                youBotConfiguration.baseConfiguration.youBotBase->getBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
                vx = longitudinalVelocity.value();
                vy = transversalVelocity.value();
                vtheta = angularVelocity.value();
                //ROS_DEBUG("Perceived odometric values (x,y,tetha, vx,vy,vtetha): %f, %f, %f \t %f, %f, %f", x, y, theta, vx, vy, vtheta);

                /* Setup odometry tf frame */
                odometryQuaternion = tf::createQuaternionMsgFromYaw(theta);

                odometryTransform.header.stamp = currentTime;
                odometryTransform.header.frame_id = youBotOdometryFrameID;
                odometryTransform.child_frame_id = youBotOdometryChildFrameID;

                odometryTransform.transform.translation.x = x;
                odometryTransform.transform.translation.y = y;
                odometryTransform.transform.translation.z = 0.0;
                odometryTransform.transform.rotation = odometryQuaternion;

                /* Setup odometry Message */
                odometryMessage.header.stamp = currentTime;
                odometryMessage.header.frame_id = youBotOdometryFrameID;

                odometryMessage.pose.pose.position.x = x;
                odometryMessage.pose.pose.position.y = y;
                odometryMessage.pose.pose.position.z = 0.0;
                odometryMessage.pose.pose.orientation = odometryQuaternion;

                odometryMessage.child_frame_id = youBotOdometryChildFrameID;
                //		odometryMessage.child_frame_id = youBotOdometryFrameID;
                odometryMessage.twist.twist.linear.x = vx;
                odometryMessage.twist.twist.linear.y = vy;
                odometryMessage.twist.twist.angular.z = vtheta;

                /* Set up joint state message for the wheels */
                baseJointStateMessage.header.stamp = currentTime;
                baseJointStateMessage.name.resize(youBotNumberOfWheels * 2); // *2 because of virtual wheel joints in the URDF description
                baseJointStateMessage.position.resize(youBotNumberOfWheels * 2);
                baseJointStateMessage.velocity.resize(youBotNumberOfWheels * 2);

                ROS_ASSERT((youBotConfiguration.baseConfiguration.wheelNames.size() == static_cast<unsigned int>(youBotNumberOfWheels)));
                for (int i = 0; i < youBotNumberOfWheels; ++i) {
                    youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i + 1).getData(currentAngle); //youBot joints start with 1 not with 0 -> i + 1
                    youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(i + 1).getData(currentVelocity);

                    baseJointStateMessage.name[i] = youBotConfiguration.baseConfiguration.wheelNames[i];
                    baseJointStateMessage.position[i] = currentAngle.angle.value();
                    baseJointStateMessage.velocity[i] = currentVelocity.angularVelocity.value();
                }

                /*
         * Here we add values for "virtual" rotation joints in URDF - robot_state_publisher can't
         * handle non-aggregated jointState messages well ...
         */
                baseJointStateMessage.name[4] = "caster_joint_fl";
                baseJointStateMessage.position[4] = 0.0;

                baseJointStateMessage.name[5] = "caster_joint_fr";
                baseJointStateMessage.position[5] = 0.0;

                baseJointStateMessage.name[6] = "caster_joint_bl";
                baseJointStateMessage.position[6] = 0.0;

                baseJointStateMessage.name[7] = "caster_joint_br";
                baseJointStateMessage.position[7] = 0.0;

                /*
         * Yet another hack to make the published values compatible with the URDF description.
         * We actually flipp the directions of the wheel on the right side such that the standard ROS controllers
         * (e.g. for PR2) can be used for the youBot
         */
                baseJointStateMessage.position[0] = -baseJointStateMessage.position[0];
                baseJointStateMessage.position[2] = -baseJointStateMessage.position[2];
            }

            youbot::EthercatMaster::getInstance().AutomaticReceiveOn(true); // ensure that all joint values will be received at the same time
        }
        catch (youbot::EtherCATConnectionException& e) {
            ROS_WARN("%s", e.what());
            youBotConfiguration.hasBase = false;
            ;
        }
        catch (std::exception& e) {
            ROS_WARN_ONCE("%s", e.what());
        }
    }

    void YouBotOODLWrapper::publishOODLSensorReadings()
    {

        if (youBotConfiguration.hasBase) {
            youBotConfiguration.baseConfiguration.odometryBroadcaster.sendTransform(odometryTransform);
            youBotConfiguration.baseConfiguration.baseOdometryPublisher.publish(odometryMessage);
            youBotConfiguration.baseConfiguration.baseJointStatePublisher.publish(baseJointStateMessage);
        }
    }

    bool YouBotOODLWrapper::switchOffBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        ROS_INFO("Switch off the base motors");
        if (youBotConfiguration.hasBase) { // in case stop has been invoked

            youbot::JointCurrentSetpoint currentStopMovement;
            currentStopMovement.current = 0.0 * ampere;
            try {
                youbot::EthercatMaster::getInstance().AutomaticSendOn(false); // ensure that all joint values will be send at the same time
                youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(1).setData(currentStopMovement);
                youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(2).setData(currentStopMovement);
                youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(3).setData(currentStopMovement);
                youBotConfiguration.baseConfiguration.youBotBase->getBaseJoint(4).setData(currentStopMovement);
                youbot::EthercatMaster::getInstance().AutomaticSendOn(true); // ensure that all joint values will be send at the same time
            }
            catch (std::exception& e) {
                std::string errorMessage = e.what();
                ROS_WARN("Cannot switch off the base motors: %s", errorMessage.c_str());
                return false;
            }
        }
        else {
            ROS_ERROR("No base initialized!");
            return false;
        }
        areBaseMotorsSwitchedOn = false;
        return true;
    }

    bool YouBotOODLWrapper::switchOnBaseMotorsCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        ROS_INFO("Switch on the base motors");
        if (youBotConfiguration.hasBase) { // in case stop has been invoked
            quantity<si::velocity> longitudinalVelocity;
            quantity<si::velocity> transversalVelocity;
            quantity<si::angular_velocity> angularVelocity;

            longitudinalVelocity = 0.0 * meter_per_second;
            transversalVelocity = 0.0 * meter_per_second;
            angularVelocity = 0.0 * radian_per_second;

            try {
                youBotConfiguration.baseConfiguration.youBotBase->setBaseVelocity(longitudinalVelocity, transversalVelocity, angularVelocity);
            }
            catch (std::exception& e) {
                std::string errorMessage = e.what();
                ROS_WARN("Cannot set base velocities: %s", errorMessage.c_str());
                return false;
            }
        }
        else {
            ROS_ERROR("No base initialized!");
            return false;
        }
        areBaseMotorsSwitchedOn = true;
        return true;
    }

    bool YouBotOODLWrapper::baseSetPositionCallback(youbot_driver_ros_interface::BaseSetPosition::Request& request, youbot_driver_ros_interface::BaseSetPosition::Response& response)
    {
        quantity<si::length> longitudinalPosition = request.longitudinal * meter;
        quantity<si::length> transversalPosition = request.transversal * meter;
        quantity<si::plane_angle> orientation = request.orientation * radian;

        if (youBotConfiguration.hasBase) { // in case stop has been invoked
            try {
                youBotConfiguration.baseConfiguration.youBotBase->setBasePosition(longitudinalPosition, transversalPosition, orientation);
            }
            catch (std::exception& e) {
                std::string errorMessage = e.what();
                ROS_WARN("Cannot set base position: %s", errorMessage.c_str());
                return false;
            }
        }
        else {
            ROS_ERROR("No base initialized!");
            return false;
        }
        return true;
    }

    bool YouBotOODLWrapper::baseDisplaceCallback(youbot_driver_ros_interface::BaseDisplace::Request& request, youbot_driver_ros_interface::BaseDisplace::Response& response)
    {
        quantity<si::length> longitudinalDisplacement = request.longitudinal * meter;
        quantity<si::length> transversalDisplacement = request.transversal * meter;

        if (youBotConfiguration.hasBase) { // in case stop has been invoked
            try {
                youBotConfiguration.baseConfiguration.youBotBase->setBaseDisplacement(longitudinalDisplacement, transversalDisplacement);

                std::vector<youbot::JointSensedAngle> last;
                youBotConfiguration.baseConfiguration.youBotBase->getJointData(last);
                bool finished = false;

                while (!finished) {
                    ros::Duration(0.1).sleep();
                    std::vector<youbot::JointSensedAngle> data;
                    youBotConfiguration.baseConfiguration.youBotBase->getJointData(data);
                    double diff = 0;
                    for (size_t i = 0; i < 4; i++)
                        diff += std::abs(last[i].angle.value() - data[i].angle.value());
                    last = data;
                    finished = diff < 1e-3;
                }
            }
            catch (std::exception& e) {
                std::string errorMessage = e.what();
                ROS_WARN("Cannot displace base: %s", errorMessage.c_str());
                return false;
            }
        }
        else {
            ROS_ERROR("No base initialized!");
            return false;
        }
        return true;
    }

    bool YouBotOODLWrapper::baseRotateCallback(youbot_driver_ros_interface::BaseRotate::Request& request, youbot_driver_ros_interface::BaseRotate::Response& response)
    {
        quantity<si::plane_angle> orientation = request.angle * radian;

        if (youBotConfiguration.hasBase) { // in case stop has been invoked
            try {
                youBotConfiguration.baseConfiguration.youBotBase->setBaseRotation(orientation);
                std::vector<youbot::JointSensedAngle> last;
                youBotConfiguration.baseConfiguration.youBotBase->getJointData(last);
                bool finished = false;

                while (!finished) {
                    ros::Duration(0.1).sleep();
                    std::vector<youbot::JointSensedAngle> data;
                    youBotConfiguration.baseConfiguration.youBotBase->getJointData(data);
                    double diff = 0;
                    for (size_t i = 0; i < 4; i++)
                        diff += std::abs(last[i].angle.value() - data[i].angle.value());
                    last = data;
                    finished = diff < 1e-3;
                }
            }
            catch (std::exception& e) {
                std::string errorMessage = e.what();
                ROS_WARN("Cannot rotate base: %s", errorMessage.c_str());
                return false;
            }
        }
        else {
            ROS_ERROR("No base initialized!");
            return false;
        }
        return true;
    }

    bool YouBotOODLWrapper::reconnectCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {

        this->stop();

        /* configuration */
        bool youBotHasBase;
        node.param("youBotHasBase", youBotHasBase, false);

        ROS_ASSERT(youBotHasBase); // At least one should be true, otherwise nothing to be started.
        if (youBotHasBase) {
            this->initializeBase(this->youBotConfiguration.baseConfiguration.baseID);
        }

        return true;
    }

    void YouBotOODLWrapper::publishBaseDiagnostics(double publish_rate_in_secs)
    {
        // only publish every X seconds
        if ((ros::Time::now() - lastDiagnosticPublishTime).toSec() < publish_rate_in_secs)
            return;

        lastDiagnosticPublishTime = ros::Time::now();

        platformStateMessage.header.stamp = ros::Time::now();
        diagnosticArrayMessage.header.stamp = ros::Time::now();
        diagnosticArrayMessage.status.clear();

        // diagnostics message
        // base status
        diagnosticStatusMessage.name = diagnosticNameBase;
        if (youBotConfiguration.hasBase) {
            diagnosticStatusMessage.message = "base is present";
            diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::OK;
        }
        else {
            diagnosticStatusMessage.message = "base is not connected or switched off";
            diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        }
        diagnosticArrayMessage.status.push_back(diagnosticStatusMessage);

        // EtherCAT status
        diagnosticStatusMessage.name = "platform_EtherCAT";
        if (youbot::EthercatMaster::getInstance().isEtherCATConnectionEstablished()) {
            diagnosticStatusMessage.message = "EtherCAT connnection is established";
            diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::OK;
            platformStateMessage.run_stop = false;
        }
        else {
            diagnosticStatusMessage.message = "EtherCAT connnection lost";
            diagnosticStatusMessage.level = diagnostic_msgs::DiagnosticStatus::ERROR;
            platformStateMessage.run_stop = true;
        }
        diagnosticArrayMessage.status.push_back(diagnosticStatusMessage);

        // dashboard message
        if (youBotConfiguration.hasBase && areBaseMotorsSwitchedOn)
            platformStateMessage.circuit_state[0] = pr2_msgs::PowerBoardState::STATE_ENABLED;
        else if (youBotConfiguration.hasBase && !areBaseMotorsSwitchedOn)
            platformStateMessage.circuit_state[0] = pr2_msgs::PowerBoardState::STATE_STANDBY;
        else
            platformStateMessage.circuit_state[0] = pr2_msgs::PowerBoardState::STATE_DISABLED;

        // publish established messages
        dashboardMessagePublisher.publish(platformStateMessage);
        diagnosticArrayPublisher.publish(diagnosticArrayMessage);
    }

} // namespace youBot

/* EOF */
