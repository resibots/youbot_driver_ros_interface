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

int main(int argc, char** argv)
{

    youbot::Logger::toConsole = false;
    youbot::Logger::toFile = false;
    youbot::Logger::toROS = true;
    ros::init(argc, argv, "youbot_oodl_driver");
    ros::NodeHandle n;
    youBot::YouBotOODLWrapper youBot(n);

    /* configuration */
    bool youBotHasBase;
    double youBotDriverCycleFrequencyInHz; //the driver recives commands and publishes them with a fixed frequency
    double youBotBaseJointVelocity;
    double youBotBaseJointAcceleration;
    n.param("youBotHasBase", youBotHasBase, true);
    n.param("youBotDriverCycleFrequencyInHz", youBotDriverCycleFrequencyInHz, 50.0);
    n.param<std::string>("youBotConfigurationFilePath", youBot.youBotConfiguration.configurationFilePath, mkstr(YOUBOT_CONFIGURATIONS_DIR));
    n.param<std::string>("youBotBaseName", youBot.youBotConfiguration.baseConfiguration.baseID, "youbot-base");
    n.param("youBotBaseJointVelocity", youBotBaseJointVelocity, 8.0);
    n.param("youBotBaseJointAcceleration", youBotBaseJointAcceleration, 4.0);

    ros::ServiceServer reconnectService = n.advertiseService("reconnect", &youBot::YouBotOODLWrapper::reconnectCallback, &youBot);

    ROS_INFO("Configuration file path: %s", youBot.youBotConfiguration.configurationFilePath.c_str());
    try {
        youbot::EthercatMaster::getInstance("youbot-ethercat.cfg", youBot.youBotConfiguration.configurationFilePath);
    }
    catch (std::exception& e) {
        ROS_ERROR("No EtherCAT connection:");
        ROS_FATAL("%s", e.what());
        return 0;
    }

    ROS_ASSERT(youBotHasBase); // At least one should be true, otherwise nothing to be started.
    if (youBotHasBase == true) {
        youBot.initializeBase(youBot.youBotConfiguration.baseConfiguration.baseID);
        youBot.setBaseVelocity(youBotBaseJointVelocity);
        youBot.setBaseAcceleration(youBotBaseJointAcceleration);
    }

    /* coordination */
    ros::Rate rate(youBotDriverCycleFrequencyInHz); //Input and output at the same time... (in Hz)
    while (n.ok()) {
        ros::spinOnce();
        youBot.computeOODLSensorReadings();
        youBot.publishOODLSensorReadings();
        youBot.publishBaseDiagnostics(2.0); //publish only every 2 seconds
        rate.sleep();
    }

    youBot.stop();

    return 0;
}
