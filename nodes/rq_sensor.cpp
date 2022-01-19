/* Software License Agreement (BSD License)
*
* Copyright (c) 2014, Robotiq, Inc.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of Robotiq, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Copyright (c) 2014, Robotiq, Inc
*/

/**
 * \file rq_sensor.cpp
 * \date July 14, 2014
 *  \author Jonathan Savoie <jonathan.savoie@robotiq.com>
 *  \maintainer Jean-Philippe Roberge <ros@robotiq.com>
 */

#include <string.h>
#include <stdio.h>
#include <memory>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include "robotiq_ft_sensor/rq_sensor_state.h"
#include "robotiq_ft_sensor/msg/ftsensor.hpp"
#include "robotiq_ft_sensor/srv/sensoraccessor.hpp"

typedef robotiq_ft_sensor::srv::Sensoraccessor::Request Request;
int max_retries_(100);
std::string ftdi_id;

std::shared_ptr<rclcpp::Node> node ;

/**
 * @brief decode_message_and_do Decode the message received and do the associated action
 * @param req request (of which the command_id is used)
 * @param res result with requested data
 * @return true iff a valid command_id was given in the request
 */
static bool decode_message_and_do(const std::shared_ptr<robotiq_ft_sensor::srv::Sensoraccessor::Request> req,
               std::shared_ptr<robotiq_ft_sensor::srv::Sensoraccessor::Response> res)
{
	INT_8 buffer[100];
	res->success = rq_state_get_command(req->command_id, buffer);
	res->res = buffer;

	// Request::COMMAND_GET_SERIAL_NUMBER = 1;
	// Request::COMMAND_GET_FIRMWARE_VERSION=2;
	// Request::COMMAND_GET_PRODUCTION_YEAR=4;
	// Request::COMMAND_SET_ZERO=8;


	if (!res->success)
	{
		RCLCPP_WARN(rclcpp::get_logger("rclcpp"),"Unsupported command_id '%i', should be in [%i, %i, %i, %i]",
			 req->command_id,
			 Request::COMMAND_GET_SERIAL_NUMBER,
			 Request::COMMAND_GET_FIRMWARE_VERSION,
			 Request::COMMAND_GET_PRODUCTION_YEAR,
			 Request::COMMAND_SET_ZERO);
	}

	return res->success;
}


/**
 * \brief Decode the message received and do the associated action
 * \param buff message to decode
 * \param ret buffer containing the return value from a GET command
 */
static void decode_message_and_do(INT_8 const  * const buff, INT_8 * const ret)
{
	INT_8 get_or_set[3];
	INT_8 nom_var[4];

	if(buff == NULL || strlen(buff) != 7)
	{
		return;
	}

	strncpy(get_or_set, &buff[0], 3);
	strncpy(nom_var, &buff[4], strlen(buff) -3);

	if(strstr(get_or_set, "GET"))
	{
		rq_state_get_command(nom_var, ret);
	}
	else if(strstr(get_or_set, "SET"))
	{
		if(strstr(nom_var, "ZRO"))
		{
			rq_state_do_zero_force_flag();
			strcpy(ret,"Done");
		}
	}
}

bool receiverCallback(const std::shared_ptr<robotiq_ft_sensor::srv::Sensoraccessor::Request> req,
        std::shared_ptr<robotiq_ft_sensor::srv::Sensoraccessor::Response> res)
{
	/// Support for old string-based interface
	if (req->command.length())
	{
		RCLCPP_WARN_ONCE(rclcpp::get_logger("rclcpp"),"Usage of command-string is deprecated, please use the numeric command_id");
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"I heard: [%s]",req->command.c_str());
		INT_8 buffer[512];
		decode_message_and_do((char*)req->command.c_str(), buffer);
		res->res = buffer;
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"I send: [%s]", res->res.c_str());
		return true;
	}

	/// New interface with numerical commands
	decode_message_and_do(req, res);
	return true;
}

static INT_8 sensor_state_machine()
{
    if (ftdi_id.empty())
    {
        return rq_sensor_state(max_retries_);
    }

    return rq_sensor_state(max_retries_, ftdi_id);
}

/**
 * \fn static void wait_for_other_connection()
 * \brief Each second, checks for a sensor that has been connected
 */
static void wait_for_other_connection()
{
	INT_8 ret;

	while(rclcpp::ok())
	{
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Waiting for sensor connection...");
		usleep(1000000);//Attend 1 second.

        ret = sensor_state_machine();
        if(ret == 0)
		{
			RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Sensor connected!");
			break;
		}

		rclcpp::spin_some(node);
	}
}

/**
 * \fn void get_data(void)
 * \brief Builds the message with the force/torque data
 * \return ft_sensor updated with the latest data
 */
static robotiq_ft_sensor::msg::Ftsensor get_data(void)
{
	robotiq_ft_sensor::msg::Ftsensor msgStream;
	// msgStream = robotiq_ft_sensor::msg::Ftsensor();

	msgStream.fx = rq_state_get_received_data(0);
	msgStream.fy = rq_state_get_received_data(1);
	msgStream.fz = rq_state_get_received_data(2);
	msgStream.mx = rq_state_get_received_data(3);
	msgStream.my = rq_state_get_received_data(4);
	msgStream.mz = rq_state_get_received_data(5);

	return msgStream;
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);

	node = rclcpp::Node::make_shared("robotiq_ft_sensor");
	// node->declare_parameter<int>("max_retries_",100);
	if(node->get_parameter("max_retries", max_retries_))
	{
		max_retries_=100;
	}
	node->declare_parameter<std::string>("serial_id","ttyUSB0");
	if(!node->get_parameter("serial_id", ftdi_id))
	{
		ftdi_id = "ttyUSB0";
		RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Damn get_parameter");
	}
		

    if (!ftdi_id.empty())
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Trying to connect to a sensor at /dev/%s", ftdi_id.c_str());
    }
    else
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"No device filename specified. Will attempt to discover Robotiq force torque sensor./dev/%s", ftdi_id.c_str());
    }

	INT_8 bufStream[512];
	robotiq_ft_sensor::msg::Ftsensor msgStream;
	msgStream = robotiq_ft_sensor::msg::Ftsensor();
	INT_8 ret;

	//If we can't initialize, we return an error
	ret = sensor_state_machine();
	if(ret == -1)
	{
		wait_for_other_connection();
	}

	//Reads basic info on the sensor
	ret = sensor_state_machine();
	if(ret == -1)
	{
		wait_for_other_connection();
	}

	//Starts the stream
	ret = sensor_state_machine();
	if(ret == -1)
	{
		wait_for_other_connection();
	}
	rclcpp::Publisher<robotiq_ft_sensor::msg::Ftsensor>::SharedPtr sensor_pub;
	rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub;
	rclcpp::Service<robotiq_ft_sensor::srv::Sensoraccessor>::SharedPtr service;
	
	sensor_pub = node->create_publisher<robotiq_ft_sensor::msg::Ftsensor>("robotiq_ft_sensor", 512);
	wrench_pub = node->create_publisher<geometry_msgs::msg::WrenchStamped>("robotiq_ft_wrench", 512);
	service = node->create_service<robotiq_ft_sensor::srv::Sensoraccessor>("robotiq_ft_sensor_acc", &receiverCallback);

	//std_msgs::String msg;
	geometry_msgs::msg::WrenchStamped wrenchMsg;
	// wrenchMsg = geometry_msgs::msg::WrenchStamped();
	
	// node->declare_parameter<std::string>("wrenchMsg.header.frame_id","robotiq_ft_frame_id");
	if(node->get_parameter("frame_id", wrenchMsg.header.frame_id))
		wrenchMsg.header.frame_id = "robotiq_ft_frame_id";

	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"Starting Sensor");
	// ros::Rate r(1000);
	while(rclcpp::ok())
	{
        ret = sensor_state_machine();
        if (ret == -1)
        {
			wait_for_other_connection();
		}

		if(rq_sensor_get_current_state() == RQ_STATE_RUN)
		{
			strcpy(bufStream,"");
			msgStream = get_data();
			// msgStream.fx = rq_state_get_received_data(0);
			// msgStream.fy = rq_state_get_received_data(1);
			// msgStream.fz = rq_state_get_received_data(2);
			// msgStream.mx = rq_state_get_received_data(3);
			// msgStream.my = rq_state_get_received_data(4);
			// msgStream.mz = rq_state_get_received_data(5);

			if(rq_state_got_new_message())
			{
				sensor_pub->publish(msgStream);

				//compose WrenchStamped Msg
				// wrenchMsg.header.stamp = node->now();
				wrenchMsg.wrench.force.x = msgStream.fx;
				wrenchMsg.wrench.force.y = msgStream.fy;
				wrenchMsg.wrench.force.z = msgStream.fz;
				wrenchMsg.wrench.torque.x = msgStream.mx;
				wrenchMsg.wrench.torque.y = msgStream.my;
				wrenchMsg.wrench.torque.z = msgStream.mz;
				wrench_pub->publish(wrenchMsg);
			}
		}

		rclcpp::spin_some(node);
		// r.sleep();
	}
	return 0;
}
