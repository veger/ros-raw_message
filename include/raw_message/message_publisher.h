/**
 * (C) Copyright 2015 Robotics and Mechatronics, University of Twente
 *
 * This file is part of the message_decoder library
 *
 * message_decoder is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * message_decoder is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with TERRA. If not, see <http://www.gnu.org/licenses/>.
 */
#pragma once

#include <string>

#include "raw_message/raw_message.h"
#include "ros/ros.h"

namespace raw_message
{

/**
 */
class MessagePublisher
{
public:
  MessagePublisher(ros::NodeHandle& nh, std::string const& topic, FieldTypes fieldType, const uint32_t queueSize = 10);

  void publish(const int8_t i);

  void publish(const int16_t i);

  void publish(const int32_t i);

  void publish(const int64_t i);

  void publish(std::string const& str);
private:

  /** Type of topic field */
  FieldTypes fieldType;

  /** Publisher that is used to publish the messages */
  ros::Publisher publisher;
};
}
