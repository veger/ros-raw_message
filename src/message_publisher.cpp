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
#include "raw_message/message_publisher.h"

#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"

using namespace std;

namespace raw_message
{
  MessagePublisher::MessagePublisher(ros::NodeHandle& nh, string const& topic, FieldTypes fieldType, const uint32_t queueSize) :
      fieldType(fieldType)
  {
    // Advertise topic using correct message type
    ros::AdvertiseOptions opts;
    switch(fieldType)
    {
      case String:
        opts.init<std_msgs::String>(topic, queueSize);
        break;
      case Int8:
        opts.init<std_msgs::Int8>(topic, queueSize);
        break;
      case Int16:
        opts.init<std_msgs::Int16>(topic, queueSize);
        break;
      case Int32:
        opts.init<std_msgs::Int32>(topic, queueSize);
        break;
      case Int64:
        opts.init<std_msgs::Int64>(topic, queueSize);
        break;
      default:
        ROS_ERROR(" Unsupported FieldType: %d", fieldType);
        this->fieldType = UNKNOWN;
        return;
    }

    publisher = nh.advertise(opts);
  }


  void MessagePublisher::publish(const int8_t i)
  {
    std_msgs::Int8 msg;
    switch(fieldType)
    {
      case Int8:
        msg.data = i;
        publisher.publish(msg);
        break;
      case Int16:
        // Send given 8 bit int as 16 bit int
        publish((const int16_t) i);
        break;
      case Int32:
        // Send given 8 bit int as 32 bit int
        publish((const int32_t) i);
        break;
      case Int64:
        // Send given 8 bit int as 64 bit int
        publish((const int64_t) i);
        break;
      default:
        ROS_ERROR("Tried to send an Int8 value using %s publisher, ignoring.", toString(fieldType));
        break;
    }
  }

  void MessagePublisher::publish(const int16_t i)
  {
    std_msgs::Int16 msg;
    switch(fieldType)
    {
      case Int16:
        msg.data = i;
        publisher.publish(msg);
        break;
      case Int32:
        // Send given 8 bit int as 32 bit int
        publish((const int32_t) i);
        break;
      case Int64:
        // Send given 8 bit int as 64 bit int
        publish((const int64_t) i);
        break;
      default:
        ROS_ERROR("Tried to send an Int16 value using %s publisher, ignoring.", toString(fieldType));
        break;
    }
  }

  void MessagePublisher::publish(const int32_t i)
  {
    std_msgs::Int32 msg;
    switch(fieldType)
    {
      case Int32:
        msg.data = i;
        publisher.publish(msg);
        break;
      case Int64:
        // Send given 8 bit int as 64 bit int
        publish((const int64_t) i);
        break;
      default:
        ROS_ERROR("Tried to send an Int32 value using %s publisher, ignoring.", toString(fieldType));
        break;
    }
  }

  void MessagePublisher::publish(const int64_t i)
  {
    std_msgs::Int64 msg;
    switch(fieldType)
    {
      case Int64:
        msg.data = i;
        publisher.publish(msg);
        break;
      default:
        ROS_ERROR("Tried to send an Int64 value using %s publisher, ignoring.", toString(fieldType));
        break;
    }
  }

  void MessagePublisher::publish(std::string const& str)
  {
    if(fieldType != String)
    {
        ROS_ERROR("Tried to send an String value using %s publisher, ignoring.", toString(fieldType));
      return;
    }
    std_msgs::String msg;
    msg.data = str;
    publisher.publish(msg);
  }
}