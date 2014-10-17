/**
 * (C) Copyright 2014 Robotics and Mechatronics, University of Twente
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
#include "message_decoder/message_decoder.h"

using namespace std;

namespace message_decoder
{

void MessageDecoder::decodeMessage(boost::shared_ptr<topic_tools::ShapeShifter const> const &msg)
{
  ROS_INFO("Data type: %s, size %u", msg->getDataType().c_str(), msg->size());

  messagePosition = 0;
  ros::serialization::OStream stream(messageData, sizeof(messageData));
  msg->write(stream);

  // Parse the message definition
  // Each message field is on a new line, so read each line separately
  std::stringstream ss(msg->getMessageDefinition());
  std::string line;
  while (std::getline(ss, line, '\n'))
  {
    // Each line consists of a field type and name
    stringstream ss2(line);
    string fieldType, fieldName;
    ss2 >> fieldType;
    ss2 >> fieldName;

    ROS_INFO("Field %s of type %s", fieldName.c_str(), fieldType.c_str());

    decodeField(fieldType);
  }
}

void MessageDecoder::decodeField(string const& field)
{
  if (field == "string")
  {
    // First word is the text length, followed by the text
    uint32_t textLength = ((int32_t *)(messageData + messagePosition))[0];
    messagePosition += 4;
    string text(messageData + messagePosition, messageData + messagePosition + textLength);
    ROS_INFO(" (string): %s", text.c_str());
    messagePosition += textLength;
  }
  else if (field == "int64")
  {
    ROS_INFO(" (int64_t): %ld", ((int64_t * ) (messageData + messagePosition))[0]);
    messagePosition += 8;
  }
  else if (field == "int32")
  {
    ROS_INFO(" (int32_t): %d", ((int32_t * ) (messageData + messagePosition))[0]);
    messagePosition += 4;
  }
  else if (field == "int16")
  {
    ROS_INFO(" (int16_t): %d", ((int16_t * ) (messageData + messagePosition))[0]);
    messagePosition += 2;
  }
  else if (field == "int8")
  {
    ROS_INFO(" (int8_t): %d", ((int8_t * ) (messageData + messagePosition))[0]);
    messagePosition += 1;
  }
  else
  {
    ROS_ERROR(" Unknown data type: %s", field.c_str());
  }
}
}
