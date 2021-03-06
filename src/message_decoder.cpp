/**
 * (C) Copyright 2014-2015 Robotics and Mechatronics, University of Twente
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
#include "raw_message/message_decoder.h"

using namespace std;

namespace raw_message
{

MessageDecoder::MessageDecoder() :
    fieldType(UNKNOWN), fieldValueInt(0), messagePosition(0)
{
}

void MessageDecoder::startDecodingMessage(boost::shared_ptr<topic_tools::ShapeShifter const> const &msg)
{
  // Put message data into messageData buffer
  messagePosition = 0;
  ros::serialization::OStream stream(messageData, sizeof(messageData));
  msg->write(stream);

  // Store message definition, so we can iterate over it
  messageDescriptor.str(msg->getMessageDefinition());
}

bool MessageDecoder::findField(string const& searchFieldName, bool fromBegin)
{
  if (fromBegin)
  {
    resetDecoder();
  }

  // Iterate over fields until the requested one is found
  while (MessageDecoder::decodeNextField())
  {
    if (fieldName == searchFieldName)
    {
      // Found field
      return true;
    }
  }

  return false;
}

bool MessageDecoder::decodeNextField()
{
  // Parse the message descriptor
  // Each message field is on a new line, so read next line
  std::string line;
  if (std::getline(messageDescriptor, line, '\n') == NULL)
  {
    // Finished decoding
    return false;
  }

  // Each line consists of a field type and name
  string fieldTypeStr;
  stringstream ss2(line);
  ss2 >> fieldTypeStr;
  ss2 >> fieldName;

  if (fieldTypeStr == "string")
  {
    fieldType = String;
    // First word is the text length, followed by the text
    uint32_t textLength = ((int32_t *)(messageData + messagePosition))[0];
    messagePosition += 4;
    fieldValueString.assign(messageData + messagePosition, messageData + messagePosition + textLength);
    messagePosition += textLength;
  }
  else if (fieldTypeStr == "int64")
  {
    fieldType = Int64;
    fieldValueInt = ((int64_t *)(messageData + messagePosition))[0];
    messagePosition += 8;
  }
  else if (fieldTypeStr == "int32")
  {
    fieldType = Int32;
    fieldValueInt = ((int32_t *)(messageData + messagePosition))[0];
    messagePosition += 4;
  }
  else if (fieldTypeStr == "int16")
  {
    fieldType = Int16;
    fieldValueInt = ((int16_t *)(messageData + messagePosition))[0];
    messagePosition += 2;
  }
  else if (fieldTypeStr == "int8")
  {
    fieldType = Int8;
    fieldValueInt = ((int8_t *)(messageData + messagePosition))[0];
    messagePosition += 1;
  }
  else if (fieldTypeStr == "bool")
  {
    fieldType = Bool;
    fieldValueInt = ((bool *)(messageData + messagePosition))[0];
    messagePosition += 1;
  }
  else
  {
    fieldType = UNKNOWN;
    ROS_ERROR(" Unknown data type: %s", fieldTypeStr.c_str());
    // Finished decoding, as we do not know how large the unknown data blob is
    messageDescriptor.str("");
    return false;
  }

  return true;
}

string const& MessageDecoder::getFieldName()
{
  return fieldName;
}

FieldTypes MessageDecoder::getFieldType()
{
  return fieldType;
}

bool MessageDecoder::getFieldString(string& value)
{
  if (fieldType == String)
  {
    value = fieldValueString;
    return true;
  }
  return false;
}

bool MessageDecoder::getFieldInt64(int64_t& value)
{
  if (fieldType == Int64)
  {
    value = fieldValueInt;
    return true;
  }
  return false;
}

bool MessageDecoder::getFieldInt32(int32_t& value)
{
  if (fieldType == Int32)
  {
    value = (int32_t)fieldValueInt;
    return true;
  }
  return false;
}

bool MessageDecoder::getFieldInt16(int16_t& value)
{
  if (fieldType == Int16)
  {
    value = (int16_t)fieldValueInt;
    return true;
  }
  return false;
}

bool MessageDecoder::getFieldInt8(int8_t& value)
{
  if (fieldType == Int8)
  {
    value = (int8_t)fieldValueInt;
    return true;
  }
  return false;
}

bool MessageDecoder::getFieldBool(bool& value)
{
  if (fieldType == Bool)
  {
    value = (bool)fieldValueInt;
    return true;
  }
  return false;
}

void MessageDecoder::resetDecoder()
{
  messageDescriptor.clear();
  messageDescriptor.seekg(0, ios::beg);
  messagePosition = 0;
  fieldType = UNKNOWN;
}

void MessageDecoder::outputField()
{
// Show field data
  if (fieldType == String)
  {
    ROS_INFO(" (string): %s", fieldValueString.c_str());
  }
  else if (fieldType == Int64)
  {
    ROS_INFO(" (int64_t): %ld", fieldValueInt);
  }
  else if (fieldType == Int32)
  {
    ROS_INFO(" (int32_t): %d", (int32_t ) fieldValueInt);
  }
  else if (fieldType == Int16)
  {
    ROS_INFO(" (int16_t): %d", (int16_t ) fieldValueInt);
  }
  else if (fieldType == Int8)
  {
    ROS_INFO(" (int8_t): %d", (int8_t ) fieldValueInt);
  }
}
}
