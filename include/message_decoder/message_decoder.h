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
#pragma once

#include <string>

#include "topic_tools/shape_shifter.h"

namespace message_decoder
{

/** Supported message field types */
enum FieldTypes
{
  String, Int64, Int32, Int16, Int8, UNKNOWN
};

/**
 * Class to handle decoding ROS topic messages without having prior knowledge about the actual type.
 *
 * Subscribe to any topic using the ShapeShifter message type and provide the messages to the {@link #startDecodingMessage()) function.
 * Next the {@link decodeNextField()} function can be used to iterate over the fields and obtain their information, until it returns false.
 */
class MessageDecoder
{
public:
  MessageDecoder();

  /** Start to decode the given message */
  void startDecodingMessage(boost::shared_ptr<topic_tools::ShapeShifter const> const &msg);

  /**
   * Decode the next field
   * @return false if the decoder is finished
   */
  bool decodeNextField();

  /** Display the value of the current field */
  void outputField();

  /** @return the name of the current field */
  std::string const& getFieldName();

  /** @return the type of the current field */
  FieldTypes getFieldType();

  /**
   * Obtains the string value of the current field. If the function returns false the current
   * field type was not a string and value is not updated.
   *
   * @param value to copy the string into (only if the function returns true)
   * @return true if the current field contains a string value
   */
  bool getFieldString(std::string& value);

  /**
   * Obtains the Int64 value of the current field. If the function returns false the current
   * field type was not a string and value is not updated.
   *
   * @param value to copy the string into (only if the function returns true)
   * @return true if the current field contains an Int64 value
   */
  bool getFieldInt64(int64_t& value);

  /**
   * Obtains the Int64 value of the current field. If the function returns false the current
   * field type was not a string and value is not updated.
   *
   * @param value to copy the string into (only if the function returns true)
   * @return true if the current field contains an Int32 value
   */
  bool getFieldInt32(int32_t& value);

  /**
   * Obtains the Int64 value of the current field. If the function returns false the current
   * field type was not a string and value is not updated.
   *
   * @param value to copy the string into (only if the function returns true)
   * @return true if the current field contains an Int16 value
   */
  bool getFieldInt16(int16_t& value);

  /**
   * Obtains the Int64 value of the current field. If the function returns false the current
   * field type was not a string and value is not updated.
   *
   * @param value to copy the string into (only if the function returns true)
   * @return true if the current field contains an Int8 value
   */
  bool getFieldInt8(int8_t& value);

  /** Reset the decoder, i.e. decoder behaves as if {@link startDecodingMessage()} just got called. */
  void resetDecoder();
private:

  std::stringstream messageDescriptor;

  /** Position in messageData were the decoder is */
  uint32_t messagePosition;

  /** Binary data blob of the message that is being decoded */
  uint8_t messageData[256];

  /** Type of current field */
  FieldTypes fieldType;

  /** Name of current field */
  std::string fieldName;

  /** Value of the current field assuming it is a string, otherwise undefined */
  std::string fieldValueString;

  /** Value of the current field assuming it is integer based, otherwise undefined */
  uint64_t fieldValueInt;
};
}
