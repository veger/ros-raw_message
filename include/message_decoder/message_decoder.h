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

/**
 * Class to handle decoding ROS topic messages without having prior knowledge about the actual type.
 *
 * Subscribe to any topic using the ShapeShifter message type and feed the received messages into this class
 */
class MessageDecoder
{
public:
  /** Decode the given message by printing all of its field separately */
  void decodeMessage(boost::shared_ptr<topic_tools::ShapeShifter const> const &msg);

private:

  /**
   * Decode the next field
   * @return false if the decoder is finished
   */
  bool decodeField();

  std::stringstream messageDescriptor;

  /** Position in messageData were the decoder is */
  uint32_t messagePosition;

  /** Binary data blob of the message that is being decoded */
  uint8_t messageData[256];
};
}
