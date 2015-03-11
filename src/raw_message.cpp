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
#include"raw_message/raw_message.h"

namespace raw_message
{
  const char* toString(FieldTypes type)
  {
    switch(type)
    {
      case String: return "String";
      case Int64: return "Int64";
      case Int32: return "Int32";
      case Int16: return "Int16";
      case Int8: return "Int8";
    }
    return "UNKNOWN";
  }
}
