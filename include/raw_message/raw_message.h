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

namespace raw_message
{
  /** Supported raw_message field types */
  enum FieldTypes
  {
    String, Int64, Int32, Int16, Int8, Bool, UNKNOWN
  };

  /** Convert the FieldTypes enum value into a string */
  const char* toString(FieldTypes type);
}
