// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// point_field_conversion.h
//
// Created on: 16.07.2015
// Authors: Sebastian Pütz <spuetz@uni-osnabrueck.de>

#ifndef SENSOR_MSGS_POINT_FIELD_CONVERSION_HPP_  // NOLINT
#define SENSOR_MSGS_POINT_FIELD_CONVERSION_HPP_  // NOLINT

#include <stdio.h>
/**
 * \brief  This file provides a type to enum mapping for the different
 *         PointField types and methods to read and write in
 *         a PointCloud2 buffer for the different PointField types.
 * \author Sebastian Pütz
 */
namespace sensor_msgs
{
/*!
 * \Enum to type mapping.
 */
template<int>
struct pointFieldTypeAsType {};
template<>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::INT8>
{
  typedef int8_t type;
};
template<>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::UINT8>
{
  typedef uint8_t type;
};
template<>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::INT16>
{
  typedef int16_t type;
};
template<>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::UINT16>
{
  typedef uint16_t type;
};
template<>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::INT32>
{
  typedef int32_t type;
};
template<>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::UINT32>
{
  typedef uint32_t type;
};
template<>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::FLOAT32>
{
  typedef float type;
};
template<>
struct pointFieldTypeAsType<sensor_msgs::msg::PointField::FLOAT64>
{
  typedef double type;
};

/*!
 * \Type to enum mapping.
 */
template<typename T>
struct typeAsPointFieldType {};
template<>
struct typeAsPointFieldType<int8_t>
{
  static const uint8_t value = sensor_msgs::msg::PointField::INT8;
};
template<>
struct typeAsPointFieldType<uint8_t>
{
  static const uint8_t value = sensor_msgs::msg::PointField::UINT8;
};
template<>
struct typeAsPointFieldType<int16_t>
{
  static const uint8_t value = sensor_msgs::msg::PointField::INT16;
};
template<>
struct typeAsPointFieldType<uint16_t>
{
  static const uint8_t value = sensor_msgs::msg::PointField::UINT16;
};
template<>
struct typeAsPointFieldType<int32_t>
{
  static const uint8_t value = sensor_msgs::msg::PointField::INT32;
};
template<>
struct typeAsPointFieldType<uint32_t>
{
  static const uint8_t value = sensor_msgs::msg::PointField::UINT32;
};
template<>
struct typeAsPointFieldType<float>
{
  static const uint8_t value = sensor_msgs::msg::PointField::FLOAT32;
};
template<>
struct typeAsPointFieldType<double>
{
  static const uint8_t value = sensor_msgs::msg::PointField::FLOAT64;
};

/*!
 * \Converts a value at the given pointer position, interpreted as the datatype
 *  specified by the given template argument point_field_type, to the given
 *  template type T and returns it.
 * \param data_ptr            pointer into the point cloud 2 buffer
 * \tparam point_field_type   sensor_msgs::msg::PointField datatype value
 * \tparam T                  return type
 */
template<int point_field_type, typename T>
inline T readPointCloud2BufferValue(const unsigned char * data_ptr)
{
  typedef typename pointFieldTypeAsType<point_field_type>::type type;
  return static_cast<T>(*(reinterpret_cast<type const *>(data_ptr)));
}

/*!
 * \Converts a value at the given pointer position interpreted as the datatype
 *  specified by the given datatype parameter to the given template type and
 * returns it. \param data_ptr    pointer into the point cloud 2 buffer \param
 * datatype    sensor_msgs::msg::PointField datatype value \tparam T return type
 */
template<typename T>
inline T readPointCloud2BufferValue(
  const unsigned char * data_ptr,
  const unsigned char datatype)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::INT8, T>(
        data_ptr);
    case sensor_msgs::msg::PointField::UINT8:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::UINT8, T>(
        data_ptr);
    case sensor_msgs::msg::PointField::INT16:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::INT16, T>(
        data_ptr);
    case sensor_msgs::msg::PointField::UINT16:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::UINT16, T>(
        data_ptr);
    case sensor_msgs::msg::PointField::INT32:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::INT32, T>(
        data_ptr);
    case sensor_msgs::msg::PointField::UINT32:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::UINT32, T>(
        data_ptr);
    case sensor_msgs::msg::PointField::FLOAT32:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::FLOAT32, T>(
        data_ptr);
    case sensor_msgs::msg::PointField::FLOAT64:
      return readPointCloud2BufferValue<sensor_msgs::msg::PointField::FLOAT64, T>(
        data_ptr);
  }
  // This should never be reached, but return statement added to avoid compiler
  // warning. (#84)
  return T();
}

/*!
 * \Inserts a given value at the given point position interpreted as the
 * datatype specified by the template argument point_field_type. \param data_ptr
 * pointer into the point cloud 2 buffer \param value               the value to
 * insert \tparam point_field_type   sensor_msgs::msg::PointField datatype value
 * \tparam T                  type of the value to insert
 */
template<int point_field_type, typename T>
inline void writePointCloud2BufferValue(unsigned char * data_ptr, T value)
{
  typedef typename pointFieldTypeAsType<point_field_type>::type type;
  *(reinterpret_cast<type *>(data_ptr)) = static_cast<type>(value);
}

/*!
 * \Inserts a given value at the given point position interpreted as the
 * datatype specified by the given datatype parameter. \param data_ptr pointer
 * into the point cloud 2 buffer \param datatype    sensor_msgs::msg::PointField
 * datatype value \param value       the value to insert \tparam T          type
 * of the value to insert
 */
template<typename T>
inline void writePointCloud2BufferValue(
  unsigned char * data_ptr,
  const unsigned char datatype, T value)
{
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
      writePointCloud2BufferValue<sensor_msgs::msg::PointField::INT8, T>(data_ptr,
        value);
      break;
    case sensor_msgs::msg::PointField::UINT8:
      writePointCloud2BufferValue<sensor_msgs::msg::PointField::UINT8, T>(
        data_ptr, value);
      break;
    case sensor_msgs::msg::PointField::INT16:
      writePointCloud2BufferValue<sensor_msgs::msg::PointField::INT16, T>(
        data_ptr, value);
      break;
    case sensor_msgs::msg::PointField::UINT16:
      writePointCloud2BufferValue<sensor_msgs::msg::PointField::UINT16, T>(
        data_ptr, value);
      break;
    case sensor_msgs::msg::PointField::INT32:
      writePointCloud2BufferValue<sensor_msgs::msg::PointField::INT32, T>(
        data_ptr, value);
      break;
    case sensor_msgs::msg::PointField::UINT32:
      writePointCloud2BufferValue<sensor_msgs::msg::PointField::UINT32, T>(
        data_ptr, value);
      break;
    case sensor_msgs::msg::PointField::FLOAT32:
      writePointCloud2BufferValue<sensor_msgs::msg::PointField::FLOAT32, T>(
        data_ptr, value);
      break;
    case sensor_msgs::msg::PointField::FLOAT64:
      writePointCloud2BufferValue<sensor_msgs::msg::PointField::FLOAT64, T>(
        data_ptr, value);
      break;
  }
}
}  // namespace sensor_msgs

#endif  // SENSOR_MSGS_POINT_FIELD_CONVERSION_HPP_  // NOLINT
