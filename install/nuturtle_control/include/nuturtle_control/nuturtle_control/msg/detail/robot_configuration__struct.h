// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from nuturtle_control:msg/RobotConfiguration.idl
// generated code does not contain a copyright notice

#ifndef NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__STRUCT_H_
#define NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/RobotConfiguration in the package nuturtle_control.
/**
  * the angle of the robot relative to the world frame
 */
typedef struct nuturtle_control__msg__RobotConfiguration
{
  double theta;
  /// the x position of the robot relative to the world frame
  double x;
  /// the y position of the robot relative to the world frame
  double y;
} nuturtle_control__msg__RobotConfiguration;

// Struct for a sequence of nuturtle_control__msg__RobotConfiguration.
typedef struct nuturtle_control__msg__RobotConfiguration__Sequence
{
  nuturtle_control__msg__RobotConfiguration * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} nuturtle_control__msg__RobotConfiguration__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // NUTURTLE_CONTROL__MSG__DETAIL__ROBOT_CONFIGURATION__STRUCT_H_
