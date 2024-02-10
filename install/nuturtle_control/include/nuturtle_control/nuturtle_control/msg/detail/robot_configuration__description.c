// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from nuturtle_control:msg/RobotConfiguration.idl
// generated code does not contain a copyright notice

#include "nuturtle_control/msg/detail/robot_configuration__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_nuturtle_control
const rosidl_type_hash_t *
nuturtle_control__msg__RobotConfiguration__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x83, 0x64, 0xa2, 0x93, 0xff, 0x18, 0x3b, 0xbb,
      0xf6, 0x64, 0xfb, 0xcd, 0x93, 0x1f, 0x31, 0x75,
      0x50, 0x18, 0x28, 0x53, 0x4c, 0xd5, 0x47, 0x06,
      0x78, 0x0c, 0xad, 0x51, 0xbf, 0xaa, 0x19, 0x09,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char nuturtle_control__msg__RobotConfiguration__TYPE_NAME[] = "nuturtle_control/msg/RobotConfiguration";

// Define type names, field names, and default values
static char nuturtle_control__msg__RobotConfiguration__FIELD_NAME__theta[] = "theta";
static char nuturtle_control__msg__RobotConfiguration__FIELD_NAME__x[] = "x";
static char nuturtle_control__msg__RobotConfiguration__FIELD_NAME__y[] = "y";

static rosidl_runtime_c__type_description__Field nuturtle_control__msg__RobotConfiguration__FIELDS[] = {
  {
    {nuturtle_control__msg__RobotConfiguration__FIELD_NAME__theta, 5, 5},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__msg__RobotConfiguration__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__msg__RobotConfiguration__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nuturtle_control__msg__RobotConfiguration__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nuturtle_control__msg__RobotConfiguration__TYPE_NAME, 39, 39},
      {nuturtle_control__msg__RobotConfiguration__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# the angle of the robot relative to the world frame\n"
  "float64 theta\n"
  "# the x position of the robot relative to the world frame\n"
  "float64 x\n"
  "# the y position of the robot relative to the world frame\n"
  "float64 y";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
nuturtle_control__msg__RobotConfiguration__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nuturtle_control__msg__RobotConfiguration__TYPE_NAME, 39, 39},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 202, 202},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nuturtle_control__msg__RobotConfiguration__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nuturtle_control__msg__RobotConfiguration__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
