// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from nuturtle_control:srv/InitialPose.idl
// generated code does not contain a copyright notice

#include "nuturtle_control/srv/detail/initial_pose__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_nuturtle_control
const rosidl_type_hash_t *
nuturtle_control__srv__InitialPose__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xfd, 0x97, 0xba, 0xbb, 0xe4, 0x1b, 0xc8, 0xa6,
      0xc2, 0xf6, 0xe1, 0xdb, 0xbc, 0x3e, 0x31, 0x63,
      0x8b, 0x93, 0xfd, 0x24, 0x77, 0xa2, 0x4e, 0xd8,
      0x43, 0x1a, 0xb1, 0xb3, 0x7a, 0x33, 0xe5, 0xdb,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_nuturtle_control
const rosidl_type_hash_t *
nuturtle_control__srv__InitialPose_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x52, 0xac, 0x59, 0xa2, 0xf6, 0x5f, 0xb5, 0x1f,
      0xb7, 0xba, 0xf0, 0x66, 0x21, 0xc7, 0x78, 0x45,
      0x69, 0x66, 0x6d, 0x87, 0x72, 0x2a, 0xcc, 0xc5,
      0x02, 0xc7, 0x28, 0x60, 0xf8, 0x4e, 0x3c, 0x64,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_nuturtle_control
const rosidl_type_hash_t *
nuturtle_control__srv__InitialPose_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe3, 0xc8, 0x40, 0x7a, 0xdc, 0x51, 0xe8, 0x1c,
      0x94, 0xc8, 0x85, 0xe8, 0x82, 0x5d, 0x0d, 0xa3,
      0x25, 0x91, 0x53, 0x35, 0xfb, 0x9e, 0x2b, 0xf6,
      0xd8, 0xd4, 0x54, 0x5e, 0xcd, 0x6f, 0xe9, 0xf7,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_nuturtle_control
const rosidl_type_hash_t *
nuturtle_control__srv__InitialPose_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x15, 0x48, 0x92, 0x6f, 0xde, 0x35, 0x14, 0xc4,
      0x04, 0x0c, 0xd4, 0x54, 0x08, 0xe4, 0x48, 0x77,
      0x55, 0xae, 0xc8, 0x9d, 0x19, 0x28, 0xde, 0xa3,
      0xef, 0xb9, 0xbf, 0x00, 0x46, 0x6f, 0x8c, 0x13,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "nuturtle_control/msg/detail/robot_configuration__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t nuturtle_control__msg__RobotConfiguration__EXPECTED_HASH = {1, {
    0x83, 0x64, 0xa2, 0x93, 0xff, 0x18, 0x3b, 0xbb,
    0xf6, 0x64, 0xfb, 0xcd, 0x93, 0x1f, 0x31, 0x75,
    0x50, 0x18, 0x28, 0x53, 0x4c, 0xd5, 0x47, 0x06,
    0x78, 0x0c, 0xad, 0x51, 0xbf, 0xaa, 0x19, 0x09,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char nuturtle_control__srv__InitialPose__TYPE_NAME[] = "nuturtle_control/srv/InitialPose";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char nuturtle_control__msg__RobotConfiguration__TYPE_NAME[] = "nuturtle_control/msg/RobotConfiguration";
static char nuturtle_control__srv__InitialPose_Event__TYPE_NAME[] = "nuturtle_control/srv/InitialPose_Event";
static char nuturtle_control__srv__InitialPose_Request__TYPE_NAME[] = "nuturtle_control/srv/InitialPose_Request";
static char nuturtle_control__srv__InitialPose_Response__TYPE_NAME[] = "nuturtle_control/srv/InitialPose_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char nuturtle_control__srv__InitialPose__FIELD_NAME__request_message[] = "request_message";
static char nuturtle_control__srv__InitialPose__FIELD_NAME__response_message[] = "response_message";
static char nuturtle_control__srv__InitialPose__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field nuturtle_control__srv__InitialPose__FIELDS[] = {
  {
    {nuturtle_control__srv__InitialPose__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nuturtle_control__srv__InitialPose_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__srv__InitialPose__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nuturtle_control__srv__InitialPose_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__srv__InitialPose__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nuturtle_control__srv__InitialPose_Event__TYPE_NAME, 38, 38},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription nuturtle_control__srv__InitialPose__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__msg__RobotConfiguration__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__srv__InitialPose_Event__TYPE_NAME, 38, 38},
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__srv__InitialPose_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__srv__InitialPose_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nuturtle_control__srv__InitialPose__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nuturtle_control__srv__InitialPose__TYPE_NAME, 32, 32},
      {nuturtle_control__srv__InitialPose__FIELDS, 3, 3},
    },
    {nuturtle_control__srv__InitialPose__REFERENCED_TYPE_DESCRIPTIONS, 6, 6},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nuturtle_control__msg__RobotConfiguration__EXPECTED_HASH, nuturtle_control__msg__RobotConfiguration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = nuturtle_control__msg__RobotConfiguration__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = nuturtle_control__srv__InitialPose_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = nuturtle_control__srv__InitialPose_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[4].fields = nuturtle_control__srv__InitialPose_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[5].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char nuturtle_control__srv__InitialPose_Request__FIELD_NAME__q[] = "q";

static rosidl_runtime_c__type_description__Field nuturtle_control__srv__InitialPose_Request__FIELDS[] = {
  {
    {nuturtle_control__srv__InitialPose_Request__FIELD_NAME__q, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nuturtle_control__msg__RobotConfiguration__TYPE_NAME, 39, 39},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription nuturtle_control__srv__InitialPose_Request__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {nuturtle_control__msg__RobotConfiguration__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nuturtle_control__srv__InitialPose_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nuturtle_control__srv__InitialPose_Request__TYPE_NAME, 40, 40},
      {nuturtle_control__srv__InitialPose_Request__FIELDS, 1, 1},
    },
    {nuturtle_control__srv__InitialPose_Request__REFERENCED_TYPE_DESCRIPTIONS, 1, 1},
  };
  if (!constructed) {
    assert(0 == memcmp(&nuturtle_control__msg__RobotConfiguration__EXPECTED_HASH, nuturtle_control__msg__RobotConfiguration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = nuturtle_control__msg__RobotConfiguration__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char nuturtle_control__srv__InitialPose_Response__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field nuturtle_control__srv__InitialPose_Response__FIELDS[] = {
  {
    {nuturtle_control__srv__InitialPose_Response__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nuturtle_control__srv__InitialPose_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nuturtle_control__srv__InitialPose_Response__TYPE_NAME, 41, 41},
      {nuturtle_control__srv__InitialPose_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char nuturtle_control__srv__InitialPose_Event__FIELD_NAME__info[] = "info";
static char nuturtle_control__srv__InitialPose_Event__FIELD_NAME__request[] = "request";
static char nuturtle_control__srv__InitialPose_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field nuturtle_control__srv__InitialPose_Event__FIELDS[] = {
  {
    {nuturtle_control__srv__InitialPose_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__srv__InitialPose_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {nuturtle_control__srv__InitialPose_Request__TYPE_NAME, 40, 40},
    },
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__srv__InitialPose_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {nuturtle_control__srv__InitialPose_Response__TYPE_NAME, 41, 41},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription nuturtle_control__srv__InitialPose_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__msg__RobotConfiguration__TYPE_NAME, 39, 39},
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__srv__InitialPose_Request__TYPE_NAME, 40, 40},
    {NULL, 0, 0},
  },
  {
    {nuturtle_control__srv__InitialPose_Response__TYPE_NAME, 41, 41},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nuturtle_control__srv__InitialPose_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nuturtle_control__srv__InitialPose_Event__TYPE_NAME, 38, 38},
      {nuturtle_control__srv__InitialPose_Event__FIELDS, 3, 3},
    },
    {nuturtle_control__srv__InitialPose_Event__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&nuturtle_control__msg__RobotConfiguration__EXPECTED_HASH, nuturtle_control__msg__RobotConfiguration__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[1].fields = nuturtle_control__msg__RobotConfiguration__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = nuturtle_control__srv__InitialPose_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = nuturtle_control__srv__InitialPose_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "# the configuration of the robot in the world frame\n"
  "RobotConfiguration q\n"
  "---";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
nuturtle_control__srv__InitialPose__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nuturtle_control__srv__InitialPose__TYPE_NAME, 32, 32},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 76, 76},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
nuturtle_control__srv__InitialPose_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nuturtle_control__srv__InitialPose_Request__TYPE_NAME, 40, 40},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
nuturtle_control__srv__InitialPose_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nuturtle_control__srv__InitialPose_Response__TYPE_NAME, 41, 41},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
nuturtle_control__srv__InitialPose_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nuturtle_control__srv__InitialPose_Event__TYPE_NAME, 38, 38},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nuturtle_control__srv__InitialPose__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[7];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 7, 7};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nuturtle_control__srv__InitialPose__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *nuturtle_control__msg__RobotConfiguration__get_individual_type_description_source(NULL);
    sources[3] = *nuturtle_control__srv__InitialPose_Event__get_individual_type_description_source(NULL);
    sources[4] = *nuturtle_control__srv__InitialPose_Request__get_individual_type_description_source(NULL);
    sources[5] = *nuturtle_control__srv__InitialPose_Response__get_individual_type_description_source(NULL);
    sources[6] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nuturtle_control__srv__InitialPose_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[2];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 2, 2};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nuturtle_control__srv__InitialPose_Request__get_individual_type_description_source(NULL),
    sources[1] = *nuturtle_control__msg__RobotConfiguration__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nuturtle_control__srv__InitialPose_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nuturtle_control__srv__InitialPose_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nuturtle_control__srv__InitialPose_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nuturtle_control__srv__InitialPose_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *nuturtle_control__msg__RobotConfiguration__get_individual_type_description_source(NULL);
    sources[3] = *nuturtle_control__srv__InitialPose_Request__get_individual_type_description_source(NULL);
    sources[4] = *nuturtle_control__srv__InitialPose_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
