// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from nusim:srv/Teleport.idl
// generated code does not contain a copyright notice

#include "nusim/srv/detail/teleport__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_nusim
const rosidl_type_hash_t *
nusim__srv__Teleport__get_type_hash(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3d, 0x11, 0xa6, 0xea, 0x5c, 0x7a, 0x4d, 0xbf,
      0x01, 0xb6, 0xcb, 0x67, 0x95, 0xc9, 0x94, 0xa8,
      0x17, 0xfb, 0x53, 0x09, 0x59, 0xae, 0x93, 0x72,
      0x9c, 0xb5, 0x5e, 0x1b, 0xd4, 0x5f, 0x50, 0xe9,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_nusim
const rosidl_type_hash_t *
nusim__srv__Teleport_Request__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xdf, 0xec, 0xef, 0x97, 0x94, 0xb1, 0x11, 0x77,
      0x4b, 0x10, 0x70, 0xdb, 0x91, 0xdc, 0xd4, 0x8a,
      0x0d, 0x02, 0x07, 0xbe, 0x4e, 0x10, 0x0e, 0xdb,
      0x35, 0x54, 0xda, 0xec, 0x0a, 0xc2, 0xd9, 0x0e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_nusim
const rosidl_type_hash_t *
nusim__srv__Teleport_Response__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xe8, 0x0f, 0x18, 0x26, 0x29, 0x3a, 0x52, 0xa2,
      0x59, 0x6e, 0x79, 0xef, 0xed, 0x80, 0x76, 0x29,
      0x1d, 0xb6, 0xe7, 0x9b, 0x78, 0x9a, 0xf8, 0xd2,
      0xd7, 0x1c, 0xae, 0x46, 0xeb, 0xc0, 0x86, 0x5e,
    }};
  return &hash;
}

ROSIDL_GENERATOR_C_PUBLIC_nusim
const rosidl_type_hash_t *
nusim__srv__Teleport_Event__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x72, 0xea, 0xf9, 0x94, 0xd4, 0x5c, 0x33, 0xd8,
      0x86, 0xc7, 0xa8, 0x0e, 0x79, 0x3c, 0xac, 0x9a,
      0xcc, 0xcb, 0x42, 0xe8, 0xf5, 0x00, 0x70, 0x7d,
      0xba, 0x09, 0x88, 0x29, 0x32, 0x92, 0x31, 0x9f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types
#include "service_msgs/msg/detail/service_event_info__functions.h"
#include "builtin_interfaces/msg/detail/time__functions.h"

// Hashes for external referenced types
#ifndef NDEBUG
static const rosidl_type_hash_t builtin_interfaces__msg__Time__EXPECTED_HASH = {1, {
    0xb1, 0x06, 0x23, 0x5e, 0x25, 0xa4, 0xc5, 0xed,
    0x35, 0x09, 0x8a, 0xa0, 0xa6, 0x1a, 0x3e, 0xe9,
    0xc9, 0xb1, 0x8d, 0x19, 0x7f, 0x39, 0x8b, 0x0e,
    0x42, 0x06, 0xce, 0xa9, 0xac, 0xf9, 0xc1, 0x97,
  }};
static const rosidl_type_hash_t service_msgs__msg__ServiceEventInfo__EXPECTED_HASH = {1, {
    0x41, 0xbc, 0xbb, 0xe0, 0x7a, 0x75, 0xc9, 0xb5,
    0x2b, 0xc9, 0x6b, 0xfd, 0x5c, 0x24, 0xd7, 0xf0,
    0xfc, 0x0a, 0x08, 0xc0, 0xcb, 0x79, 0x21, 0xb3,
    0x37, 0x3c, 0x57, 0x32, 0x34, 0x5a, 0x6f, 0x45,
  }};
#endif

static char nusim__srv__Teleport__TYPE_NAME[] = "nusim/srv/Teleport";
static char builtin_interfaces__msg__Time__TYPE_NAME[] = "builtin_interfaces/msg/Time";
static char nusim__srv__Teleport_Event__TYPE_NAME[] = "nusim/srv/Teleport_Event";
static char nusim__srv__Teleport_Request__TYPE_NAME[] = "nusim/srv/Teleport_Request";
static char nusim__srv__Teleport_Response__TYPE_NAME[] = "nusim/srv/Teleport_Response";
static char service_msgs__msg__ServiceEventInfo__TYPE_NAME[] = "service_msgs/msg/ServiceEventInfo";

// Define type names, field names, and default values
static char nusim__srv__Teleport__FIELD_NAME__request_message[] = "request_message";
static char nusim__srv__Teleport__FIELD_NAME__response_message[] = "response_message";
static char nusim__srv__Teleport__FIELD_NAME__event_message[] = "event_message";

static rosidl_runtime_c__type_description__Field nusim__srv__Teleport__FIELDS[] = {
  {
    {nusim__srv__Teleport__FIELD_NAME__request_message, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nusim__srv__Teleport_Request__TYPE_NAME, 26, 26},
    },
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport__FIELD_NAME__response_message, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nusim__srv__Teleport_Response__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport__FIELD_NAME__event_message, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {nusim__srv__Teleport_Event__TYPE_NAME, 24, 24},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription nusim__srv__Teleport__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport_Event__TYPE_NAME, 24, 24},
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport_Request__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport_Response__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nusim__srv__Teleport__get_type_description(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nusim__srv__Teleport__TYPE_NAME, 18, 18},
      {nusim__srv__Teleport__FIELDS, 3, 3},
    },
    {nusim__srv__Teleport__REFERENCED_TYPE_DESCRIPTIONS, 5, 5},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = nusim__srv__Teleport_Event__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = nusim__srv__Teleport_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[3].fields = nusim__srv__Teleport_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[4].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char nusim__srv__Teleport_Request__FIELD_NAME__x[] = "x";
static char nusim__srv__Teleport_Request__FIELD_NAME__y[] = "y";
static char nusim__srv__Teleport_Request__FIELD_NAME__theta[] = "theta";

static rosidl_runtime_c__type_description__Field nusim__srv__Teleport_Request__FIELDS[] = {
  {
    {nusim__srv__Teleport_Request__FIELD_NAME__x, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport_Request__FIELD_NAME__y, 1, 1},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport_Request__FIELD_NAME__theta, 5, 5},
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
nusim__srv__Teleport_Request__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nusim__srv__Teleport_Request__TYPE_NAME, 26, 26},
      {nusim__srv__Teleport_Request__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char nusim__srv__Teleport_Response__FIELD_NAME__structure_needs_at_least_one_member[] = "structure_needs_at_least_one_member";

static rosidl_runtime_c__type_description__Field nusim__srv__Teleport_Response__FIELDS[] = {
  {
    {nusim__srv__Teleport_Response__FIELD_NAME__structure_needs_at_least_one_member, 35, 35},
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
nusim__srv__Teleport_Response__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nusim__srv__Teleport_Response__TYPE_NAME, 27, 27},
      {nusim__srv__Teleport_Response__FIELDS, 1, 1},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}
// Define type names, field names, and default values
static char nusim__srv__Teleport_Event__FIELD_NAME__info[] = "info";
static char nusim__srv__Teleport_Event__FIELD_NAME__request[] = "request";
static char nusim__srv__Teleport_Event__FIELD_NAME__response[] = "response";

static rosidl_runtime_c__type_description__Field nusim__srv__Teleport_Event__FIELDS[] = {
  {
    {nusim__srv__Teleport_Event__FIELD_NAME__info, 4, 4},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE,
      0,
      0,
      {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    },
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport_Event__FIELD_NAME__request, 7, 7},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {nusim__srv__Teleport_Request__TYPE_NAME, 26, 26},
    },
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport_Event__FIELD_NAME__response, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_NESTED_TYPE_BOUNDED_SEQUENCE,
      1,
      0,
      {nusim__srv__Teleport_Response__TYPE_NAME, 27, 27},
    },
    {NULL, 0, 0},
  },
};

static rosidl_runtime_c__type_description__IndividualTypeDescription nusim__srv__Teleport_Event__REFERENCED_TYPE_DESCRIPTIONS[] = {
  {
    {builtin_interfaces__msg__Time__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport_Request__TYPE_NAME, 26, 26},
    {NULL, 0, 0},
  },
  {
    {nusim__srv__Teleport_Response__TYPE_NAME, 27, 27},
    {NULL, 0, 0},
  },
  {
    {service_msgs__msg__ServiceEventInfo__TYPE_NAME, 33, 33},
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
nusim__srv__Teleport_Event__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {nusim__srv__Teleport_Event__TYPE_NAME, 24, 24},
      {nusim__srv__Teleport_Event__FIELDS, 3, 3},
    },
    {nusim__srv__Teleport_Event__REFERENCED_TYPE_DESCRIPTIONS, 4, 4},
  };
  if (!constructed) {
    assert(0 == memcmp(&builtin_interfaces__msg__Time__EXPECTED_HASH, builtin_interfaces__msg__Time__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[0].fields = builtin_interfaces__msg__Time__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[1].fields = nusim__srv__Teleport_Request__get_type_description(NULL)->type_description.fields;
    description.referenced_type_descriptions.data[2].fields = nusim__srv__Teleport_Response__get_type_description(NULL)->type_description.fields;
    assert(0 == memcmp(&service_msgs__msg__ServiceEventInfo__EXPECTED_HASH, service_msgs__msg__ServiceEventInfo__get_type_hash(NULL), sizeof(rosidl_type_hash_t)));
    description.referenced_type_descriptions.data[3].fields = service_msgs__msg__ServiceEventInfo__get_type_description(NULL)->type_description.fields;
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64 x\n"
  "float64 y\n"
  "float64 theta\n"
  "---";

static char srv_encoding[] = "srv";
static char implicit_encoding[] = "implicit";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
nusim__srv__Teleport__get_individual_type_description_source(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nusim__srv__Teleport__TYPE_NAME, 18, 18},
    {srv_encoding, 3, 3},
    {toplevel_type_raw_source, 38, 38},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
nusim__srv__Teleport_Request__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nusim__srv__Teleport_Request__TYPE_NAME, 26, 26},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
nusim__srv__Teleport_Response__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nusim__srv__Teleport_Response__TYPE_NAME, 27, 27},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource *
nusim__srv__Teleport_Event__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {nusim__srv__Teleport_Event__TYPE_NAME, 24, 24},
    {implicit_encoding, 8, 8},
    {NULL, 0, 0},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nusim__srv__Teleport__get_type_description_sources(
  const rosidl_service_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[6];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 6, 6};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nusim__srv__Teleport__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *nusim__srv__Teleport_Event__get_individual_type_description_source(NULL);
    sources[3] = *nusim__srv__Teleport_Request__get_individual_type_description_source(NULL);
    sources[4] = *nusim__srv__Teleport_Response__get_individual_type_description_source(NULL);
    sources[5] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nusim__srv__Teleport_Request__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nusim__srv__Teleport_Request__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nusim__srv__Teleport_Response__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nusim__srv__Teleport_Response__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
nusim__srv__Teleport_Event__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[5];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 5, 5};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *nusim__srv__Teleport_Event__get_individual_type_description_source(NULL),
    sources[1] = *builtin_interfaces__msg__Time__get_individual_type_description_source(NULL);
    sources[2] = *nusim__srv__Teleport_Request__get_individual_type_description_source(NULL);
    sources[3] = *nusim__srv__Teleport_Response__get_individual_type_description_source(NULL);
    sources[4] = *service_msgs__msg__ServiceEventInfo__get_individual_type_description_source(NULL);
    constructed = true;
  }
  return &source_sequence;
}
