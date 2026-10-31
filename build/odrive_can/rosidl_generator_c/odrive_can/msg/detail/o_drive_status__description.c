// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from odrive_can:msg/ODriveStatus.idl
// generated code does not contain a copyright notice

#include "odrive_can/msg/detail/o_drive_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_odrive_can
const rosidl_type_hash_t *
odrive_can__msg__ODriveStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x3d, 0x42, 0x7d, 0xb7, 0xa0, 0x96, 0x3b, 0x99,
      0xa4, 0xae, 0x5c, 0x17, 0x61, 0xaa, 0xd2, 0x24,
      0x9f, 0x54, 0xd5, 0xf3, 0x52, 0x69, 0x98, 0xab,
      0x37, 0x0d, 0x76, 0xd2, 0x18, 0x86, 0xfc, 0x9b,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char odrive_can__msg__ODriveStatus__TYPE_NAME[] = "odrive_can/msg/ODriveStatus";

// Define type names, field names, and default values
static char odrive_can__msg__ODriveStatus__FIELD_NAME__bus_voltage[] = "bus_voltage";
static char odrive_can__msg__ODriveStatus__FIELD_NAME__bus_current[] = "bus_current";
static char odrive_can__msg__ODriveStatus__FIELD_NAME__fet_temperature[] = "fet_temperature";
static char odrive_can__msg__ODriveStatus__FIELD_NAME__motor_temperature[] = "motor_temperature";
static char odrive_can__msg__ODriveStatus__FIELD_NAME__active_errors[] = "active_errors";
static char odrive_can__msg__ODriveStatus__FIELD_NAME__disarm_reason[] = "disarm_reason";

static rosidl_runtime_c__type_description__Field odrive_can__msg__ODriveStatus__FIELDS[] = {
  {
    {odrive_can__msg__ODriveStatus__FIELD_NAME__bus_voltage, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ODriveStatus__FIELD_NAME__bus_current, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ODriveStatus__FIELD_NAME__fet_temperature, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ODriveStatus__FIELD_NAME__motor_temperature, 17, 17},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ODriveStatus__FIELD_NAME__active_errors, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ODriveStatus__FIELD_NAME__disarm_reason, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
odrive_can__msg__ODriveStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {odrive_can__msg__ODriveStatus__TYPE_NAME, 27, 27},
      {odrive_can__msg__ODriveStatus__FIELDS, 6, 6},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 bus_voltage\n"
  "float32 bus_current\n"
  "float32 fet_temperature\n"
  "float32 motor_temperature\n"
  "uint32 active_errors\n"
  "uint32 disarm_reason";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
odrive_can__msg__ODriveStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {odrive_can__msg__ODriveStatus__TYPE_NAME, 27, 27},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 132, 132},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
odrive_can__msg__ODriveStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *odrive_can__msg__ODriveStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
