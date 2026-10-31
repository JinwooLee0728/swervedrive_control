// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from odrive_can:msg/ControlMessage.idl
// generated code does not contain a copyright notice

#include "odrive_can/msg/detail/control_message__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_odrive_can
const rosidl_type_hash_t *
odrive_can__msg__ControlMessage__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x85, 0x8a, 0x13, 0xbd, 0x4f, 0x1a, 0x6e, 0x57,
      0x0b, 0xf7, 0x96, 0xd6, 0xb1, 0xab, 0xd7, 0x41,
      0xd6, 0x29, 0x00, 0x42, 0x37, 0xee, 0xbd, 0x46,
      0x1d, 0xa5, 0xa5, 0xde, 0xb1, 0x87, 0xc6, 0xcb,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char odrive_can__msg__ControlMessage__TYPE_NAME[] = "odrive_can/msg/ControlMessage";

// Define type names, field names, and default values
static char odrive_can__msg__ControlMessage__FIELD_NAME__control_mode[] = "control_mode";
static char odrive_can__msg__ControlMessage__FIELD_NAME__input_mode[] = "input_mode";
static char odrive_can__msg__ControlMessage__FIELD_NAME__input_pos[] = "input_pos";
static char odrive_can__msg__ControlMessage__FIELD_NAME__input_vel[] = "input_vel";
static char odrive_can__msg__ControlMessage__FIELD_NAME__input_torque[] = "input_torque";

static rosidl_runtime_c__type_description__Field odrive_can__msg__ControlMessage__FIELDS[] = {
  {
    {odrive_can__msg__ControlMessage__FIELD_NAME__control_mode, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControlMessage__FIELD_NAME__input_mode, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControlMessage__FIELD_NAME__input_pos, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControlMessage__FIELD_NAME__input_vel, 9, 9},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControlMessage__FIELD_NAME__input_torque, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
odrive_can__msg__ControlMessage__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {odrive_can__msg__ControlMessage__TYPE_NAME, 29, 29},
      {odrive_can__msg__ControlMessage__FIELDS, 5, 5},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "uint32 control_mode\n"
  "uint32 input_mode\n"
  "float32 input_pos\n"
  "float32 input_vel\n"
  "float32 input_torque";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
odrive_can__msg__ControlMessage__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {odrive_can__msg__ControlMessage__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 95, 95},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
odrive_can__msg__ControlMessage__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *odrive_can__msg__ControlMessage__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
