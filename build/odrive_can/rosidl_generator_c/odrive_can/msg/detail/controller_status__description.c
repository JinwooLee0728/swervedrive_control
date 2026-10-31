// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from odrive_can:msg/ControllerStatus.idl
// generated code does not contain a copyright notice

#include "odrive_can/msg/detail/controller_status__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_odrive_can
const rosidl_type_hash_t *
odrive_can__msg__ControllerStatus__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0xc9, 0x48, 0xa4, 0x45, 0x53, 0x2f, 0x05, 0x5b,
      0x24, 0x7d, 0xc1, 0x97, 0x26, 0xdd, 0x14, 0xd5,
      0xbb, 0xb3, 0x62, 0x8d, 0xae, 0xb8, 0xca, 0xc3,
      0x87, 0x9c, 0xaf, 0x44, 0xa9, 0x31, 0x2f, 0x43,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char odrive_can__msg__ControllerStatus__TYPE_NAME[] = "odrive_can/msg/ControllerStatus";

// Define type names, field names, and default values
static char odrive_can__msg__ControllerStatus__FIELD_NAME__pos_estimate[] = "pos_estimate";
static char odrive_can__msg__ControllerStatus__FIELD_NAME__vel_estimate[] = "vel_estimate";
static char odrive_can__msg__ControllerStatus__FIELD_NAME__torque_target[] = "torque_target";
static char odrive_can__msg__ControllerStatus__FIELD_NAME__torque_estimate[] = "torque_estimate";
static char odrive_can__msg__ControllerStatus__FIELD_NAME__iq_setpoint[] = "iq_setpoint";
static char odrive_can__msg__ControllerStatus__FIELD_NAME__iq_measured[] = "iq_measured";
static char odrive_can__msg__ControllerStatus__FIELD_NAME__active_errors[] = "active_errors";
static char odrive_can__msg__ControllerStatus__FIELD_NAME__axis_state[] = "axis_state";
static char odrive_can__msg__ControllerStatus__FIELD_NAME__procedure_result[] = "procedure_result";
static char odrive_can__msg__ControllerStatus__FIELD_NAME__trajectory_done_flag[] = "trajectory_done_flag";

static rosidl_runtime_c__type_description__Field odrive_can__msg__ControllerStatus__FIELDS[] = {
  {
    {odrive_can__msg__ControllerStatus__FIELD_NAME__pos_estimate, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControllerStatus__FIELD_NAME__vel_estimate, 12, 12},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControllerStatus__FIELD_NAME__torque_target, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControllerStatus__FIELD_NAME__torque_estimate, 15, 15},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControllerStatus__FIELD_NAME__iq_setpoint, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControllerStatus__FIELD_NAME__iq_measured, 11, 11},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_FLOAT,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControllerStatus__FIELD_NAME__active_errors, 13, 13},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT32,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControllerStatus__FIELD_NAME__axis_state, 10, 10},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControllerStatus__FIELD_NAME__procedure_result, 16, 16},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_UINT8,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {odrive_can__msg__ControllerStatus__FIELD_NAME__trajectory_done_flag, 20, 20},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_BOOLEAN,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
odrive_can__msg__ControllerStatus__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {odrive_can__msg__ControllerStatus__TYPE_NAME, 31, 31},
      {odrive_can__msg__ControllerStatus__FIELDS, 10, 10},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float32 pos_estimate\n"
  "float32 vel_estimate\n"
  "float32 torque_target\n"
  "float32 torque_estimate\n"
  "float32 iq_setpoint\n"
  "float32 iq_measured\n"
  "uint32 active_errors\n"
  "uint8 axis_state\n"
  "uint8 procedure_result\n"
  "bool trajectory_done_flag";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
odrive_can__msg__ControllerStatus__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {odrive_can__msg__ControllerStatus__TYPE_NAME, 31, 31},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 215, 215},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
odrive_can__msg__ControllerStatus__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *odrive_can__msg__ControllerStatus__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
