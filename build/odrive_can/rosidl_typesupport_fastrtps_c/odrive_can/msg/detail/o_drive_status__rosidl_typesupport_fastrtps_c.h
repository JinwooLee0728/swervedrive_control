// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from odrive_can:msg/ODriveStatus.idl
// generated code does not contain a copyright notice
#ifndef ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "odrive_can/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "odrive_can/msg/detail/o_drive_status__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
bool cdr_serialize_odrive_can__msg__ODriveStatus(
  const odrive_can__msg__ODriveStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
bool cdr_deserialize_odrive_can__msg__ODriveStatus(
  eprosima::fastcdr::Cdr &,
  odrive_can__msg__ODriveStatus * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
size_t get_serialized_size_odrive_can__msg__ODriveStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
size_t max_serialized_size_odrive_can__msg__ODriveStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
bool cdr_serialize_key_odrive_can__msg__ODriveStatus(
  const odrive_can__msg__ODriveStatus * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
size_t get_serialized_size_key_odrive_can__msg__ODriveStatus(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
size_t max_serialized_size_key_odrive_can__msg__ODriveStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_odrive_can
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, odrive_can, msg, ODriveStatus)();

#ifdef __cplusplus
}
#endif

#endif  // ODRIVE_CAN__MSG__DETAIL__O_DRIVE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
