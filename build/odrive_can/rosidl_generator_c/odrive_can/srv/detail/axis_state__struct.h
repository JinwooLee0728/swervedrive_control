// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from odrive_can:srv/AxisState.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "odrive_can/srv/axis_state.h"


#ifndef ODRIVE_CAN__SRV__DETAIL__AXIS_STATE__STRUCT_H_
#define ODRIVE_CAN__SRV__DETAIL__AXIS_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/AxisState in the package odrive_can.
typedef struct odrive_can__srv__AxisState_Request
{
  uint32_t axis_requested_state;
} odrive_can__srv__AxisState_Request;

// Struct for a sequence of odrive_can__srv__AxisState_Request.
typedef struct odrive_can__srv__AxisState_Request__Sequence
{
  odrive_can__srv__AxisState_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} odrive_can__srv__AxisState_Request__Sequence;

// Constants defined in the message

/// Struct defined in srv/AxisState in the package odrive_can.
typedef struct odrive_can__srv__AxisState_Response
{
  /// Bitfield of active errors at the time the service returns.
  /// See https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.active_errors
  uint32_t active_errors;
  /// The axis state at the time the service returns.
  /// See https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.current_state
  uint8_t axis_state;
  /// Result of the axis state or state transition (0 = SUCCESS).
  /// See https://docs.odriverobotics.com/v/latest/fibre_types/com_odriverobotics_ODrive.html#ODrive.Axis.procedure_result
  uint8_t procedure_result;
} odrive_can__srv__AxisState_Response;

// Struct for a sequence of odrive_can__srv__AxisState_Response.
typedef struct odrive_can__srv__AxisState_Response__Sequence
{
  odrive_can__srv__AxisState_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} odrive_can__srv__AxisState_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  odrive_can__srv__AxisState_Event__request__MAX_SIZE = 1
};
// response
enum
{
  odrive_can__srv__AxisState_Event__response__MAX_SIZE = 1
};

/// Struct defined in srv/AxisState in the package odrive_can.
typedef struct odrive_can__srv__AxisState_Event
{
  service_msgs__msg__ServiceEventInfo info;
  odrive_can__srv__AxisState_Request__Sequence request;
  odrive_can__srv__AxisState_Response__Sequence response;
} odrive_can__srv__AxisState_Event;

// Struct for a sequence of odrive_can__srv__AxisState_Event.
typedef struct odrive_can__srv__AxisState_Event__Sequence
{
  odrive_can__srv__AxisState_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} odrive_can__srv__AxisState_Event__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ODRIVE_CAN__SRV__DETAIL__AXIS_STATE__STRUCT_H_
