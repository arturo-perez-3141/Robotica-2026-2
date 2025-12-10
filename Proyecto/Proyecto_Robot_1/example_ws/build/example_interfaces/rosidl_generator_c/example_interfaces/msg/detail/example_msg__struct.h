// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from example_interfaces:msg/ExampleMsg.idl
// generated code does not contain a copyright notice

#ifndef EXAMPLE_INTERFACES__MSG__DETAIL__EXAMPLE_MSG__STRUCT_H_
#define EXAMPLE_INTERFACES__MSG__DETAIL__EXAMPLE_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/ExampleMsg in the package example_interfaces.
/**
  * Mensaje personalizado
 */
typedef struct example_interfaces__msg__ExampleMsg
{
  rosidl_runtime_c__String message;
  int32_t id;
} example_interfaces__msg__ExampleMsg;

// Struct for a sequence of example_interfaces__msg__ExampleMsg.
typedef struct example_interfaces__msg__ExampleMsg__Sequence
{
  example_interfaces__msg__ExampleMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} example_interfaces__msg__ExampleMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // EXAMPLE_INTERFACES__MSG__DETAIL__EXAMPLE_MSG__STRUCT_H_
