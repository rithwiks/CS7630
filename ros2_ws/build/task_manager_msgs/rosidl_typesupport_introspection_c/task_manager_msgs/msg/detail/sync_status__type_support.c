// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from task_manager_msgs:msg/SyncStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "task_manager_msgs/msg/detail/sync_status__rosidl_typesupport_introspection_c.h"
#include "task_manager_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "task_manager_msgs/msg/detail/sync_status__functions.h"
#include "task_manager_msgs/msg/detail/sync_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  task_manager_msgs__msg__SyncStatus__init(message_memory);
}

void task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_fini_function(void * message_memory)
{
  task_manager_msgs__msg__SyncStatus__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(task_manager_msgs__msg__SyncStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(task_manager_msgs__msg__SyncStatus, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_message_members = {
  "task_manager_msgs__msg",  // message namespace
  "SyncStatus",  // message name
  2,  // number of fields
  sizeof(task_manager_msgs__msg__SyncStatus),
  task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_message_member_array,  // message members
  task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_message_type_support_handle = {
  0,
  &task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_task_manager_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, task_manager_msgs, msg, SyncStatus)() {
  task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_message_type_support_handle.typesupport_identifier) {
    task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &task_manager_msgs__msg__SyncStatus__rosidl_typesupport_introspection_c__SyncStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
