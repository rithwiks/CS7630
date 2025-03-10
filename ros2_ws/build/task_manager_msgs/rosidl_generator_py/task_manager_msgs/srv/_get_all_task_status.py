# generated from rosidl_generator_py/resource/_idl.py.em
# with input from task_manager_msgs:srv/GetAllTaskStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetAllTaskStatus_Request(type):
    """Metaclass of message 'GetAllTaskStatus_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('task_manager_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'task_manager_msgs.srv.GetAllTaskStatus_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_all_task_status__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_all_task_status__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_all_task_status__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_all_task_status__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_all_task_status__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetAllTaskStatus_Request(metaclass=Metaclass_GetAllTaskStatus_Request):
    """Message class 'GetAllTaskStatus_Request'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


# Import statements for member types

import builtins  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_GetAllTaskStatus_Response(type):
    """Metaclass of message 'GetAllTaskStatus_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('task_manager_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'task_manager_msgs.srv.GetAllTaskStatus_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_all_task_status__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_all_task_status__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_all_task_status__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_all_task_status__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_all_task_status__response

            from task_manager_msgs.msg import TaskStatus
            if TaskStatus.__class__._TYPE_SUPPORT is None:
                TaskStatus.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetAllTaskStatus_Response(metaclass=Metaclass_GetAllTaskStatus_Response):
    """Message class 'GetAllTaskStatus_Response'."""

    __slots__ = [
        '_running_tasks',
        '_zombie_tasks',
    ]

    _fields_and_field_types = {
        'running_tasks': 'sequence<task_manager_msgs/TaskStatus>',
        'zombie_tasks': 'sequence<task_manager_msgs/TaskStatus>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['task_manager_msgs', 'msg'], 'TaskStatus')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['task_manager_msgs', 'msg'], 'TaskStatus')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.running_tasks = kwargs.get('running_tasks', [])
        self.zombie_tasks = kwargs.get('zombie_tasks', [])

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.running_tasks != other.running_tasks:
            return False
        if self.zombie_tasks != other.zombie_tasks:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def running_tasks(self):
        """Message field 'running_tasks'."""
        return self._running_tasks

    @running_tasks.setter
    def running_tasks(self, value):
        if __debug__:
            from task_manager_msgs.msg import TaskStatus
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, TaskStatus) for v in value) and
                 True), \
                "The 'running_tasks' field must be a set or sequence and each value of type 'TaskStatus'"
        self._running_tasks = value

    @builtins.property
    def zombie_tasks(self):
        """Message field 'zombie_tasks'."""
        return self._zombie_tasks

    @zombie_tasks.setter
    def zombie_tasks(self, value):
        if __debug__:
            from task_manager_msgs.msg import TaskStatus
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, TaskStatus) for v in value) and
                 True), \
                "The 'zombie_tasks' field must be a set or sequence and each value of type 'TaskStatus'"
        self._zombie_tasks = value


class Metaclass_GetAllTaskStatus(type):
    """Metaclass of service 'GetAllTaskStatus'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('task_manager_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'task_manager_msgs.srv.GetAllTaskStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_all_task_status

            from task_manager_msgs.srv import _get_all_task_status
            if _get_all_task_status.Metaclass_GetAllTaskStatus_Request._TYPE_SUPPORT is None:
                _get_all_task_status.Metaclass_GetAllTaskStatus_Request.__import_type_support__()
            if _get_all_task_status.Metaclass_GetAllTaskStatus_Response._TYPE_SUPPORT is None:
                _get_all_task_status.Metaclass_GetAllTaskStatus_Response.__import_type_support__()


class GetAllTaskStatus(metaclass=Metaclass_GetAllTaskStatus):
    from task_manager_msgs.srv._get_all_task_status import GetAllTaskStatus_Request as Request
    from task_manager_msgs.srv._get_all_task_status import GetAllTaskStatus_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
