# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_interfaces:srv/ColdStart.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ColdStart_Request(type):
    """Metaclass of message 'ColdStart_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'HW_RESET_IMMEDIATELY': 0,
        'SW_RESET_CONTROLLED': 1,
        'SW_RESET_CONTROLLED_GNSS': 2,
        'HW_RESET_AFTER_SHUTDOWN': 4,
        'GNSS_STOP_CONTROLLED': 8,
        'GNSS_START_CONTROLLED': 9,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ublox_ubx_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ublox_ubx_interfaces.srv.ColdStart_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__cold_start__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__cold_start__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__cold_start__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__cold_start__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__cold_start__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'HW_RESET_IMMEDIATELY': cls.__constants['HW_RESET_IMMEDIATELY'],
            'SW_RESET_CONTROLLED': cls.__constants['SW_RESET_CONTROLLED'],
            'SW_RESET_CONTROLLED_GNSS': cls.__constants['SW_RESET_CONTROLLED_GNSS'],
            'HW_RESET_AFTER_SHUTDOWN': cls.__constants['HW_RESET_AFTER_SHUTDOWN'],
            'GNSS_STOP_CONTROLLED': cls.__constants['GNSS_STOP_CONTROLLED'],
            'GNSS_START_CONTROLLED': cls.__constants['GNSS_START_CONTROLLED'],
        }

    @property
    def HW_RESET_IMMEDIATELY(self):
        """Message constant 'HW_RESET_IMMEDIATELY'."""
        return Metaclass_ColdStart_Request.__constants['HW_RESET_IMMEDIATELY']

    @property
    def SW_RESET_CONTROLLED(self):
        """Message constant 'SW_RESET_CONTROLLED'."""
        return Metaclass_ColdStart_Request.__constants['SW_RESET_CONTROLLED']

    @property
    def SW_RESET_CONTROLLED_GNSS(self):
        """Message constant 'SW_RESET_CONTROLLED_GNSS'."""
        return Metaclass_ColdStart_Request.__constants['SW_RESET_CONTROLLED_GNSS']

    @property
    def HW_RESET_AFTER_SHUTDOWN(self):
        """Message constant 'HW_RESET_AFTER_SHUTDOWN'."""
        return Metaclass_ColdStart_Request.__constants['HW_RESET_AFTER_SHUTDOWN']

    @property
    def GNSS_STOP_CONTROLLED(self):
        """Message constant 'GNSS_STOP_CONTROLLED'."""
        return Metaclass_ColdStart_Request.__constants['GNSS_STOP_CONTROLLED']

    @property
    def GNSS_START_CONTROLLED(self):
        """Message constant 'GNSS_START_CONTROLLED'."""
        return Metaclass_ColdStart_Request.__constants['GNSS_START_CONTROLLED']


class ColdStart_Request(metaclass=Metaclass_ColdStart_Request):
    """
    Message class 'ColdStart_Request'.

    Constants:
      HW_RESET_IMMEDIATELY
      SW_RESET_CONTROLLED
      SW_RESET_CONTROLLED_GNSS
      HW_RESET_AFTER_SHUTDOWN
      GNSS_STOP_CONTROLLED
      GNSS_START_CONTROLLED
    """

    __slots__ = [
        '_reset_type',
    ]

    _fields_and_field_types = {
        'reset_type': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.reset_type = kwargs.get('reset_type', int())

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
        if self.reset_type != other.reset_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def reset_type(self):
        """Message field 'reset_type'."""
        return self._reset_type

    @reset_type.setter
    def reset_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'reset_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'reset_type' field must be an unsigned integer in [0, 255]"
        self._reset_type = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_ColdStart_Response(type):
    """Metaclass of message 'ColdStart_Response'."""

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
            module = import_type_support('ublox_ubx_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ublox_ubx_interfaces.srv.ColdStart_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__cold_start__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__cold_start__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__cold_start__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__cold_start__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__cold_start__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ColdStart_Response(metaclass=Metaclass_ColdStart_Response):
    """Message class 'ColdStart_Response'."""

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


class Metaclass_ColdStart(type):
    """Metaclass of service 'ColdStart'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ublox_ubx_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ublox_ubx_interfaces.srv.ColdStart')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__cold_start

            from ublox_ubx_interfaces.srv import _cold_start
            if _cold_start.Metaclass_ColdStart_Request._TYPE_SUPPORT is None:
                _cold_start.Metaclass_ColdStart_Request.__import_type_support__()
            if _cold_start.Metaclass_ColdStart_Response._TYPE_SUPPORT is None:
                _cold_start.Metaclass_ColdStart_Response.__import_type_support__()


class ColdStart(metaclass=Metaclass_ColdStart):
    from ublox_ubx_interfaces.srv._cold_start import ColdStart_Request as Request
    from ublox_ubx_interfaces.srv._cold_start import ColdStart_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
