# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/GpsFix.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GpsFix(type):
    """Metaclass of message 'GpsFix'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'GPS_NO_FIX': 0,
        'GPS_DEAD_RECKONING_ONLY': 1,
        'GPS_FIX_2D': 2,
        'GPS_FIX_3D': 3,
        'GPS_PLUS_DEAD_RECKONING': 4,
        'GPS_TIME_ONLY': 5,
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('ublox_ubx_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ublox_ubx_msgs.msg.GpsFix')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__gps_fix
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__gps_fix
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__gps_fix
            cls._TYPE_SUPPORT = module.type_support_msg__msg__gps_fix
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__gps_fix

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'GPS_NO_FIX': cls.__constants['GPS_NO_FIX'],
            'GPS_DEAD_RECKONING_ONLY': cls.__constants['GPS_DEAD_RECKONING_ONLY'],
            'GPS_FIX_2D': cls.__constants['GPS_FIX_2D'],
            'GPS_FIX_3D': cls.__constants['GPS_FIX_3D'],
            'GPS_PLUS_DEAD_RECKONING': cls.__constants['GPS_PLUS_DEAD_RECKONING'],
            'GPS_TIME_ONLY': cls.__constants['GPS_TIME_ONLY'],
        }

    @property
    def GPS_NO_FIX(self):
        """Message constant 'GPS_NO_FIX'."""
        return Metaclass_GpsFix.__constants['GPS_NO_FIX']

    @property
    def GPS_DEAD_RECKONING_ONLY(self):
        """Message constant 'GPS_DEAD_RECKONING_ONLY'."""
        return Metaclass_GpsFix.__constants['GPS_DEAD_RECKONING_ONLY']

    @property
    def GPS_FIX_2D(self):
        """Message constant 'GPS_FIX_2D'."""
        return Metaclass_GpsFix.__constants['GPS_FIX_2D']

    @property
    def GPS_FIX_3D(self):
        """Message constant 'GPS_FIX_3D'."""
        return Metaclass_GpsFix.__constants['GPS_FIX_3D']

    @property
    def GPS_PLUS_DEAD_RECKONING(self):
        """Message constant 'GPS_PLUS_DEAD_RECKONING'."""
        return Metaclass_GpsFix.__constants['GPS_PLUS_DEAD_RECKONING']

    @property
    def GPS_TIME_ONLY(self):
        """Message constant 'GPS_TIME_ONLY'."""
        return Metaclass_GpsFix.__constants['GPS_TIME_ONLY']


class GpsFix(metaclass=Metaclass_GpsFix):
    """
    Message class 'GpsFix'.

    Constants:
      GPS_NO_FIX
      GPS_DEAD_RECKONING_ONLY
      GPS_FIX_2D
      GPS_FIX_3D
      GPS_PLUS_DEAD_RECKONING
      GPS_TIME_ONLY
    """

    __slots__ = [
        '_fix_type',
    ]

    _fields_and_field_types = {
        'fix_type': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.fix_type = kwargs.get('fix_type', int())

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
        if self.fix_type != other.fix_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def fix_type(self):
        """Message field 'fix_type'."""
        return self._fix_type

    @fix_type.setter
    def fix_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fix_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'fix_type' field must be an unsigned integer in [0, 255]"
        self._fix_type = value
