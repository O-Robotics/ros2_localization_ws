# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXNavVelNED.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXNavVelNED(type):
    """Metaclass of message 'UBXNavVelNED'."""

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
            module = import_type_support('ublox_ubx_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'ublox_ubx_msgs.msg.UBXNavVelNED')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_nav_vel_ned
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_nav_vel_ned
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_nav_vel_ned
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_nav_vel_ned
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_nav_vel_ned

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UBXNavVelNED(metaclass=Metaclass_UBXNavVelNED):
    """Message class 'UBXNavVelNED'."""

    __slots__ = [
        '_header',
        '_itow',
        '_vel_n',
        '_vel_e',
        '_vel_d',
        '_speed',
        '_g_speed',
        '_heading',
        '_s_acc',
        '_c_acc',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'itow': 'uint32',
        'vel_n': 'int32',
        'vel_e': 'int32',
        'vel_d': 'int32',
        'speed': 'uint32',
        'g_speed': 'uint32',
        'heading': 'int32',
        's_acc': 'uint32',
        'c_acc': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.itow = kwargs.get('itow', int())
        self.vel_n = kwargs.get('vel_n', int())
        self.vel_e = kwargs.get('vel_e', int())
        self.vel_d = kwargs.get('vel_d', int())
        self.speed = kwargs.get('speed', int())
        self.g_speed = kwargs.get('g_speed', int())
        self.heading = kwargs.get('heading', int())
        self.s_acc = kwargs.get('s_acc', int())
        self.c_acc = kwargs.get('c_acc', int())

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
        if self.header != other.header:
            return False
        if self.itow != other.itow:
            return False
        if self.vel_n != other.vel_n:
            return False
        if self.vel_e != other.vel_e:
            return False
        if self.vel_d != other.vel_d:
            return False
        if self.speed != other.speed:
            return False
        if self.g_speed != other.g_speed:
            return False
        if self.heading != other.heading:
            return False
        if self.s_acc != other.s_acc:
            return False
        if self.c_acc != other.c_acc:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def itow(self):
        """Message field 'itow'."""
        return self._itow

    @itow.setter
    def itow(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'itow' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'itow' field must be an unsigned integer in [0, 4294967295]"
        self._itow = value

    @builtins.property
    def vel_n(self):
        """Message field 'vel_n'."""
        return self._vel_n

    @vel_n.setter
    def vel_n(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vel_n' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'vel_n' field must be an integer in [-2147483648, 2147483647]"
        self._vel_n = value

    @builtins.property
    def vel_e(self):
        """Message field 'vel_e'."""
        return self._vel_e

    @vel_e.setter
    def vel_e(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vel_e' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'vel_e' field must be an integer in [-2147483648, 2147483647]"
        self._vel_e = value

    @builtins.property
    def vel_d(self):
        """Message field 'vel_d'."""
        return self._vel_d

    @vel_d.setter
    def vel_d(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'vel_d' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'vel_d' field must be an integer in [-2147483648, 2147483647]"
        self._vel_d = value

    @builtins.property
    def speed(self):
        """Message field 'speed'."""
        return self._speed

    @speed.setter
    def speed(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'speed' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'speed' field must be an unsigned integer in [0, 4294967295]"
        self._speed = value

    @builtins.property
    def g_speed(self):
        """Message field 'g_speed'."""
        return self._g_speed

    @g_speed.setter
    def g_speed(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'g_speed' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'g_speed' field must be an unsigned integer in [0, 4294967295]"
        self._g_speed = value

    @builtins.property
    def heading(self):
        """Message field 'heading'."""
        return self._heading

    @heading.setter
    def heading(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'heading' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'heading' field must be an integer in [-2147483648, 2147483647]"
        self._heading = value

    @builtins.property
    def s_acc(self):
        """Message field 's_acc'."""
        return self._s_acc

    @s_acc.setter
    def s_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 's_acc' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 's_acc' field must be an unsigned integer in [0, 4294967295]"
        self._s_acc = value

    @builtins.property
    def c_acc(self):
        """Message field 'c_acc'."""
        return self._c_acc

    @c_acc.setter
    def c_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'c_acc' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'c_acc' field must be an unsigned integer in [0, 4294967295]"
        self._c_acc = value
