# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/RecStat.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_RecStat(type):
    """Metaclass of message 'RecStat'."""

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
                'ublox_ubx_msgs.msg.RecStat')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__rec_stat
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__rec_stat
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__rec_stat
            cls._TYPE_SUPPORT = module.type_support_msg__msg__rec_stat
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__rec_stat

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class RecStat(metaclass=Metaclass_RecStat):
    """Message class 'RecStat'."""

    __slots__ = [
        '_leap_sec',
        '_clk_reset',
    ]

    _fields_and_field_types = {
        'leap_sec': 'boolean',
        'clk_reset': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.leap_sec = kwargs.get('leap_sec', bool())
        self.clk_reset = kwargs.get('clk_reset', bool())

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
        if self.leap_sec != other.leap_sec:
            return False
        if self.clk_reset != other.clk_reset:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def leap_sec(self):
        """Message field 'leap_sec'."""
        return self._leap_sec

    @leap_sec.setter
    def leap_sec(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'leap_sec' field must be of type 'bool'"
        self._leap_sec = value

    @builtins.property
    def clk_reset(self):
        """Message field 'clk_reset'."""
        return self._clk_reset

    @clk_reset.setter
    def clk_reset(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'clk_reset' field must be of type 'bool'"
        self._clk_reset = value
