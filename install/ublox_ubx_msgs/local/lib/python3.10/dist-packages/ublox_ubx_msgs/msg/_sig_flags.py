# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/SigFlags.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SigFlags(type):
    """Metaclass of message 'SigFlags'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'HEALTH_UNKNOWN': 0,
        'HEALTH_HEALTHY': 1,
        'HEALTH_UNHEALTHY': 2,
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
                'ublox_ubx_msgs.msg.SigFlags')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sig_flags
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sig_flags
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sig_flags
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sig_flags
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sig_flags

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'HEALTH_UNKNOWN': cls.__constants['HEALTH_UNKNOWN'],
            'HEALTH_HEALTHY': cls.__constants['HEALTH_HEALTHY'],
            'HEALTH_UNHEALTHY': cls.__constants['HEALTH_UNHEALTHY'],
        }

    @property
    def HEALTH_UNKNOWN(self):
        """Message constant 'HEALTH_UNKNOWN'."""
        return Metaclass_SigFlags.__constants['HEALTH_UNKNOWN']

    @property
    def HEALTH_HEALTHY(self):
        """Message constant 'HEALTH_HEALTHY'."""
        return Metaclass_SigFlags.__constants['HEALTH_HEALTHY']

    @property
    def HEALTH_UNHEALTHY(self):
        """Message constant 'HEALTH_UNHEALTHY'."""
        return Metaclass_SigFlags.__constants['HEALTH_UNHEALTHY']


class SigFlags(metaclass=Metaclass_SigFlags):
    """
    Message class 'SigFlags'.

    Constants:
      HEALTH_UNKNOWN
      HEALTH_HEALTHY
      HEALTH_UNHEALTHY
    """

    __slots__ = [
        '_health',
        '_pr_smoothed',
        '_pr_used',
        '_cr_used',
        '_do_used',
        '_pr_corr_used',
        '_cr_corr_used',
        '_do_corr_used',
    ]

    _fields_and_field_types = {
        'health': 'uint8',
        'pr_smoothed': 'boolean',
        'pr_used': 'boolean',
        'cr_used': 'boolean',
        'do_used': 'boolean',
        'pr_corr_used': 'boolean',
        'cr_corr_used': 'boolean',
        'do_corr_used': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.health = kwargs.get('health', int())
        self.pr_smoothed = kwargs.get('pr_smoothed', bool())
        self.pr_used = kwargs.get('pr_used', bool())
        self.cr_used = kwargs.get('cr_used', bool())
        self.do_used = kwargs.get('do_used', bool())
        self.pr_corr_used = kwargs.get('pr_corr_used', bool())
        self.cr_corr_used = kwargs.get('cr_corr_used', bool())
        self.do_corr_used = kwargs.get('do_corr_used', bool())

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
        if self.health != other.health:
            return False
        if self.pr_smoothed != other.pr_smoothed:
            return False
        if self.pr_used != other.pr_used:
            return False
        if self.cr_used != other.cr_used:
            return False
        if self.do_used != other.do_used:
            return False
        if self.pr_corr_used != other.pr_corr_used:
            return False
        if self.cr_corr_used != other.cr_corr_used:
            return False
        if self.do_corr_used != other.do_corr_used:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def health(self):
        """Message field 'health'."""
        return self._health

    @health.setter
    def health(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'health' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'health' field must be an unsigned integer in [0, 255]"
        self._health = value

    @builtins.property
    def pr_smoothed(self):
        """Message field 'pr_smoothed'."""
        return self._pr_smoothed

    @pr_smoothed.setter
    def pr_smoothed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pr_smoothed' field must be of type 'bool'"
        self._pr_smoothed = value

    @builtins.property
    def pr_used(self):
        """Message field 'pr_used'."""
        return self._pr_used

    @pr_used.setter
    def pr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pr_used' field must be of type 'bool'"
        self._pr_used = value

    @builtins.property
    def cr_used(self):
        """Message field 'cr_used'."""
        return self._cr_used

    @cr_used.setter
    def cr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cr_used' field must be of type 'bool'"
        self._cr_used = value

    @builtins.property
    def do_used(self):
        """Message field 'do_used'."""
        return self._do_used

    @do_used.setter
    def do_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'do_used' field must be of type 'bool'"
        self._do_used = value

    @builtins.property
    def pr_corr_used(self):
        """Message field 'pr_corr_used'."""
        return self._pr_corr_used

    @pr_corr_used.setter
    def pr_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pr_corr_used' field must be of type 'bool'"
        self._pr_corr_used = value

    @builtins.property
    def cr_corr_used(self):
        """Message field 'cr_corr_used'."""
        return self._cr_corr_used

    @cr_corr_used.setter
    def cr_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cr_corr_used' field must be of type 'bool'"
        self._cr_corr_used = value

    @builtins.property
    def do_corr_used(self):
        """Message field 'do_corr_used'."""
        return self._do_corr_used

    @do_corr_used.setter
    def do_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'do_corr_used' field must be of type 'bool'"
        self._do_corr_used = value
