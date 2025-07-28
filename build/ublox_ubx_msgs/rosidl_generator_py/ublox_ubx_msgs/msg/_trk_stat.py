# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/TrkStat.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrkStat(type):
    """Metaclass of message 'TrkStat'."""

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
                'ublox_ubx_msgs.msg.TrkStat')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__trk_stat
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__trk_stat
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__trk_stat
            cls._TYPE_SUPPORT = module.type_support_msg__msg__trk_stat
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__trk_stat

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TrkStat(metaclass=Metaclass_TrkStat):
    """Message class 'TrkStat'."""

    __slots__ = [
        '_pr_valid',
        '_cp_valid',
        '_half_cyc',
        '_sub_half_cyc',
    ]

    _fields_and_field_types = {
        'pr_valid': 'boolean',
        'cp_valid': 'boolean',
        'half_cyc': 'boolean',
        'sub_half_cyc': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.pr_valid = kwargs.get('pr_valid', bool())
        self.cp_valid = kwargs.get('cp_valid', bool())
        self.half_cyc = kwargs.get('half_cyc', bool())
        self.sub_half_cyc = kwargs.get('sub_half_cyc', bool())

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
        if self.pr_valid != other.pr_valid:
            return False
        if self.cp_valid != other.cp_valid:
            return False
        if self.half_cyc != other.half_cyc:
            return False
        if self.sub_half_cyc != other.sub_half_cyc:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def pr_valid(self):
        """Message field 'pr_valid'."""
        return self._pr_valid

    @pr_valid.setter
    def pr_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pr_valid' field must be of type 'bool'"
        self._pr_valid = value

    @builtins.property
    def cp_valid(self):
        """Message field 'cp_valid'."""
        return self._cp_valid

    @cp_valid.setter
    def cp_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cp_valid' field must be of type 'bool'"
        self._cp_valid = value

    @builtins.property
    def half_cyc(self):
        """Message field 'half_cyc'."""
        return self._half_cyc

    @half_cyc.setter
    def half_cyc(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'half_cyc' field must be of type 'bool'"
        self._half_cyc = value

    @builtins.property
    def sub_half_cyc(self):
        """Message field 'sub_half_cyc'."""
        return self._sub_half_cyc

    @sub_half_cyc.setter
    def sub_half_cyc(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sub_half_cyc' field must be of type 'bool'"
        self._sub_half_cyc = value
