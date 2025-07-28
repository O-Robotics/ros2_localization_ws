# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXNavTimeUTC.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXNavTimeUTC(type):
    """Metaclass of message 'UBXNavTimeUTC'."""

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
                'ublox_ubx_msgs.msg.UBXNavTimeUTC')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_nav_time_utc
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_nav_time_utc
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_nav_time_utc
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_nav_time_utc
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_nav_time_utc

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import UtcStd
            if UtcStd.__class__._TYPE_SUPPORT is None:
                UtcStd.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UBXNavTimeUTC(metaclass=Metaclass_UBXNavTimeUTC):
    """Message class 'UBXNavTimeUTC'."""

    __slots__ = [
        '_header',
        '_itow',
        '_t_acc',
        '_nano',
        '_year',
        '_month',
        '_day',
        '_hour',
        '_min',
        '_sec',
        '_valid_tow',
        '_valid_wkn',
        '_valid_utc',
        '_utc_std',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'itow': 'uint32',
        't_acc': 'uint32',
        'nano': 'int32',
        'year': 'int16',
        'month': 'int8',
        'day': 'int8',
        'hour': 'int8',
        'min': 'int8',
        'sec': 'int8',
        'valid_tow': 'boolean',
        'valid_wkn': 'boolean',
        'valid_utc': 'boolean',
        'utc_std': 'ublox_ubx_msgs/UtcStd',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'UtcStd'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.itow = kwargs.get('itow', int())
        self.t_acc = kwargs.get('t_acc', int())
        self.nano = kwargs.get('nano', int())
        self.year = kwargs.get('year', int())
        self.month = kwargs.get('month', int())
        self.day = kwargs.get('day', int())
        self.hour = kwargs.get('hour', int())
        self.min = kwargs.get('min', int())
        self.sec = kwargs.get('sec', int())
        self.valid_tow = kwargs.get('valid_tow', bool())
        self.valid_wkn = kwargs.get('valid_wkn', bool())
        self.valid_utc = kwargs.get('valid_utc', bool())
        from ublox_ubx_msgs.msg import UtcStd
        self.utc_std = kwargs.get('utc_std', UtcStd())

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
        if self.t_acc != other.t_acc:
            return False
        if self.nano != other.nano:
            return False
        if self.year != other.year:
            return False
        if self.month != other.month:
            return False
        if self.day != other.day:
            return False
        if self.hour != other.hour:
            return False
        if self.min != other.min:
            return False
        if self.sec != other.sec:
            return False
        if self.valid_tow != other.valid_tow:
            return False
        if self.valid_wkn != other.valid_wkn:
            return False
        if self.valid_utc != other.valid_utc:
            return False
        if self.utc_std != other.utc_std:
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
    def t_acc(self):
        """Message field 't_acc'."""
        return self._t_acc

    @t_acc.setter
    def t_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 't_acc' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 't_acc' field must be an unsigned integer in [0, 4294967295]"
        self._t_acc = value

    @builtins.property
    def nano(self):
        """Message field 'nano'."""
        return self._nano

    @nano.setter
    def nano(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'nano' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'nano' field must be an integer in [-2147483648, 2147483647]"
        self._nano = value

    @builtins.property
    def year(self):
        """Message field 'year'."""
        return self._year

    @year.setter
    def year(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'year' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'year' field must be an integer in [-32768, 32767]"
        self._year = value

    @builtins.property
    def month(self):
        """Message field 'month'."""
        return self._month

    @month.setter
    def month(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'month' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'month' field must be an integer in [-128, 127]"
        self._month = value

    @builtins.property
    def day(self):
        """Message field 'day'."""
        return self._day

    @day.setter
    def day(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'day' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'day' field must be an integer in [-128, 127]"
        self._day = value

    @builtins.property
    def hour(self):
        """Message field 'hour'."""
        return self._hour

    @hour.setter
    def hour(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hour' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'hour' field must be an integer in [-128, 127]"
        self._hour = value

    @builtins.property  # noqa: A003
    def min(self):  # noqa: A003
        """Message field 'min'."""
        return self._min

    @min.setter  # noqa: A003
    def min(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'min' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'min' field must be an integer in [-128, 127]"
        self._min = value

    @builtins.property
    def sec(self):
        """Message field 'sec'."""
        return self._sec

    @sec.setter
    def sec(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sec' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'sec' field must be an integer in [-128, 127]"
        self._sec = value

    @builtins.property
    def valid_tow(self):
        """Message field 'valid_tow'."""
        return self._valid_tow

    @valid_tow.setter
    def valid_tow(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'valid_tow' field must be of type 'bool'"
        self._valid_tow = value

    @builtins.property
    def valid_wkn(self):
        """Message field 'valid_wkn'."""
        return self._valid_wkn

    @valid_wkn.setter
    def valid_wkn(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'valid_wkn' field must be of type 'bool'"
        self._valid_wkn = value

    @builtins.property
    def valid_utc(self):
        """Message field 'valid_utc'."""
        return self._valid_utc

    @valid_utc.setter
    def valid_utc(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'valid_utc' field must be of type 'bool'"
        self._valid_utc = value

    @builtins.property
    def utc_std(self):
        """Message field 'utc_std'."""
        return self._utc_std

    @utc_std.setter
    def utc_std(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import UtcStd
            assert \
                isinstance(value, UtcStd), \
                "The 'utc_std' field must be a sub message of type 'UtcStd'"
        self._utc_std = value
