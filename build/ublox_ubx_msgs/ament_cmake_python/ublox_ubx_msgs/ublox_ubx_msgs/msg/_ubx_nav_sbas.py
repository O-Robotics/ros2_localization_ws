# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXNavSBAS.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

# Member 'reserved_0'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXNavSBAS(type):
    """Metaclass of message 'UBXNavSBAS'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MODE_DISABLED': 0,
        'MODE_ENABLED_INTEGRITY': 1,
        'MODE_ENABLED_TEST': 3,
        'SYS_UNKNOWN': -1,
        'SYS_WAAS': 0,
        'SYS_EGNOS': 1,
        'SYS_MSAS': 2,
        'SYS_GAGAN': 3,
        'SYS_GPS': 16,
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
                'ublox_ubx_msgs.msg.UBXNavSBAS')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_nav_sbas
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_nav_sbas
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_nav_sbas
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_nav_sbas
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_nav_sbas

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import SBASService
            if SBASService.__class__._TYPE_SUPPORT is None:
                SBASService.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import SBASStatusFlags
            if SBASStatusFlags.__class__._TYPE_SUPPORT is None:
                SBASStatusFlags.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import SBASSvData
            if SBASSvData.__class__._TYPE_SUPPORT is None:
                SBASSvData.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MODE_DISABLED': cls.__constants['MODE_DISABLED'],
            'MODE_ENABLED_INTEGRITY': cls.__constants['MODE_ENABLED_INTEGRITY'],
            'MODE_ENABLED_TEST': cls.__constants['MODE_ENABLED_TEST'],
            'SYS_UNKNOWN': cls.__constants['SYS_UNKNOWN'],
            'SYS_WAAS': cls.__constants['SYS_WAAS'],
            'SYS_EGNOS': cls.__constants['SYS_EGNOS'],
            'SYS_MSAS': cls.__constants['SYS_MSAS'],
            'SYS_GAGAN': cls.__constants['SYS_GAGAN'],
            'SYS_GPS': cls.__constants['SYS_GPS'],
        }

    @property
    def MODE_DISABLED(self):
        """Message constant 'MODE_DISABLED'."""
        return Metaclass_UBXNavSBAS.__constants['MODE_DISABLED']

    @property
    def MODE_ENABLED_INTEGRITY(self):
        """Message constant 'MODE_ENABLED_INTEGRITY'."""
        return Metaclass_UBXNavSBAS.__constants['MODE_ENABLED_INTEGRITY']

    @property
    def MODE_ENABLED_TEST(self):
        """Message constant 'MODE_ENABLED_TEST'."""
        return Metaclass_UBXNavSBAS.__constants['MODE_ENABLED_TEST']

    @property
    def SYS_UNKNOWN(self):
        """Message constant 'SYS_UNKNOWN'."""
        return Metaclass_UBXNavSBAS.__constants['SYS_UNKNOWN']

    @property
    def SYS_WAAS(self):
        """Message constant 'SYS_WAAS'."""
        return Metaclass_UBXNavSBAS.__constants['SYS_WAAS']

    @property
    def SYS_EGNOS(self):
        """Message constant 'SYS_EGNOS'."""
        return Metaclass_UBXNavSBAS.__constants['SYS_EGNOS']

    @property
    def SYS_MSAS(self):
        """Message constant 'SYS_MSAS'."""
        return Metaclass_UBXNavSBAS.__constants['SYS_MSAS']

    @property
    def SYS_GAGAN(self):
        """Message constant 'SYS_GAGAN'."""
        return Metaclass_UBXNavSBAS.__constants['SYS_GAGAN']

    @property
    def SYS_GPS(self):
        """Message constant 'SYS_GPS'."""
        return Metaclass_UBXNavSBAS.__constants['SYS_GPS']


class UBXNavSBAS(metaclass=Metaclass_UBXNavSBAS):
    """
    Message class 'UBXNavSBAS'.

    Constants:
      MODE_DISABLED
      MODE_ENABLED_INTEGRITY
      MODE_ENABLED_TEST
      SYS_UNKNOWN
      SYS_WAAS
      SYS_EGNOS
      SYS_MSAS
      SYS_GAGAN
      SYS_GPS
    """

    __slots__ = [
        '_header',
        '_itow',
        '_geo',
        '_mode',
        '_sys',
        '_service',
        '_cnt',
        '_status_flags',
        '_reserved_0',
        '_sv_data',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'itow': 'uint32',
        'geo': 'uint8',
        'mode': 'uint8',
        'sys': 'int8',
        'service': 'ublox_ubx_msgs/SBASService',
        'cnt': 'uint8',
        'status_flags': 'ublox_ubx_msgs/SBASStatusFlags',
        'reserved_0': 'uint8[2]',
        'sv_data': 'sequence<ublox_ubx_msgs/SBASSvData>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'SBASService'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'SBASStatusFlags'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 2),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'SBASSvData')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.itow = kwargs.get('itow', int())
        self.geo = kwargs.get('geo', int())
        self.mode = kwargs.get('mode', int())
        self.sys = kwargs.get('sys', int())
        from ublox_ubx_msgs.msg import SBASService
        self.service = kwargs.get('service', SBASService())
        self.cnt = kwargs.get('cnt', int())
        from ublox_ubx_msgs.msg import SBASStatusFlags
        self.status_flags = kwargs.get('status_flags', SBASStatusFlags())
        if 'reserved_0' not in kwargs:
            self.reserved_0 = numpy.zeros(2, dtype=numpy.uint8)
        else:
            self.reserved_0 = numpy.array(kwargs.get('reserved_0'), dtype=numpy.uint8)
            assert self.reserved_0.shape == (2, )
        self.sv_data = kwargs.get('sv_data', [])

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
        if self.geo != other.geo:
            return False
        if self.mode != other.mode:
            return False
        if self.sys != other.sys:
            return False
        if self.service != other.service:
            return False
        if self.cnt != other.cnt:
            return False
        if self.status_flags != other.status_flags:
            return False
        if all(self.reserved_0 != other.reserved_0):
            return False
        if self.sv_data != other.sv_data:
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
    def geo(self):
        """Message field 'geo'."""
        return self._geo

    @geo.setter
    def geo(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'geo' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'geo' field must be an unsigned integer in [0, 255]"
        self._geo = value

    @builtins.property
    def mode(self):
        """Message field 'mode'."""
        return self._mode

    @mode.setter
    def mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mode' field must be an unsigned integer in [0, 255]"
        self._mode = value

    @builtins.property
    def sys(self):
        """Message field 'sys'."""
        return self._sys

    @sys.setter
    def sys(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sys' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'sys' field must be an integer in [-128, 127]"
        self._sys = value

    @builtins.property
    def service(self):
        """Message field 'service'."""
        return self._service

    @service.setter
    def service(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import SBASService
            assert \
                isinstance(value, SBASService), \
                "The 'service' field must be a sub message of type 'SBASService'"
        self._service = value

    @builtins.property
    def cnt(self):
        """Message field 'cnt'."""
        return self._cnt

    @cnt.setter
    def cnt(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cnt' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'cnt' field must be an unsigned integer in [0, 255]"
        self._cnt = value

    @builtins.property
    def status_flags(self):
        """Message field 'status_flags'."""
        return self._status_flags

    @status_flags.setter
    def status_flags(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import SBASStatusFlags
            assert \
                isinstance(value, SBASStatusFlags), \
                "The 'status_flags' field must be a sub message of type 'SBASStatusFlags'"
        self._status_flags = value

    @builtins.property
    def reserved_0(self):
        """Message field 'reserved_0'."""
        return self._reserved_0

    @reserved_0.setter
    def reserved_0(self, value):
        if isinstance(value, numpy.ndarray):
            assert value.dtype == numpy.uint8, \
                "The 'reserved_0' numpy.ndarray() must have the dtype of 'numpy.uint8'"
            assert value.size == 2, \
                "The 'reserved_0' numpy.ndarray() must have a size of 2"
            self._reserved_0 = value
            return
        if __debug__:
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
                 len(value) == 2 and
                 all(isinstance(v, int) for v in value) and
                 all(val >= 0 and val < 256 for val in value)), \
                "The 'reserved_0' field must be a set or sequence with length 2 and each value of type 'int' and each unsigned integer in [0, 255]"
        self._reserved_0 = numpy.array(value, dtype=numpy.uint8)

    @builtins.property
    def sv_data(self):
        """Message field 'sv_data'."""
        return self._sv_data

    @sv_data.setter
    def sv_data(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import SBASSvData
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
                 all(isinstance(v, SBASSvData) for v in value) and
                 True), \
                "The 'sv_data' field must be a set or sequence and each value of type 'SBASSvData'"
        self._sv_data = value
