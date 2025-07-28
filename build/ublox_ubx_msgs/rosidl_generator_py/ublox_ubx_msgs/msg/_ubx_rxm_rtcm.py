# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXRxmRTCM.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXRxmRTCM(type):
    """Metaclass of message 'UBXRxmRTCM'."""

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
                'ublox_ubx_msgs.msg.UBXRxmRTCM')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_rxm_rtcm
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_rxm_rtcm
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_rxm_rtcm
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_rxm_rtcm
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_rxm_rtcm

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


class UBXRxmRTCM(metaclass=Metaclass_UBXRxmRTCM):
    """Message class 'UBXRxmRTCM'."""

    __slots__ = [
        '_header',
        '_version',
        '_crc_failed',
        '_msg_used',
        '_sub_type',
        '_ref_station',
        '_msg_type',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'version': 'uint8',
        'crc_failed': 'boolean',
        'msg_used': 'uint8',
        'sub_type': 'uint16',
        'ref_station': 'uint16',
        'msg_type': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.version = kwargs.get('version', int())
        self.crc_failed = kwargs.get('crc_failed', bool())
        self.msg_used = kwargs.get('msg_used', int())
        self.sub_type = kwargs.get('sub_type', int())
        self.ref_station = kwargs.get('ref_station', int())
        self.msg_type = kwargs.get('msg_type', int())

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
        if self.version != other.version:
            return False
        if self.crc_failed != other.crc_failed:
            return False
        if self.msg_used != other.msg_used:
            return False
        if self.sub_type != other.sub_type:
            return False
        if self.ref_station != other.ref_station:
            return False
        if self.msg_type != other.msg_type:
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
    def version(self):
        """Message field 'version'."""
        return self._version

    @version.setter
    def version(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'version' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'version' field must be an unsigned integer in [0, 255]"
        self._version = value

    @builtins.property
    def crc_failed(self):
        """Message field 'crc_failed'."""
        return self._crc_failed

    @crc_failed.setter
    def crc_failed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'crc_failed' field must be of type 'bool'"
        self._crc_failed = value

    @builtins.property
    def msg_used(self):
        """Message field 'msg_used'."""
        return self._msg_used

    @msg_used.setter
    def msg_used(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'msg_used' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'msg_used' field must be an unsigned integer in [0, 255]"
        self._msg_used = value

    @builtins.property
    def sub_type(self):
        """Message field 'sub_type'."""
        return self._sub_type

    @sub_type.setter
    def sub_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sub_type' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'sub_type' field must be an unsigned integer in [0, 65535]"
        self._sub_type = value

    @builtins.property
    def ref_station(self):
        """Message field 'ref_station'."""
        return self._ref_station

    @ref_station.setter
    def ref_station(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ref_station' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'ref_station' field must be an unsigned integer in [0, 65535]"
        self._ref_station = value

    @builtins.property
    def msg_type(self):
        """Message field 'msg_type'."""
        return self._msg_type

    @msg_type.setter
    def msg_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'msg_type' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'msg_type' field must be an unsigned integer in [0, 65535]"
        self._msg_type = value
