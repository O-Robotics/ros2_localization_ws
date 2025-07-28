# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXEsfMeas.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXEsfMeas(type):
    """Metaclass of message 'UBXEsfMeas'."""

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
                'ublox_ubx_msgs.msg.UBXEsfMeas')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_esf_meas
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_esf_meas
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_esf_meas
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_esf_meas
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_esf_meas

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import ESFMeasDataItem
            if ESFMeasDataItem.__class__._TYPE_SUPPORT is None:
                ESFMeasDataItem.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UBXEsfMeas(metaclass=Metaclass_UBXEsfMeas):
    """Message class 'UBXEsfMeas'."""

    __slots__ = [
        '_header',
        '_time_tag',
        '_time_mark_sent',
        '_time_mark_edge',
        '_calib_ttag_valid',
        '_num_meas',
        '_id',
        '_data',
        '_calib_ttag',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'time_tag': 'uint32',
        'time_mark_sent': 'uint8',
        'time_mark_edge': 'boolean',
        'calib_ttag_valid': 'boolean',
        'num_meas': 'uint8',
        'id': 'uint16',
        'data': 'sequence<ublox_ubx_msgs/ESFMeasDataItem>',
        'calib_ttag': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'ESFMeasDataItem')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.time_tag = kwargs.get('time_tag', int())
        self.time_mark_sent = kwargs.get('time_mark_sent', int())
        self.time_mark_edge = kwargs.get('time_mark_edge', bool())
        self.calib_ttag_valid = kwargs.get('calib_ttag_valid', bool())
        self.num_meas = kwargs.get('num_meas', int())
        self.id = kwargs.get('id', int())
        self.data = kwargs.get('data', [])
        self.calib_ttag = kwargs.get('calib_ttag', int())

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
        if self.time_tag != other.time_tag:
            return False
        if self.time_mark_sent != other.time_mark_sent:
            return False
        if self.time_mark_edge != other.time_mark_edge:
            return False
        if self.calib_ttag_valid != other.calib_ttag_valid:
            return False
        if self.num_meas != other.num_meas:
            return False
        if self.id != other.id:
            return False
        if self.data != other.data:
            return False
        if self.calib_ttag != other.calib_ttag:
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
    def time_tag(self):
        """Message field 'time_tag'."""
        return self._time_tag

    @time_tag.setter
    def time_tag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_tag' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'time_tag' field must be an unsigned integer in [0, 4294967295]"
        self._time_tag = value

    @builtins.property
    def time_mark_sent(self):
        """Message field 'time_mark_sent'."""
        return self._time_mark_sent

    @time_mark_sent.setter
    def time_mark_sent(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_mark_sent' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'time_mark_sent' field must be an unsigned integer in [0, 255]"
        self._time_mark_sent = value

    @builtins.property
    def time_mark_edge(self):
        """Message field 'time_mark_edge'."""
        return self._time_mark_edge

    @time_mark_edge.setter
    def time_mark_edge(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'time_mark_edge' field must be of type 'bool'"
        self._time_mark_edge = value

    @builtins.property
    def calib_ttag_valid(self):
        """Message field 'calib_ttag_valid'."""
        return self._calib_ttag_valid

    @calib_ttag_valid.setter
    def calib_ttag_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'calib_ttag_valid' field must be of type 'bool'"
        self._calib_ttag_valid = value

    @builtins.property
    def num_meas(self):
        """Message field 'num_meas'."""
        return self._num_meas

    @num_meas.setter
    def num_meas(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_meas' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'num_meas' field must be an unsigned integer in [0, 255]"
        self._num_meas = value

    @builtins.property  # noqa: A003
    def id(self):  # noqa: A003
        """Message field 'id'."""
        return self._id

    @id.setter  # noqa: A003
    def id(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'id' field must be an unsigned integer in [0, 65535]"
        self._id = value

    @builtins.property
    def data(self):
        """Message field 'data'."""
        return self._data

    @data.setter
    def data(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import ESFMeasDataItem
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
                 all(isinstance(v, ESFMeasDataItem) for v in value) and
                 True), \
                "The 'data' field must be a set or sequence and each value of type 'ESFMeasDataItem'"
        self._data = value

    @builtins.property
    def calib_ttag(self):
        """Message field 'calib_ttag'."""
        return self._calib_ttag

    @calib_ttag.setter
    def calib_ttag(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'calib_ttag' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'calib_ttag' field must be an unsigned integer in [0, 4294967295]"
        self._calib_ttag = value
