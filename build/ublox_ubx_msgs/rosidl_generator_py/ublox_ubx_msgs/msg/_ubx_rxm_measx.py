# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXRxmMeasx.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXRxmMeasx(type):
    """Metaclass of message 'UBXRxmMeasx'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'TOW_NOT_SET': 0,
        'TOW_SET': 1,
        'TOW_SET2': 2,
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
                'ublox_ubx_msgs.msg.UBXRxmMeasx')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_rxm_measx
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_rxm_measx
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_rxm_measx
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_rxm_measx
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_rxm_measx

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import MeasxData
            if MeasxData.__class__._TYPE_SUPPORT is None:
                MeasxData.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'TOW_NOT_SET': cls.__constants['TOW_NOT_SET'],
            'TOW_SET': cls.__constants['TOW_SET'],
            'TOW_SET2': cls.__constants['TOW_SET2'],
        }

    @property
    def TOW_NOT_SET(self):
        """Message constant 'TOW_NOT_SET'."""
        return Metaclass_UBXRxmMeasx.__constants['TOW_NOT_SET']

    @property
    def TOW_SET(self):
        """Message constant 'TOW_SET'."""
        return Metaclass_UBXRxmMeasx.__constants['TOW_SET']

    @property
    def TOW_SET2(self):
        """Message constant 'TOW_SET2'."""
        return Metaclass_UBXRxmMeasx.__constants['TOW_SET2']


class UBXRxmMeasx(metaclass=Metaclass_UBXRxmMeasx):
    """
    Message class 'UBXRxmMeasx'.

    Constants:
      TOW_NOT_SET
      TOW_SET
      TOW_SET2
    """

    __slots__ = [
        '_header',
        '_version',
        '_gps_tow',
        '_glo_tow',
        '_bds_tow',
        '_qzss_tow',
        '_gps_tow_acc',
        '_glo_tow_acc',
        '_bds_tow_acc',
        '_qzss_tow_acc',
        '_num_sv',
        '_flags',
        '_sv_data',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'version': 'uint8',
        'gps_tow': 'uint32',
        'glo_tow': 'uint32',
        'bds_tow': 'uint32',
        'qzss_tow': 'uint32',
        'gps_tow_acc': 'uint16',
        'glo_tow_acc': 'uint16',
        'bds_tow_acc': 'uint16',
        'qzss_tow_acc': 'uint16',
        'num_sv': 'uint8',
        'flags': 'uint8',
        'sv_data': 'sequence<ublox_ubx_msgs/MeasxData>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'MeasxData')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.version = kwargs.get('version', int())
        self.gps_tow = kwargs.get('gps_tow', int())
        self.glo_tow = kwargs.get('glo_tow', int())
        self.bds_tow = kwargs.get('bds_tow', int())
        self.qzss_tow = kwargs.get('qzss_tow', int())
        self.gps_tow_acc = kwargs.get('gps_tow_acc', int())
        self.glo_tow_acc = kwargs.get('glo_tow_acc', int())
        self.bds_tow_acc = kwargs.get('bds_tow_acc', int())
        self.qzss_tow_acc = kwargs.get('qzss_tow_acc', int())
        self.num_sv = kwargs.get('num_sv', int())
        self.flags = kwargs.get('flags', int())
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
        if self.version != other.version:
            return False
        if self.gps_tow != other.gps_tow:
            return False
        if self.glo_tow != other.glo_tow:
            return False
        if self.bds_tow != other.bds_tow:
            return False
        if self.qzss_tow != other.qzss_tow:
            return False
        if self.gps_tow_acc != other.gps_tow_acc:
            return False
        if self.glo_tow_acc != other.glo_tow_acc:
            return False
        if self.bds_tow_acc != other.bds_tow_acc:
            return False
        if self.qzss_tow_acc != other.qzss_tow_acc:
            return False
        if self.num_sv != other.num_sv:
            return False
        if self.flags != other.flags:
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
    def gps_tow(self):
        """Message field 'gps_tow'."""
        return self._gps_tow

    @gps_tow.setter
    def gps_tow(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gps_tow' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'gps_tow' field must be an unsigned integer in [0, 4294967295]"
        self._gps_tow = value

    @builtins.property
    def glo_tow(self):
        """Message field 'glo_tow'."""
        return self._glo_tow

    @glo_tow.setter
    def glo_tow(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'glo_tow' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'glo_tow' field must be an unsigned integer in [0, 4294967295]"
        self._glo_tow = value

    @builtins.property
    def bds_tow(self):
        """Message field 'bds_tow'."""
        return self._bds_tow

    @bds_tow.setter
    def bds_tow(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bds_tow' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'bds_tow' field must be an unsigned integer in [0, 4294967295]"
        self._bds_tow = value

    @builtins.property
    def qzss_tow(self):
        """Message field 'qzss_tow'."""
        return self._qzss_tow

    @qzss_tow.setter
    def qzss_tow(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'qzss_tow' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'qzss_tow' field must be an unsigned integer in [0, 4294967295]"
        self._qzss_tow = value

    @builtins.property
    def gps_tow_acc(self):
        """Message field 'gps_tow_acc'."""
        return self._gps_tow_acc

    @gps_tow_acc.setter
    def gps_tow_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gps_tow_acc' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'gps_tow_acc' field must be an unsigned integer in [0, 65535]"
        self._gps_tow_acc = value

    @builtins.property
    def glo_tow_acc(self):
        """Message field 'glo_tow_acc'."""
        return self._glo_tow_acc

    @glo_tow_acc.setter
    def glo_tow_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'glo_tow_acc' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'glo_tow_acc' field must be an unsigned integer in [0, 65535]"
        self._glo_tow_acc = value

    @builtins.property
    def bds_tow_acc(self):
        """Message field 'bds_tow_acc'."""
        return self._bds_tow_acc

    @bds_tow_acc.setter
    def bds_tow_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'bds_tow_acc' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'bds_tow_acc' field must be an unsigned integer in [0, 65535]"
        self._bds_tow_acc = value

    @builtins.property
    def qzss_tow_acc(self):
        """Message field 'qzss_tow_acc'."""
        return self._qzss_tow_acc

    @qzss_tow_acc.setter
    def qzss_tow_acc(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'qzss_tow_acc' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'qzss_tow_acc' field must be an unsigned integer in [0, 65535]"
        self._qzss_tow_acc = value

    @builtins.property
    def num_sv(self):
        """Message field 'num_sv'."""
        return self._num_sv

    @num_sv.setter
    def num_sv(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_sv' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'num_sv' field must be an unsigned integer in [0, 255]"
        self._num_sv = value

    @builtins.property
    def flags(self):
        """Message field 'flags'."""
        return self._flags

    @flags.setter
    def flags(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'flags' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'flags' field must be an unsigned integer in [0, 255]"
        self._flags = value

    @builtins.property
    def sv_data(self):
        """Message field 'sv_data'."""
        return self._sv_data

    @sv_data.setter
    def sv_data(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import MeasxData
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
                 all(isinstance(v, MeasxData) for v in value) and
                 True), \
                "The 'sv_data' field must be a set or sequence and each value of type 'MeasxData'"
        self._sv_data = value
