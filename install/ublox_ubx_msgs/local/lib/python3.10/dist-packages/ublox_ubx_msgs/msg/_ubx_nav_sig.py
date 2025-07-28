# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXNavSig.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

# Member 'reserved_0'
import numpy  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXNavSig(type):
    """Metaclass of message 'UBXNavSig'."""

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
                'ublox_ubx_msgs.msg.UBXNavSig')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_nav_sig
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_nav_sig
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_nav_sig
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_nav_sig
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_nav_sig

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import SigData
            if SigData.__class__._TYPE_SUPPORT is None:
                SigData.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UBXNavSig(metaclass=Metaclass_UBXNavSig):
    """Message class 'UBXNavSig'."""

    __slots__ = [
        '_header',
        '_itow',
        '_version',
        '_num_sigs',
        '_reserved_0',
        '_sig_data',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'itow': 'uint32',
        'version': 'uint8',
        'num_sigs': 'uint8',
        'reserved_0': 'uint8[2]',
        'sig_data': 'sequence<ublox_ubx_msgs/SigData>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.Array(rosidl_parser.definition.BasicType('uint8'), 2),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'SigData')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.itow = kwargs.get('itow', int())
        self.version = kwargs.get('version', int())
        self.num_sigs = kwargs.get('num_sigs', int())
        if 'reserved_0' not in kwargs:
            self.reserved_0 = numpy.zeros(2, dtype=numpy.uint8)
        else:
            self.reserved_0 = numpy.array(kwargs.get('reserved_0'), dtype=numpy.uint8)
            assert self.reserved_0.shape == (2, )
        self.sig_data = kwargs.get('sig_data', [])

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
        if self.version != other.version:
            return False
        if self.num_sigs != other.num_sigs:
            return False
        if all(self.reserved_0 != other.reserved_0):
            return False
        if self.sig_data != other.sig_data:
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
    def num_sigs(self):
        """Message field 'num_sigs'."""
        return self._num_sigs

    @num_sigs.setter
    def num_sigs(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_sigs' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'num_sigs' field must be an unsigned integer in [0, 255]"
        self._num_sigs = value

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
    def sig_data(self):
        """Message field 'sig_data'."""
        return self._sig_data

    @sig_data.setter
    def sig_data(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import SigData
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
                 all(isinstance(v, SigData) for v in value) and
                 True), \
                "The 'sig_data' field must be a set or sequence and each value of type 'SigData'"
        self._sig_data = value
