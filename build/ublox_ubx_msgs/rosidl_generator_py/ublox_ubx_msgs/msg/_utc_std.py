# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UtcStd.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UtcStd(type):
    """Metaclass of message 'UtcStd'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'NOT_AVAILABLE': 0,
        'CRL': 1,
        'NIST': 2,
        'USNO': 3,
        'BIPM': 4,
        'EURO': 5,
        'SU': 6,
        'NTSC': 7,
        'UTC_UNKNOWN': 15,
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
                'ublox_ubx_msgs.msg.UtcStd')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__utc_std
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__utc_std
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__utc_std
            cls._TYPE_SUPPORT = module.type_support_msg__msg__utc_std
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__utc_std

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'NOT_AVAILABLE': cls.__constants['NOT_AVAILABLE'],
            'CRL': cls.__constants['CRL'],
            'NIST': cls.__constants['NIST'],
            'USNO': cls.__constants['USNO'],
            'BIPM': cls.__constants['BIPM'],
            'EURO': cls.__constants['EURO'],
            'SU': cls.__constants['SU'],
            'NTSC': cls.__constants['NTSC'],
            'UTC_UNKNOWN': cls.__constants['UTC_UNKNOWN'],
        }

    @property
    def NOT_AVAILABLE(self):
        """Message constant 'NOT_AVAILABLE'."""
        return Metaclass_UtcStd.__constants['NOT_AVAILABLE']

    @property
    def CRL(self):
        """Message constant 'CRL'."""
        return Metaclass_UtcStd.__constants['CRL']

    @property
    def NIST(self):
        """Message constant 'NIST'."""
        return Metaclass_UtcStd.__constants['NIST']

    @property
    def USNO(self):
        """Message constant 'USNO'."""
        return Metaclass_UtcStd.__constants['USNO']

    @property
    def BIPM(self):
        """Message constant 'BIPM'."""
        return Metaclass_UtcStd.__constants['BIPM']

    @property
    def EURO(self):
        """Message constant 'EURO'."""
        return Metaclass_UtcStd.__constants['EURO']

    @property
    def SU(self):
        """Message constant 'SU'."""
        return Metaclass_UtcStd.__constants['SU']

    @property
    def NTSC(self):
        """Message constant 'NTSC'."""
        return Metaclass_UtcStd.__constants['NTSC']

    @property
    def UTC_UNKNOWN(self):
        """Message constant 'UTC_UNKNOWN'."""
        return Metaclass_UtcStd.__constants['UTC_UNKNOWN']


class UtcStd(metaclass=Metaclass_UtcStd):
    """
    Message class 'UtcStd'.

    Constants:
      NOT_AVAILABLE
      CRL
      NIST
      USNO
      BIPM
      EURO
      SU
      NTSC
      UTC_UNKNOWN
    """

    __slots__ = [
        '_id',
    ]

    _fields_and_field_types = {
        'id': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.id = kwargs.get('id', int())

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
        if self.id != other.id:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
            assert value >= 0 and value < 256, \
                "The 'id' field must be an unsigned integer in [0, 255]"
        self._id = value
