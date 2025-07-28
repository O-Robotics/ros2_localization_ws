# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/OrbEphInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OrbEphInfo(type):
    """Metaclass of message 'OrbEphInfo'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'EPH_USABILITY_UNKNOWN': 31,
        'EPH_USABILITY_OVER_450_MIN': 30,
        'EPH_USABILITY_EXPIRED': 0,
        'EPH_SOURCE_NOT_AVAILABLE': 0,
        'EPH_SOURCE_GNSS_TRANSMISSION': 1,
        'EPH_SOURCE_EXTERNAL_AIDING': 2,
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
                'ublox_ubx_msgs.msg.OrbEphInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__orb_eph_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__orb_eph_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__orb_eph_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__orb_eph_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__orb_eph_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'EPH_USABILITY_UNKNOWN': cls.__constants['EPH_USABILITY_UNKNOWN'],
            'EPH_USABILITY_OVER_450_MIN': cls.__constants['EPH_USABILITY_OVER_450_MIN'],
            'EPH_USABILITY_EXPIRED': cls.__constants['EPH_USABILITY_EXPIRED'],
            'EPH_SOURCE_NOT_AVAILABLE': cls.__constants['EPH_SOURCE_NOT_AVAILABLE'],
            'EPH_SOURCE_GNSS_TRANSMISSION': cls.__constants['EPH_SOURCE_GNSS_TRANSMISSION'],
            'EPH_SOURCE_EXTERNAL_AIDING': cls.__constants['EPH_SOURCE_EXTERNAL_AIDING'],
        }

    @property
    def EPH_USABILITY_UNKNOWN(self):
        """Message constant 'EPH_USABILITY_UNKNOWN'."""
        return Metaclass_OrbEphInfo.__constants['EPH_USABILITY_UNKNOWN']

    @property
    def EPH_USABILITY_OVER_450_MIN(self):
        """Message constant 'EPH_USABILITY_OVER_450_MIN'."""
        return Metaclass_OrbEphInfo.__constants['EPH_USABILITY_OVER_450_MIN']

    @property
    def EPH_USABILITY_EXPIRED(self):
        """Message constant 'EPH_USABILITY_EXPIRED'."""
        return Metaclass_OrbEphInfo.__constants['EPH_USABILITY_EXPIRED']

    @property
    def EPH_SOURCE_NOT_AVAILABLE(self):
        """Message constant 'EPH_SOURCE_NOT_AVAILABLE'."""
        return Metaclass_OrbEphInfo.__constants['EPH_SOURCE_NOT_AVAILABLE']

    @property
    def EPH_SOURCE_GNSS_TRANSMISSION(self):
        """Message constant 'EPH_SOURCE_GNSS_TRANSMISSION'."""
        return Metaclass_OrbEphInfo.__constants['EPH_SOURCE_GNSS_TRANSMISSION']

    @property
    def EPH_SOURCE_EXTERNAL_AIDING(self):
        """Message constant 'EPH_SOURCE_EXTERNAL_AIDING'."""
        return Metaclass_OrbEphInfo.__constants['EPH_SOURCE_EXTERNAL_AIDING']


class OrbEphInfo(metaclass=Metaclass_OrbEphInfo):
    """
    Message class 'OrbEphInfo'.

    Constants:
      EPH_USABILITY_UNKNOWN
      EPH_USABILITY_OVER_450_MIN
      EPH_USABILITY_EXPIRED
      EPH_SOURCE_NOT_AVAILABLE
      EPH_SOURCE_GNSS_TRANSMISSION
      EPH_SOURCE_EXTERNAL_AIDING
    """

    __slots__ = [
        '_eph_usability',
        '_eph_source',
    ]

    _fields_and_field_types = {
        'eph_usability': 'uint8',
        'eph_source': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.eph_usability = kwargs.get('eph_usability', int())
        self.eph_source = kwargs.get('eph_source', int())

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
        if self.eph_usability != other.eph_usability:
            return False
        if self.eph_source != other.eph_source:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def eph_usability(self):
        """Message field 'eph_usability'."""
        return self._eph_usability

    @eph_usability.setter
    def eph_usability(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'eph_usability' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'eph_usability' field must be an unsigned integer in [0, 255]"
        self._eph_usability = value

    @builtins.property
    def eph_source(self):
        """Message field 'eph_source'."""
        return self._eph_source

    @eph_source.setter
    def eph_source(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'eph_source' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'eph_source' field must be an unsigned integer in [0, 255]"
        self._eph_source = value
