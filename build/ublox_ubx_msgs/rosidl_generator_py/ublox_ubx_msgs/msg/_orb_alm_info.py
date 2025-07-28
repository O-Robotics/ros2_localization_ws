# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/OrbAlmInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OrbAlmInfo(type):
    """Metaclass of message 'OrbAlmInfo'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ALM_USABILITY_UNKNOWN': 31,
        'ALM_USABILITY_OVER_30_DAYS': 30,
        'ALM_USABILITY_EXPIRED': 0,
        'ALM_SOURCE_NOT_AVAILABLE': 0,
        'ALM_SOURCE_GNSS_TRANSMISSION': 1,
        'ALM_SOURCE_EXTERNAL_AIDING': 2,
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
                'ublox_ubx_msgs.msg.OrbAlmInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__orb_alm_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__orb_alm_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__orb_alm_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__orb_alm_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__orb_alm_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ALM_USABILITY_UNKNOWN': cls.__constants['ALM_USABILITY_UNKNOWN'],
            'ALM_USABILITY_OVER_30_DAYS': cls.__constants['ALM_USABILITY_OVER_30_DAYS'],
            'ALM_USABILITY_EXPIRED': cls.__constants['ALM_USABILITY_EXPIRED'],
            'ALM_SOURCE_NOT_AVAILABLE': cls.__constants['ALM_SOURCE_NOT_AVAILABLE'],
            'ALM_SOURCE_GNSS_TRANSMISSION': cls.__constants['ALM_SOURCE_GNSS_TRANSMISSION'],
            'ALM_SOURCE_EXTERNAL_AIDING': cls.__constants['ALM_SOURCE_EXTERNAL_AIDING'],
        }

    @property
    def ALM_USABILITY_UNKNOWN(self):
        """Message constant 'ALM_USABILITY_UNKNOWN'."""
        return Metaclass_OrbAlmInfo.__constants['ALM_USABILITY_UNKNOWN']

    @property
    def ALM_USABILITY_OVER_30_DAYS(self):
        """Message constant 'ALM_USABILITY_OVER_30_DAYS'."""
        return Metaclass_OrbAlmInfo.__constants['ALM_USABILITY_OVER_30_DAYS']

    @property
    def ALM_USABILITY_EXPIRED(self):
        """Message constant 'ALM_USABILITY_EXPIRED'."""
        return Metaclass_OrbAlmInfo.__constants['ALM_USABILITY_EXPIRED']

    @property
    def ALM_SOURCE_NOT_AVAILABLE(self):
        """Message constant 'ALM_SOURCE_NOT_AVAILABLE'."""
        return Metaclass_OrbAlmInfo.__constants['ALM_SOURCE_NOT_AVAILABLE']

    @property
    def ALM_SOURCE_GNSS_TRANSMISSION(self):
        """Message constant 'ALM_SOURCE_GNSS_TRANSMISSION'."""
        return Metaclass_OrbAlmInfo.__constants['ALM_SOURCE_GNSS_TRANSMISSION']

    @property
    def ALM_SOURCE_EXTERNAL_AIDING(self):
        """Message constant 'ALM_SOURCE_EXTERNAL_AIDING'."""
        return Metaclass_OrbAlmInfo.__constants['ALM_SOURCE_EXTERNAL_AIDING']


class OrbAlmInfo(metaclass=Metaclass_OrbAlmInfo):
    """
    Message class 'OrbAlmInfo'.

    Constants:
      ALM_USABILITY_UNKNOWN
      ALM_USABILITY_OVER_30_DAYS
      ALM_USABILITY_EXPIRED
      ALM_SOURCE_NOT_AVAILABLE
      ALM_SOURCE_GNSS_TRANSMISSION
      ALM_SOURCE_EXTERNAL_AIDING
    """

    __slots__ = [
        '_alm_usability',
        '_alm_source',
    ]

    _fields_and_field_types = {
        'alm_usability': 'uint8',
        'alm_source': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.alm_usability = kwargs.get('alm_usability', int())
        self.alm_source = kwargs.get('alm_source', int())

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
        if self.alm_usability != other.alm_usability:
            return False
        if self.alm_source != other.alm_source:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def alm_usability(self):
        """Message field 'alm_usability'."""
        return self._alm_usability

    @alm_usability.setter
    def alm_usability(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'alm_usability' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'alm_usability' field must be an unsigned integer in [0, 255]"
        self._alm_usability = value

    @builtins.property
    def alm_source(self):
        """Message field 'alm_source'."""
        return self._alm_source

    @alm_source.setter
    def alm_source(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'alm_source' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'alm_source' field must be an unsigned integer in [0, 255]"
        self._alm_source = value
