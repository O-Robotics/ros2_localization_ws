# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/OtherOrbInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OtherOrbInfo(type):
    """Metaclass of message 'OtherOrbInfo'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'ANO_AOP_USABILITY_UNKNOWN': 31,
        'ANO_AOP_USABILITY_OVER_30_DAYS': 30,
        'ANO_AOP_USABILITY_EXPIRED': 0,
        'TYPE_NO_ORBIT_DATA': 0,
        'TYPE_ASSISTNOW_OFFLINE': 1,
        'TYPE_ASSISTNOW_AUTONOMOUS': 2,
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
                'ublox_ubx_msgs.msg.OtherOrbInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__other_orb_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__other_orb_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__other_orb_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__other_orb_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__other_orb_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'ANO_AOP_USABILITY_UNKNOWN': cls.__constants['ANO_AOP_USABILITY_UNKNOWN'],
            'ANO_AOP_USABILITY_OVER_30_DAYS': cls.__constants['ANO_AOP_USABILITY_OVER_30_DAYS'],
            'ANO_AOP_USABILITY_EXPIRED': cls.__constants['ANO_AOP_USABILITY_EXPIRED'],
            'TYPE_NO_ORBIT_DATA': cls.__constants['TYPE_NO_ORBIT_DATA'],
            'TYPE_ASSISTNOW_OFFLINE': cls.__constants['TYPE_ASSISTNOW_OFFLINE'],
            'TYPE_ASSISTNOW_AUTONOMOUS': cls.__constants['TYPE_ASSISTNOW_AUTONOMOUS'],
        }

    @property
    def ANO_AOP_USABILITY_UNKNOWN(self):
        """Message constant 'ANO_AOP_USABILITY_UNKNOWN'."""
        return Metaclass_OtherOrbInfo.__constants['ANO_AOP_USABILITY_UNKNOWN']

    @property
    def ANO_AOP_USABILITY_OVER_30_DAYS(self):
        """Message constant 'ANO_AOP_USABILITY_OVER_30_DAYS'."""
        return Metaclass_OtherOrbInfo.__constants['ANO_AOP_USABILITY_OVER_30_DAYS']

    @property
    def ANO_AOP_USABILITY_EXPIRED(self):
        """Message constant 'ANO_AOP_USABILITY_EXPIRED'."""
        return Metaclass_OtherOrbInfo.__constants['ANO_AOP_USABILITY_EXPIRED']

    @property
    def TYPE_NO_ORBIT_DATA(self):
        """Message constant 'TYPE_NO_ORBIT_DATA'."""
        return Metaclass_OtherOrbInfo.__constants['TYPE_NO_ORBIT_DATA']

    @property
    def TYPE_ASSISTNOW_OFFLINE(self):
        """Message constant 'TYPE_ASSISTNOW_OFFLINE'."""
        return Metaclass_OtherOrbInfo.__constants['TYPE_ASSISTNOW_OFFLINE']

    @property
    def TYPE_ASSISTNOW_AUTONOMOUS(self):
        """Message constant 'TYPE_ASSISTNOW_AUTONOMOUS'."""
        return Metaclass_OtherOrbInfo.__constants['TYPE_ASSISTNOW_AUTONOMOUS']


class OtherOrbInfo(metaclass=Metaclass_OtherOrbInfo):
    """
    Message class 'OtherOrbInfo'.

    Constants:
      ANO_AOP_USABILITY_UNKNOWN
      ANO_AOP_USABILITY_OVER_30_DAYS
      ANO_AOP_USABILITY_EXPIRED
      TYPE_NO_ORBIT_DATA
      TYPE_ASSISTNOW_OFFLINE
      TYPE_ASSISTNOW_AUTONOMOUS
    """

    __slots__ = [
        '_ano_aop_usability',
        '_orb_type',
    ]

    _fields_and_field_types = {
        'ano_aop_usability': 'uint8',
        'orb_type': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.ano_aop_usability = kwargs.get('ano_aop_usability', int())
        self.orb_type = kwargs.get('orb_type', int())

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
        if self.ano_aop_usability != other.ano_aop_usability:
            return False
        if self.orb_type != other.orb_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ano_aop_usability(self):
        """Message field 'ano_aop_usability'."""
        return self._ano_aop_usability

    @ano_aop_usability.setter
    def ano_aop_usability(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ano_aop_usability' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'ano_aop_usability' field must be an unsigned integer in [0, 255]"
        self._ano_aop_usability = value

    @builtins.property
    def orb_type(self):
        """Message field 'orb_type'."""
        return self._orb_type

    @orb_type.setter
    def orb_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'orb_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'orb_type' field must be an unsigned integer in [0, 255]"
        self._orb_type = value
