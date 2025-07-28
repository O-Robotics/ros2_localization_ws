# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/OrbSVInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_OrbSVInfo(type):
    """Metaclass of message 'OrbSVInfo'."""

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
                'ublox_ubx_msgs.msg.OrbSVInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__orb_sv_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__orb_sv_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__orb_sv_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__orb_sv_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__orb_sv_info

            from ublox_ubx_msgs.msg import OrbAlmInfo
            if OrbAlmInfo.__class__._TYPE_SUPPORT is None:
                OrbAlmInfo.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import OrbEphInfo
            if OrbEphInfo.__class__._TYPE_SUPPORT is None:
                OrbEphInfo.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import OrbSVFlag
            if OrbSVFlag.__class__._TYPE_SUPPORT is None:
                OrbSVFlag.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import OtherOrbInfo
            if OtherOrbInfo.__class__._TYPE_SUPPORT is None:
                OtherOrbInfo.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class OrbSVInfo(metaclass=Metaclass_OrbSVInfo):
    """Message class 'OrbSVInfo'."""

    __slots__ = [
        '_gnss_id',
        '_sv_id',
        '_sv_flag',
        '_eph',
        '_alm',
        '_other_orb',
    ]

    _fields_and_field_types = {
        'gnss_id': 'uint8',
        'sv_id': 'uint8',
        'sv_flag': 'ublox_ubx_msgs/OrbSVFlag',
        'eph': 'ublox_ubx_msgs/OrbEphInfo',
        'alm': 'ublox_ubx_msgs/OrbAlmInfo',
        'other_orb': 'ublox_ubx_msgs/OtherOrbInfo',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'OrbSVFlag'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'OrbEphInfo'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'OrbAlmInfo'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'OtherOrbInfo'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.gnss_id = kwargs.get('gnss_id', int())
        self.sv_id = kwargs.get('sv_id', int())
        from ublox_ubx_msgs.msg import OrbSVFlag
        self.sv_flag = kwargs.get('sv_flag', OrbSVFlag())
        from ublox_ubx_msgs.msg import OrbEphInfo
        self.eph = kwargs.get('eph', OrbEphInfo())
        from ublox_ubx_msgs.msg import OrbAlmInfo
        self.alm = kwargs.get('alm', OrbAlmInfo())
        from ublox_ubx_msgs.msg import OtherOrbInfo
        self.other_orb = kwargs.get('other_orb', OtherOrbInfo())

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
        if self.gnss_id != other.gnss_id:
            return False
        if self.sv_id != other.sv_id:
            return False
        if self.sv_flag != other.sv_flag:
            return False
        if self.eph != other.eph:
            return False
        if self.alm != other.alm:
            return False
        if self.other_orb != other.other_orb:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def gnss_id(self):
        """Message field 'gnss_id'."""
        return self._gnss_id

    @gnss_id.setter
    def gnss_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'gnss_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'gnss_id' field must be an unsigned integer in [0, 255]"
        self._gnss_id = value

    @builtins.property
    def sv_id(self):
        """Message field 'sv_id'."""
        return self._sv_id

    @sv_id.setter
    def sv_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sv_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sv_id' field must be an unsigned integer in [0, 255]"
        self._sv_id = value

    @builtins.property
    def sv_flag(self):
        """Message field 'sv_flag'."""
        return self._sv_flag

    @sv_flag.setter
    def sv_flag(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import OrbSVFlag
            assert \
                isinstance(value, OrbSVFlag), \
                "The 'sv_flag' field must be a sub message of type 'OrbSVFlag'"
        self._sv_flag = value

    @builtins.property
    def eph(self):
        """Message field 'eph'."""
        return self._eph

    @eph.setter
    def eph(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import OrbEphInfo
            assert \
                isinstance(value, OrbEphInfo), \
                "The 'eph' field must be a sub message of type 'OrbEphInfo'"
        self._eph = value

    @builtins.property
    def alm(self):
        """Message field 'alm'."""
        return self._alm

    @alm.setter
    def alm(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import OrbAlmInfo
            assert \
                isinstance(value, OrbAlmInfo), \
                "The 'alm' field must be a sub message of type 'OrbAlmInfo'"
        self._alm = value

    @builtins.property
    def other_orb(self):
        """Message field 'other_orb'."""
        return self._other_orb

    @other_orb.setter
    def other_orb(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import OtherOrbInfo
            assert \
                isinstance(value, OtherOrbInfo), \
                "The 'other_orb' field must be a sub message of type 'OtherOrbInfo'"
        self._other_orb = value
