# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/SatFlags.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SatFlags(type):
    """Metaclass of message 'SatFlags'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'QUALITY_NO_SIGNAL': 0,
        'QUALITY_SEARCHING': 1,
        'QUALITY_ACQUIRED': 2,
        'QUALITY_UNUSABLE': 3,
        'QUALITY_CODE_LOCKED': 4,
        'QUALITY_CARRIER_LOCKED': 5,
        'HEALTH_UNKNOWN': 0,
        'HEALTH_HEALTHY': 1,
        'HEALTH_UNHEALTHY': 2,
        'ORBIT_NO_INFO': 0,
        'ORBIT_EPH_USED': 1,
        'ORBIT_ALM_USED': 2,
        'ORBIT_ASSISTNOW_OFFLINE': 3,
        'ORBIT_ASSISTNOW_AUTONOMOUS': 4,
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
                'ublox_ubx_msgs.msg.SatFlags')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sat_flags
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sat_flags
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sat_flags
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sat_flags
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sat_flags

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'QUALITY_NO_SIGNAL': cls.__constants['QUALITY_NO_SIGNAL'],
            'QUALITY_SEARCHING': cls.__constants['QUALITY_SEARCHING'],
            'QUALITY_ACQUIRED': cls.__constants['QUALITY_ACQUIRED'],
            'QUALITY_UNUSABLE': cls.__constants['QUALITY_UNUSABLE'],
            'QUALITY_CODE_LOCKED': cls.__constants['QUALITY_CODE_LOCKED'],
            'QUALITY_CARRIER_LOCKED': cls.__constants['QUALITY_CARRIER_LOCKED'],
            'HEALTH_UNKNOWN': cls.__constants['HEALTH_UNKNOWN'],
            'HEALTH_HEALTHY': cls.__constants['HEALTH_HEALTHY'],
            'HEALTH_UNHEALTHY': cls.__constants['HEALTH_UNHEALTHY'],
            'ORBIT_NO_INFO': cls.__constants['ORBIT_NO_INFO'],
            'ORBIT_EPH_USED': cls.__constants['ORBIT_EPH_USED'],
            'ORBIT_ALM_USED': cls.__constants['ORBIT_ALM_USED'],
            'ORBIT_ASSISTNOW_OFFLINE': cls.__constants['ORBIT_ASSISTNOW_OFFLINE'],
            'ORBIT_ASSISTNOW_AUTONOMOUS': cls.__constants['ORBIT_ASSISTNOW_AUTONOMOUS'],
        }

    @property
    def QUALITY_NO_SIGNAL(self):
        """Message constant 'QUALITY_NO_SIGNAL'."""
        return Metaclass_SatFlags.__constants['QUALITY_NO_SIGNAL']

    @property
    def QUALITY_SEARCHING(self):
        """Message constant 'QUALITY_SEARCHING'."""
        return Metaclass_SatFlags.__constants['QUALITY_SEARCHING']

    @property
    def QUALITY_ACQUIRED(self):
        """Message constant 'QUALITY_ACQUIRED'."""
        return Metaclass_SatFlags.__constants['QUALITY_ACQUIRED']

    @property
    def QUALITY_UNUSABLE(self):
        """Message constant 'QUALITY_UNUSABLE'."""
        return Metaclass_SatFlags.__constants['QUALITY_UNUSABLE']

    @property
    def QUALITY_CODE_LOCKED(self):
        """Message constant 'QUALITY_CODE_LOCKED'."""
        return Metaclass_SatFlags.__constants['QUALITY_CODE_LOCKED']

    @property
    def QUALITY_CARRIER_LOCKED(self):
        """Message constant 'QUALITY_CARRIER_LOCKED'."""
        return Metaclass_SatFlags.__constants['QUALITY_CARRIER_LOCKED']

    @property
    def HEALTH_UNKNOWN(self):
        """Message constant 'HEALTH_UNKNOWN'."""
        return Metaclass_SatFlags.__constants['HEALTH_UNKNOWN']

    @property
    def HEALTH_HEALTHY(self):
        """Message constant 'HEALTH_HEALTHY'."""
        return Metaclass_SatFlags.__constants['HEALTH_HEALTHY']

    @property
    def HEALTH_UNHEALTHY(self):
        """Message constant 'HEALTH_UNHEALTHY'."""
        return Metaclass_SatFlags.__constants['HEALTH_UNHEALTHY']

    @property
    def ORBIT_NO_INFO(self):
        """Message constant 'ORBIT_NO_INFO'."""
        return Metaclass_SatFlags.__constants['ORBIT_NO_INFO']

    @property
    def ORBIT_EPH_USED(self):
        """Message constant 'ORBIT_EPH_USED'."""
        return Metaclass_SatFlags.__constants['ORBIT_EPH_USED']

    @property
    def ORBIT_ALM_USED(self):
        """Message constant 'ORBIT_ALM_USED'."""
        return Metaclass_SatFlags.__constants['ORBIT_ALM_USED']

    @property
    def ORBIT_ASSISTNOW_OFFLINE(self):
        """Message constant 'ORBIT_ASSISTNOW_OFFLINE'."""
        return Metaclass_SatFlags.__constants['ORBIT_ASSISTNOW_OFFLINE']

    @property
    def ORBIT_ASSISTNOW_AUTONOMOUS(self):
        """Message constant 'ORBIT_ASSISTNOW_AUTONOMOUS'."""
        return Metaclass_SatFlags.__constants['ORBIT_ASSISTNOW_AUTONOMOUS']


class SatFlags(metaclass=Metaclass_SatFlags):
    """
    Message class 'SatFlags'.

    Constants:
      QUALITY_NO_SIGNAL
      QUALITY_SEARCHING
      QUALITY_ACQUIRED
      QUALITY_UNUSABLE
      QUALITY_CODE_LOCKED
      QUALITY_CARRIER_LOCKED
      HEALTH_UNKNOWN
      HEALTH_HEALTHY
      HEALTH_UNHEALTHY
      ORBIT_NO_INFO
      ORBIT_EPH_USED
      ORBIT_ALM_USED
      ORBIT_ASSISTNOW_OFFLINE
      ORBIT_ASSISTNOW_AUTONOMOUS
    """

    __slots__ = [
        '_quality_ind',
        '_sv_used',
        '_health',
        '_diff_corr',
        '_smoothed',
        '_orbit_source',
        '_eph_avail',
        '_alm_avail',
        '_ano_avail',
        '_aop_avail',
        '_sbas_corr_used',
        '_rtcm_corr_used',
        '_slas_corr_used',
        '_spartn_corr_used',
        '_pr_corr_used',
        '_cr_corr_used',
        '_do_corr_used',
        '_clas_corr_used',
    ]

    _fields_and_field_types = {
        'quality_ind': 'uint8',
        'sv_used': 'boolean',
        'health': 'uint8',
        'diff_corr': 'boolean',
        'smoothed': 'boolean',
        'orbit_source': 'uint8',
        'eph_avail': 'boolean',
        'alm_avail': 'boolean',
        'ano_avail': 'boolean',
        'aop_avail': 'boolean',
        'sbas_corr_used': 'boolean',
        'rtcm_corr_used': 'boolean',
        'slas_corr_used': 'boolean',
        'spartn_corr_used': 'boolean',
        'pr_corr_used': 'boolean',
        'cr_corr_used': 'boolean',
        'do_corr_used': 'boolean',
        'clas_corr_used': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.quality_ind = kwargs.get('quality_ind', int())
        self.sv_used = kwargs.get('sv_used', bool())
        self.health = kwargs.get('health', int())
        self.diff_corr = kwargs.get('diff_corr', bool())
        self.smoothed = kwargs.get('smoothed', bool())
        self.orbit_source = kwargs.get('orbit_source', int())
        self.eph_avail = kwargs.get('eph_avail', bool())
        self.alm_avail = kwargs.get('alm_avail', bool())
        self.ano_avail = kwargs.get('ano_avail', bool())
        self.aop_avail = kwargs.get('aop_avail', bool())
        self.sbas_corr_used = kwargs.get('sbas_corr_used', bool())
        self.rtcm_corr_used = kwargs.get('rtcm_corr_used', bool())
        self.slas_corr_used = kwargs.get('slas_corr_used', bool())
        self.spartn_corr_used = kwargs.get('spartn_corr_used', bool())
        self.pr_corr_used = kwargs.get('pr_corr_used', bool())
        self.cr_corr_used = kwargs.get('cr_corr_used', bool())
        self.do_corr_used = kwargs.get('do_corr_used', bool())
        self.clas_corr_used = kwargs.get('clas_corr_used', bool())

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
        if self.quality_ind != other.quality_ind:
            return False
        if self.sv_used != other.sv_used:
            return False
        if self.health != other.health:
            return False
        if self.diff_corr != other.diff_corr:
            return False
        if self.smoothed != other.smoothed:
            return False
        if self.orbit_source != other.orbit_source:
            return False
        if self.eph_avail != other.eph_avail:
            return False
        if self.alm_avail != other.alm_avail:
            return False
        if self.ano_avail != other.ano_avail:
            return False
        if self.aop_avail != other.aop_avail:
            return False
        if self.sbas_corr_used != other.sbas_corr_used:
            return False
        if self.rtcm_corr_used != other.rtcm_corr_used:
            return False
        if self.slas_corr_used != other.slas_corr_used:
            return False
        if self.spartn_corr_used != other.spartn_corr_used:
            return False
        if self.pr_corr_used != other.pr_corr_used:
            return False
        if self.cr_corr_used != other.cr_corr_used:
            return False
        if self.do_corr_used != other.do_corr_used:
            return False
        if self.clas_corr_used != other.clas_corr_used:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def quality_ind(self):
        """Message field 'quality_ind'."""
        return self._quality_ind

    @quality_ind.setter
    def quality_ind(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'quality_ind' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'quality_ind' field must be an unsigned integer in [0, 255]"
        self._quality_ind = value

    @builtins.property
    def sv_used(self):
        """Message field 'sv_used'."""
        return self._sv_used

    @sv_used.setter
    def sv_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sv_used' field must be of type 'bool'"
        self._sv_used = value

    @builtins.property
    def health(self):
        """Message field 'health'."""
        return self._health

    @health.setter
    def health(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'health' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'health' field must be an unsigned integer in [0, 255]"
        self._health = value

    @builtins.property
    def diff_corr(self):
        """Message field 'diff_corr'."""
        return self._diff_corr

    @diff_corr.setter
    def diff_corr(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'diff_corr' field must be of type 'bool'"
        self._diff_corr = value

    @builtins.property
    def smoothed(self):
        """Message field 'smoothed'."""
        return self._smoothed

    @smoothed.setter
    def smoothed(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'smoothed' field must be of type 'bool'"
        self._smoothed = value

    @builtins.property
    def orbit_source(self):
        """Message field 'orbit_source'."""
        return self._orbit_source

    @orbit_source.setter
    def orbit_source(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'orbit_source' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'orbit_source' field must be an unsigned integer in [0, 255]"
        self._orbit_source = value

    @builtins.property
    def eph_avail(self):
        """Message field 'eph_avail'."""
        return self._eph_avail

    @eph_avail.setter
    def eph_avail(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'eph_avail' field must be of type 'bool'"
        self._eph_avail = value

    @builtins.property
    def alm_avail(self):
        """Message field 'alm_avail'."""
        return self._alm_avail

    @alm_avail.setter
    def alm_avail(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'alm_avail' field must be of type 'bool'"
        self._alm_avail = value

    @builtins.property
    def ano_avail(self):
        """Message field 'ano_avail'."""
        return self._ano_avail

    @ano_avail.setter
    def ano_avail(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ano_avail' field must be of type 'bool'"
        self._ano_avail = value

    @builtins.property
    def aop_avail(self):
        """Message field 'aop_avail'."""
        return self._aop_avail

    @aop_avail.setter
    def aop_avail(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'aop_avail' field must be of type 'bool'"
        self._aop_avail = value

    @builtins.property
    def sbas_corr_used(self):
        """Message field 'sbas_corr_used'."""
        return self._sbas_corr_used

    @sbas_corr_used.setter
    def sbas_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'sbas_corr_used' field must be of type 'bool'"
        self._sbas_corr_used = value

    @builtins.property
    def rtcm_corr_used(self):
        """Message field 'rtcm_corr_used'."""
        return self._rtcm_corr_used

    @rtcm_corr_used.setter
    def rtcm_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'rtcm_corr_used' field must be of type 'bool'"
        self._rtcm_corr_used = value

    @builtins.property
    def slas_corr_used(self):
        """Message field 'slas_corr_used'."""
        return self._slas_corr_used

    @slas_corr_used.setter
    def slas_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'slas_corr_used' field must be of type 'bool'"
        self._slas_corr_used = value

    @builtins.property
    def spartn_corr_used(self):
        """Message field 'spartn_corr_used'."""
        return self._spartn_corr_used

    @spartn_corr_used.setter
    def spartn_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'spartn_corr_used' field must be of type 'bool'"
        self._spartn_corr_used = value

    @builtins.property
    def pr_corr_used(self):
        """Message field 'pr_corr_used'."""
        return self._pr_corr_used

    @pr_corr_used.setter
    def pr_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pr_corr_used' field must be of type 'bool'"
        self._pr_corr_used = value

    @builtins.property
    def cr_corr_used(self):
        """Message field 'cr_corr_used'."""
        return self._cr_corr_used

    @cr_corr_used.setter
    def cr_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'cr_corr_used' field must be of type 'bool'"
        self._cr_corr_used = value

    @builtins.property
    def do_corr_used(self):
        """Message field 'do_corr_used'."""
        return self._do_corr_used

    @do_corr_used.setter
    def do_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'do_corr_used' field must be of type 'bool'"
        self._do_corr_used = value

    @builtins.property
    def clas_corr_used(self):
        """Message field 'clas_corr_used'."""
        return self._clas_corr_used

    @clas_corr_used.setter
    def clas_corr_used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'clas_corr_used' field must be of type 'bool'"
        self._clas_corr_used = value
