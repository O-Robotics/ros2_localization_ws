# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXNavStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXNavStatus(type):
    """Metaclass of message 'UBXNavStatus'."""

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
                'ublox_ubx_msgs.msg.UBXNavStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_nav_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_nav_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_nav_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_nav_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_nav_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import CarrSoln
            if CarrSoln.__class__._TYPE_SUPPORT is None:
                CarrSoln.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import GpsFix
            if GpsFix.__class__._TYPE_SUPPORT is None:
                GpsFix.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import MapMatching
            if MapMatching.__class__._TYPE_SUPPORT is None:
                MapMatching.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import PSMStatus
            if PSMStatus.__class__._TYPE_SUPPORT is None:
                PSMStatus.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import SpoofDet
            if SpoofDet.__class__._TYPE_SUPPORT is None:
                SpoofDet.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UBXNavStatus(metaclass=Metaclass_UBXNavStatus):
    """Message class 'UBXNavStatus'."""

    __slots__ = [
        '_header',
        '_itow',
        '_gps_fix',
        '_gps_fix_ok',
        '_diff_soln',
        '_wkn_set',
        '_tow_set',
        '_diff_corr',
        '_carr_soln_valid',
        '_map_matching',
        '_psm',
        '_spoof_det',
        '_carr_soln',
        '_ttff',
        '_msss',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'itow': 'uint32',
        'gps_fix': 'ublox_ubx_msgs/GpsFix',
        'gps_fix_ok': 'boolean',
        'diff_soln': 'boolean',
        'wkn_set': 'boolean',
        'tow_set': 'boolean',
        'diff_corr': 'boolean',
        'carr_soln_valid': 'boolean',
        'map_matching': 'ublox_ubx_msgs/MapMatching',
        'psm': 'ublox_ubx_msgs/PSMStatus',
        'spoof_det': 'ublox_ubx_msgs/SpoofDet',
        'carr_soln': 'ublox_ubx_msgs/CarrSoln',
        'ttff': 'uint32',
        'msss': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'GpsFix'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'MapMatching'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'PSMStatus'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'SpoofDet'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'CarrSoln'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.itow = kwargs.get('itow', int())
        from ublox_ubx_msgs.msg import GpsFix
        self.gps_fix = kwargs.get('gps_fix', GpsFix())
        self.gps_fix_ok = kwargs.get('gps_fix_ok', bool())
        self.diff_soln = kwargs.get('diff_soln', bool())
        self.wkn_set = kwargs.get('wkn_set', bool())
        self.tow_set = kwargs.get('tow_set', bool())
        self.diff_corr = kwargs.get('diff_corr', bool())
        self.carr_soln_valid = kwargs.get('carr_soln_valid', bool())
        from ublox_ubx_msgs.msg import MapMatching
        self.map_matching = kwargs.get('map_matching', MapMatching())
        from ublox_ubx_msgs.msg import PSMStatus
        self.psm = kwargs.get('psm', PSMStatus())
        from ublox_ubx_msgs.msg import SpoofDet
        self.spoof_det = kwargs.get('spoof_det', SpoofDet())
        from ublox_ubx_msgs.msg import CarrSoln
        self.carr_soln = kwargs.get('carr_soln', CarrSoln())
        self.ttff = kwargs.get('ttff', int())
        self.msss = kwargs.get('msss', int())

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
        if self.gps_fix != other.gps_fix:
            return False
        if self.gps_fix_ok != other.gps_fix_ok:
            return False
        if self.diff_soln != other.diff_soln:
            return False
        if self.wkn_set != other.wkn_set:
            return False
        if self.tow_set != other.tow_set:
            return False
        if self.diff_corr != other.diff_corr:
            return False
        if self.carr_soln_valid != other.carr_soln_valid:
            return False
        if self.map_matching != other.map_matching:
            return False
        if self.psm != other.psm:
            return False
        if self.spoof_det != other.spoof_det:
            return False
        if self.carr_soln != other.carr_soln:
            return False
        if self.ttff != other.ttff:
            return False
        if self.msss != other.msss:
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
    def gps_fix(self):
        """Message field 'gps_fix'."""
        return self._gps_fix

    @gps_fix.setter
    def gps_fix(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import GpsFix
            assert \
                isinstance(value, GpsFix), \
                "The 'gps_fix' field must be a sub message of type 'GpsFix'"
        self._gps_fix = value

    @builtins.property
    def gps_fix_ok(self):
        """Message field 'gps_fix_ok'."""
        return self._gps_fix_ok

    @gps_fix_ok.setter
    def gps_fix_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'gps_fix_ok' field must be of type 'bool'"
        self._gps_fix_ok = value

    @builtins.property
    def diff_soln(self):
        """Message field 'diff_soln'."""
        return self._diff_soln

    @diff_soln.setter
    def diff_soln(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'diff_soln' field must be of type 'bool'"
        self._diff_soln = value

    @builtins.property
    def wkn_set(self):
        """Message field 'wkn_set'."""
        return self._wkn_set

    @wkn_set.setter
    def wkn_set(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'wkn_set' field must be of type 'bool'"
        self._wkn_set = value

    @builtins.property
    def tow_set(self):
        """Message field 'tow_set'."""
        return self._tow_set

    @tow_set.setter
    def tow_set(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'tow_set' field must be of type 'bool'"
        self._tow_set = value

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
    def carr_soln_valid(self):
        """Message field 'carr_soln_valid'."""
        return self._carr_soln_valid

    @carr_soln_valid.setter
    def carr_soln_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'carr_soln_valid' field must be of type 'bool'"
        self._carr_soln_valid = value

    @builtins.property
    def map_matching(self):
        """Message field 'map_matching'."""
        return self._map_matching

    @map_matching.setter
    def map_matching(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import MapMatching
            assert \
                isinstance(value, MapMatching), \
                "The 'map_matching' field must be a sub message of type 'MapMatching'"
        self._map_matching = value

    @builtins.property
    def psm(self):
        """Message field 'psm'."""
        return self._psm

    @psm.setter
    def psm(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import PSMStatus
            assert \
                isinstance(value, PSMStatus), \
                "The 'psm' field must be a sub message of type 'PSMStatus'"
        self._psm = value

    @builtins.property
    def spoof_det(self):
        """Message field 'spoof_det'."""
        return self._spoof_det

    @spoof_det.setter
    def spoof_det(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import SpoofDet
            assert \
                isinstance(value, SpoofDet), \
                "The 'spoof_det' field must be a sub message of type 'SpoofDet'"
        self._spoof_det = value

    @builtins.property
    def carr_soln(self):
        """Message field 'carr_soln'."""
        return self._carr_soln

    @carr_soln.setter
    def carr_soln(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import CarrSoln
            assert \
                isinstance(value, CarrSoln), \
                "The 'carr_soln' field must be a sub message of type 'CarrSoln'"
        self._carr_soln = value

    @builtins.property
    def ttff(self):
        """Message field 'ttff'."""
        return self._ttff

    @ttff.setter
    def ttff(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ttff' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'ttff' field must be an unsigned integer in [0, 4294967295]"
        self._ttff = value

    @builtins.property
    def msss(self):
        """Message field 'msss'."""
        return self._msss

    @msss.setter
    def msss(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'msss' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'msss' field must be an unsigned integer in [0, 4294967295]"
        self._msss = value
