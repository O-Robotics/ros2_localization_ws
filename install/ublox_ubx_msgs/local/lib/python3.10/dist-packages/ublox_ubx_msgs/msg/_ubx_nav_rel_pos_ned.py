# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXNavRelPosNED.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXNavRelPosNED(type):
    """Metaclass of message 'UBXNavRelPosNED'."""

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
                'ublox_ubx_msgs.msg.UBXNavRelPosNED')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_nav_rel_pos_ned
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_nav_rel_pos_ned
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_nav_rel_pos_ned
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_nav_rel_pos_ned
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_nav_rel_pos_ned

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import CarrSoln
            if CarrSoln.__class__._TYPE_SUPPORT is None:
                CarrSoln.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UBXNavRelPosNED(metaclass=Metaclass_UBXNavRelPosNED):
    """Message class 'UBXNavRelPosNED'."""

    __slots__ = [
        '_header',
        '_version',
        '_ref_station_id',
        '_itow',
        '_rel_pos_n',
        '_rel_pos_e',
        '_rel_pos_d',
        '_rel_pos_length',
        '_rel_pos_heading',
        '_rel_pos_hp_n',
        '_rel_pos_hp_e',
        '_rel_pos_hp_d',
        '_rel_pos_hp_length',
        '_acc_n',
        '_acc_e',
        '_acc_d',
        '_acc_length',
        '_acc_heading',
        '_gnss_fix_ok',
        '_diff_soln',
        '_rel_pos_valid',
        '_carr_soln',
        '_is_moving',
        '_ref_pos_miss',
        '_ref_obs_miss',
        '_rel_pos_heading_valid',
        '_rel_pos_normalized',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'version': 'uint8',
        'ref_station_id': 'uint16',
        'itow': 'uint32',
        'rel_pos_n': 'int32',
        'rel_pos_e': 'int32',
        'rel_pos_d': 'int32',
        'rel_pos_length': 'int32',
        'rel_pos_heading': 'int32',
        'rel_pos_hp_n': 'int8',
        'rel_pos_hp_e': 'int8',
        'rel_pos_hp_d': 'int8',
        'rel_pos_hp_length': 'int8',
        'acc_n': 'uint32',
        'acc_e': 'uint32',
        'acc_d': 'uint32',
        'acc_length': 'uint32',
        'acc_heading': 'uint32',
        'gnss_fix_ok': 'boolean',
        'diff_soln': 'boolean',
        'rel_pos_valid': 'boolean',
        'carr_soln': 'ublox_ubx_msgs/CarrSoln',
        'is_moving': 'boolean',
        'ref_pos_miss': 'boolean',
        'ref_obs_miss': 'boolean',
        'rel_pos_heading_valid': 'boolean',
        'rel_pos_normalized': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'CarrSoln'),  # noqa: E501
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
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.version = kwargs.get('version', int())
        self.ref_station_id = kwargs.get('ref_station_id', int())
        self.itow = kwargs.get('itow', int())
        self.rel_pos_n = kwargs.get('rel_pos_n', int())
        self.rel_pos_e = kwargs.get('rel_pos_e', int())
        self.rel_pos_d = kwargs.get('rel_pos_d', int())
        self.rel_pos_length = kwargs.get('rel_pos_length', int())
        self.rel_pos_heading = kwargs.get('rel_pos_heading', int())
        self.rel_pos_hp_n = kwargs.get('rel_pos_hp_n', int())
        self.rel_pos_hp_e = kwargs.get('rel_pos_hp_e', int())
        self.rel_pos_hp_d = kwargs.get('rel_pos_hp_d', int())
        self.rel_pos_hp_length = kwargs.get('rel_pos_hp_length', int())
        self.acc_n = kwargs.get('acc_n', int())
        self.acc_e = kwargs.get('acc_e', int())
        self.acc_d = kwargs.get('acc_d', int())
        self.acc_length = kwargs.get('acc_length', int())
        self.acc_heading = kwargs.get('acc_heading', int())
        self.gnss_fix_ok = kwargs.get('gnss_fix_ok', bool())
        self.diff_soln = kwargs.get('diff_soln', bool())
        self.rel_pos_valid = kwargs.get('rel_pos_valid', bool())
        from ublox_ubx_msgs.msg import CarrSoln
        self.carr_soln = kwargs.get('carr_soln', CarrSoln())
        self.is_moving = kwargs.get('is_moving', bool())
        self.ref_pos_miss = kwargs.get('ref_pos_miss', bool())
        self.ref_obs_miss = kwargs.get('ref_obs_miss', bool())
        self.rel_pos_heading_valid = kwargs.get('rel_pos_heading_valid', bool())
        self.rel_pos_normalized = kwargs.get('rel_pos_normalized', bool())

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
        if self.ref_station_id != other.ref_station_id:
            return False
        if self.itow != other.itow:
            return False
        if self.rel_pos_n != other.rel_pos_n:
            return False
        if self.rel_pos_e != other.rel_pos_e:
            return False
        if self.rel_pos_d != other.rel_pos_d:
            return False
        if self.rel_pos_length != other.rel_pos_length:
            return False
        if self.rel_pos_heading != other.rel_pos_heading:
            return False
        if self.rel_pos_hp_n != other.rel_pos_hp_n:
            return False
        if self.rel_pos_hp_e != other.rel_pos_hp_e:
            return False
        if self.rel_pos_hp_d != other.rel_pos_hp_d:
            return False
        if self.rel_pos_hp_length != other.rel_pos_hp_length:
            return False
        if self.acc_n != other.acc_n:
            return False
        if self.acc_e != other.acc_e:
            return False
        if self.acc_d != other.acc_d:
            return False
        if self.acc_length != other.acc_length:
            return False
        if self.acc_heading != other.acc_heading:
            return False
        if self.gnss_fix_ok != other.gnss_fix_ok:
            return False
        if self.diff_soln != other.diff_soln:
            return False
        if self.rel_pos_valid != other.rel_pos_valid:
            return False
        if self.carr_soln != other.carr_soln:
            return False
        if self.is_moving != other.is_moving:
            return False
        if self.ref_pos_miss != other.ref_pos_miss:
            return False
        if self.ref_obs_miss != other.ref_obs_miss:
            return False
        if self.rel_pos_heading_valid != other.rel_pos_heading_valid:
            return False
        if self.rel_pos_normalized != other.rel_pos_normalized:
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
    def ref_station_id(self):
        """Message field 'ref_station_id'."""
        return self._ref_station_id

    @ref_station_id.setter
    def ref_station_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ref_station_id' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'ref_station_id' field must be an unsigned integer in [0, 65535]"
        self._ref_station_id = value

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
    def rel_pos_n(self):
        """Message field 'rel_pos_n'."""
        return self._rel_pos_n

    @rel_pos_n.setter
    def rel_pos_n(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rel_pos_n' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rel_pos_n' field must be an integer in [-2147483648, 2147483647]"
        self._rel_pos_n = value

    @builtins.property
    def rel_pos_e(self):
        """Message field 'rel_pos_e'."""
        return self._rel_pos_e

    @rel_pos_e.setter
    def rel_pos_e(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rel_pos_e' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rel_pos_e' field must be an integer in [-2147483648, 2147483647]"
        self._rel_pos_e = value

    @builtins.property
    def rel_pos_d(self):
        """Message field 'rel_pos_d'."""
        return self._rel_pos_d

    @rel_pos_d.setter
    def rel_pos_d(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rel_pos_d' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rel_pos_d' field must be an integer in [-2147483648, 2147483647]"
        self._rel_pos_d = value

    @builtins.property
    def rel_pos_length(self):
        """Message field 'rel_pos_length'."""
        return self._rel_pos_length

    @rel_pos_length.setter
    def rel_pos_length(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rel_pos_length' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rel_pos_length' field must be an integer in [-2147483648, 2147483647]"
        self._rel_pos_length = value

    @builtins.property
    def rel_pos_heading(self):
        """Message field 'rel_pos_heading'."""
        return self._rel_pos_heading

    @rel_pos_heading.setter
    def rel_pos_heading(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rel_pos_heading' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'rel_pos_heading' field must be an integer in [-2147483648, 2147483647]"
        self._rel_pos_heading = value

    @builtins.property
    def rel_pos_hp_n(self):
        """Message field 'rel_pos_hp_n'."""
        return self._rel_pos_hp_n

    @rel_pos_hp_n.setter
    def rel_pos_hp_n(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rel_pos_hp_n' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'rel_pos_hp_n' field must be an integer in [-128, 127]"
        self._rel_pos_hp_n = value

    @builtins.property
    def rel_pos_hp_e(self):
        """Message field 'rel_pos_hp_e'."""
        return self._rel_pos_hp_e

    @rel_pos_hp_e.setter
    def rel_pos_hp_e(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rel_pos_hp_e' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'rel_pos_hp_e' field must be an integer in [-128, 127]"
        self._rel_pos_hp_e = value

    @builtins.property
    def rel_pos_hp_d(self):
        """Message field 'rel_pos_hp_d'."""
        return self._rel_pos_hp_d

    @rel_pos_hp_d.setter
    def rel_pos_hp_d(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rel_pos_hp_d' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'rel_pos_hp_d' field must be an integer in [-128, 127]"
        self._rel_pos_hp_d = value

    @builtins.property
    def rel_pos_hp_length(self):
        """Message field 'rel_pos_hp_length'."""
        return self._rel_pos_hp_length

    @rel_pos_hp_length.setter
    def rel_pos_hp_length(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rel_pos_hp_length' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'rel_pos_hp_length' field must be an integer in [-128, 127]"
        self._rel_pos_hp_length = value

    @builtins.property
    def acc_n(self):
        """Message field 'acc_n'."""
        return self._acc_n

    @acc_n.setter
    def acc_n(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'acc_n' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'acc_n' field must be an unsigned integer in [0, 4294967295]"
        self._acc_n = value

    @builtins.property
    def acc_e(self):
        """Message field 'acc_e'."""
        return self._acc_e

    @acc_e.setter
    def acc_e(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'acc_e' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'acc_e' field must be an unsigned integer in [0, 4294967295]"
        self._acc_e = value

    @builtins.property
    def acc_d(self):
        """Message field 'acc_d'."""
        return self._acc_d

    @acc_d.setter
    def acc_d(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'acc_d' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'acc_d' field must be an unsigned integer in [0, 4294967295]"
        self._acc_d = value

    @builtins.property
    def acc_length(self):
        """Message field 'acc_length'."""
        return self._acc_length

    @acc_length.setter
    def acc_length(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'acc_length' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'acc_length' field must be an unsigned integer in [0, 4294967295]"
        self._acc_length = value

    @builtins.property
    def acc_heading(self):
        """Message field 'acc_heading'."""
        return self._acc_heading

    @acc_heading.setter
    def acc_heading(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'acc_heading' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'acc_heading' field must be an unsigned integer in [0, 4294967295]"
        self._acc_heading = value

    @builtins.property
    def gnss_fix_ok(self):
        """Message field 'gnss_fix_ok'."""
        return self._gnss_fix_ok

    @gnss_fix_ok.setter
    def gnss_fix_ok(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'gnss_fix_ok' field must be of type 'bool'"
        self._gnss_fix_ok = value

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
    def rel_pos_valid(self):
        """Message field 'rel_pos_valid'."""
        return self._rel_pos_valid

    @rel_pos_valid.setter
    def rel_pos_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'rel_pos_valid' field must be of type 'bool'"
        self._rel_pos_valid = value

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
    def is_moving(self):
        """Message field 'is_moving'."""
        return self._is_moving

    @is_moving.setter
    def is_moving(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'is_moving' field must be of type 'bool'"
        self._is_moving = value

    @builtins.property
    def ref_pos_miss(self):
        """Message field 'ref_pos_miss'."""
        return self._ref_pos_miss

    @ref_pos_miss.setter
    def ref_pos_miss(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ref_pos_miss' field must be of type 'bool'"
        self._ref_pos_miss = value

    @builtins.property
    def ref_obs_miss(self):
        """Message field 'ref_obs_miss'."""
        return self._ref_obs_miss

    @ref_obs_miss.setter
    def ref_obs_miss(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ref_obs_miss' field must be of type 'bool'"
        self._ref_obs_miss = value

    @builtins.property
    def rel_pos_heading_valid(self):
        """Message field 'rel_pos_heading_valid'."""
        return self._rel_pos_heading_valid

    @rel_pos_heading_valid.setter
    def rel_pos_heading_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'rel_pos_heading_valid' field must be of type 'bool'"
        self._rel_pos_heading_valid = value

    @builtins.property
    def rel_pos_normalized(self):
        """Message field 'rel_pos_normalized'."""
        return self._rel_pos_normalized

    @rel_pos_normalized.setter
    def rel_pos_normalized(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'rel_pos_normalized' field must be of type 'bool'"
        self._rel_pos_normalized = value
