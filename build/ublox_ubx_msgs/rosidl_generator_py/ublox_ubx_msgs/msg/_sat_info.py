# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/SatInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SatInfo(type):
    """Metaclass of message 'SatInfo'."""

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
                'ublox_ubx_msgs.msg.SatInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sat_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sat_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sat_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sat_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sat_info

            from ublox_ubx_msgs.msg import SatFlags
            if SatFlags.__class__._TYPE_SUPPORT is None:
                SatFlags.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SatInfo(metaclass=Metaclass_SatInfo):
    """Message class 'SatInfo'."""

    __slots__ = [
        '_gnss_id',
        '_sv_id',
        '_cno',
        '_elev',
        '_azim',
        '_pr_res',
        '_flags',
    ]

    _fields_and_field_types = {
        'gnss_id': 'uint8',
        'sv_id': 'uint8',
        'cno': 'uint8',
        'elev': 'int8',
        'azim': 'int16',
        'pr_res': 'int16',
        'flags': 'ublox_ubx_msgs/SatFlags',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'SatFlags'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.gnss_id = kwargs.get('gnss_id', int())
        self.sv_id = kwargs.get('sv_id', int())
        self.cno = kwargs.get('cno', int())
        self.elev = kwargs.get('elev', int())
        self.azim = kwargs.get('azim', int())
        self.pr_res = kwargs.get('pr_res', int())
        from ublox_ubx_msgs.msg import SatFlags
        self.flags = kwargs.get('flags', SatFlags())

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
        if self.cno != other.cno:
            return False
        if self.elev != other.elev:
            return False
        if self.azim != other.azim:
            return False
        if self.pr_res != other.pr_res:
            return False
        if self.flags != other.flags:
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
    def cno(self):
        """Message field 'cno'."""
        return self._cno

    @cno.setter
    def cno(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'cno' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'cno' field must be an unsigned integer in [0, 255]"
        self._cno = value

    @builtins.property
    def elev(self):
        """Message field 'elev'."""
        return self._elev

    @elev.setter
    def elev(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'elev' field must be of type 'int'"
            assert value >= -128 and value < 128, \
                "The 'elev' field must be an integer in [-128, 127]"
        self._elev = value

    @builtins.property
    def azim(self):
        """Message field 'azim'."""
        return self._azim

    @azim.setter
    def azim(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'azim' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'azim' field must be an integer in [-32768, 32767]"
        self._azim = value

    @builtins.property
    def pr_res(self):
        """Message field 'pr_res'."""
        return self._pr_res

    @pr_res.setter
    def pr_res(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pr_res' field must be of type 'int'"
            assert value >= -32768 and value < 32768, \
                "The 'pr_res' field must be an integer in [-32768, 32767]"
        self._pr_res = value

    @builtins.property
    def flags(self):
        """Message field 'flags'."""
        return self._flags

    @flags.setter
    def flags(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import SatFlags
            assert \
                isinstance(value, SatFlags), \
                "The 'flags' field must be a sub message of type 'SatFlags'"
        self._flags = value
