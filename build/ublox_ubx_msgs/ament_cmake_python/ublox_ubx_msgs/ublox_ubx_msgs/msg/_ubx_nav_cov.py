# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXNavCov.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXNavCov(type):
    """Metaclass of message 'UBXNavCov'."""

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
                'ublox_ubx_msgs.msg.UBXNavCov')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_nav_cov
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_nav_cov
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_nav_cov
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_nav_cov
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_nav_cov

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UBXNavCov(metaclass=Metaclass_UBXNavCov):
    """Message class 'UBXNavCov'."""

    __slots__ = [
        '_header',
        '_itow',
        '_version',
        '_pos_cor_valid',
        '_vel_cor_valid',
        '_pos_cov_nn',
        '_pos_cov_ne',
        '_pos_cov_nd',
        '_pos_cov_ee',
        '_pos_cov_ed',
        '_pos_cov_dd',
        '_vel_cov_nn',
        '_vel_cov_ne',
        '_vel_cov_nd',
        '_vel_cov_ee',
        '_vel_cov_ed',
        '_vel_cov_dd',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'itow': 'uint32',
        'version': 'uint8',
        'pos_cor_valid': 'boolean',
        'vel_cor_valid': 'boolean',
        'pos_cov_nn': 'float',
        'pos_cov_ne': 'float',
        'pos_cov_nd': 'float',
        'pos_cov_ee': 'float',
        'pos_cov_ed': 'float',
        'pos_cov_dd': 'float',
        'vel_cov_nn': 'float',
        'vel_cov_ne': 'float',
        'vel_cov_nd': 'float',
        'vel_cov_ee': 'float',
        'vel_cov_ed': 'float',
        'vel_cov_dd': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.itow = kwargs.get('itow', int())
        self.version = kwargs.get('version', int())
        self.pos_cor_valid = kwargs.get('pos_cor_valid', bool())
        self.vel_cor_valid = kwargs.get('vel_cor_valid', bool())
        self.pos_cov_nn = kwargs.get('pos_cov_nn', float())
        self.pos_cov_ne = kwargs.get('pos_cov_ne', float())
        self.pos_cov_nd = kwargs.get('pos_cov_nd', float())
        self.pos_cov_ee = kwargs.get('pos_cov_ee', float())
        self.pos_cov_ed = kwargs.get('pos_cov_ed', float())
        self.pos_cov_dd = kwargs.get('pos_cov_dd', float())
        self.vel_cov_nn = kwargs.get('vel_cov_nn', float())
        self.vel_cov_ne = kwargs.get('vel_cov_ne', float())
        self.vel_cov_nd = kwargs.get('vel_cov_nd', float())
        self.vel_cov_ee = kwargs.get('vel_cov_ee', float())
        self.vel_cov_ed = kwargs.get('vel_cov_ed', float())
        self.vel_cov_dd = kwargs.get('vel_cov_dd', float())

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
        if self.pos_cor_valid != other.pos_cor_valid:
            return False
        if self.vel_cor_valid != other.vel_cor_valid:
            return False
        if self.pos_cov_nn != other.pos_cov_nn:
            return False
        if self.pos_cov_ne != other.pos_cov_ne:
            return False
        if self.pos_cov_nd != other.pos_cov_nd:
            return False
        if self.pos_cov_ee != other.pos_cov_ee:
            return False
        if self.pos_cov_ed != other.pos_cov_ed:
            return False
        if self.pos_cov_dd != other.pos_cov_dd:
            return False
        if self.vel_cov_nn != other.vel_cov_nn:
            return False
        if self.vel_cov_ne != other.vel_cov_ne:
            return False
        if self.vel_cov_nd != other.vel_cov_nd:
            return False
        if self.vel_cov_ee != other.vel_cov_ee:
            return False
        if self.vel_cov_ed != other.vel_cov_ed:
            return False
        if self.vel_cov_dd != other.vel_cov_dd:
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
    def pos_cor_valid(self):
        """Message field 'pos_cor_valid'."""
        return self._pos_cor_valid

    @pos_cor_valid.setter
    def pos_cor_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'pos_cor_valid' field must be of type 'bool'"
        self._pos_cor_valid = value

    @builtins.property
    def vel_cor_valid(self):
        """Message field 'vel_cor_valid'."""
        return self._vel_cor_valid

    @vel_cor_valid.setter
    def vel_cor_valid(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'vel_cor_valid' field must be of type 'bool'"
        self._vel_cor_valid = value

    @builtins.property
    def pos_cov_nn(self):
        """Message field 'pos_cov_nn'."""
        return self._pos_cov_nn

    @pos_cov_nn.setter
    def pos_cov_nn(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pos_cov_nn' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pos_cov_nn' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pos_cov_nn = value

    @builtins.property
    def pos_cov_ne(self):
        """Message field 'pos_cov_ne'."""
        return self._pos_cov_ne

    @pos_cov_ne.setter
    def pos_cov_ne(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pos_cov_ne' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pos_cov_ne' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pos_cov_ne = value

    @builtins.property
    def pos_cov_nd(self):
        """Message field 'pos_cov_nd'."""
        return self._pos_cov_nd

    @pos_cov_nd.setter
    def pos_cov_nd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pos_cov_nd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pos_cov_nd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pos_cov_nd = value

    @builtins.property
    def pos_cov_ee(self):
        """Message field 'pos_cov_ee'."""
        return self._pos_cov_ee

    @pos_cov_ee.setter
    def pos_cov_ee(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pos_cov_ee' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pos_cov_ee' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pos_cov_ee = value

    @builtins.property
    def pos_cov_ed(self):
        """Message field 'pos_cov_ed'."""
        return self._pos_cov_ed

    @pos_cov_ed.setter
    def pos_cov_ed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pos_cov_ed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pos_cov_ed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pos_cov_ed = value

    @builtins.property
    def pos_cov_dd(self):
        """Message field 'pos_cov_dd'."""
        return self._pos_cov_dd

    @pos_cov_dd.setter
    def pos_cov_dd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'pos_cov_dd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'pos_cov_dd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._pos_cov_dd = value

    @builtins.property
    def vel_cov_nn(self):
        """Message field 'vel_cov_nn'."""
        return self._vel_cov_nn

    @vel_cov_nn.setter
    def vel_cov_nn(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel_cov_nn' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vel_cov_nn' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vel_cov_nn = value

    @builtins.property
    def vel_cov_ne(self):
        """Message field 'vel_cov_ne'."""
        return self._vel_cov_ne

    @vel_cov_ne.setter
    def vel_cov_ne(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel_cov_ne' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vel_cov_ne' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vel_cov_ne = value

    @builtins.property
    def vel_cov_nd(self):
        """Message field 'vel_cov_nd'."""
        return self._vel_cov_nd

    @vel_cov_nd.setter
    def vel_cov_nd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel_cov_nd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vel_cov_nd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vel_cov_nd = value

    @builtins.property
    def vel_cov_ee(self):
        """Message field 'vel_cov_ee'."""
        return self._vel_cov_ee

    @vel_cov_ee.setter
    def vel_cov_ee(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel_cov_ee' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vel_cov_ee' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vel_cov_ee = value

    @builtins.property
    def vel_cov_ed(self):
        """Message field 'vel_cov_ed'."""
        return self._vel_cov_ed

    @vel_cov_ed.setter
    def vel_cov_ed(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel_cov_ed' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vel_cov_ed' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vel_cov_ed = value

    @builtins.property
    def vel_cov_dd(self):
        """Message field 'vel_cov_dd'."""
        return self._vel_cov_dd

    @vel_cov_dd.setter
    def vel_cov_dd(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'vel_cov_dd' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'vel_cov_dd' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._vel_cov_dd = value
