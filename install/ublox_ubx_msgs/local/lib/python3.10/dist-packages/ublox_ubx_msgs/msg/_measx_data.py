# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/MeasxData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_MeasxData(type):
    """Metaclass of message 'MeasxData'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'MPATH_NOT_MEASURED': 0,
        'MPATH_LOW': 1,
        'MPATH_MEDIUM': 2,
        'MPATH_HIGH': 3,
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
                'ublox_ubx_msgs.msg.MeasxData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__measx_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__measx_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__measx_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__measx_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__measx_data

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'MPATH_NOT_MEASURED': cls.__constants['MPATH_NOT_MEASURED'],
            'MPATH_LOW': cls.__constants['MPATH_LOW'],
            'MPATH_MEDIUM': cls.__constants['MPATH_MEDIUM'],
            'MPATH_HIGH': cls.__constants['MPATH_HIGH'],
        }

    @property
    def MPATH_NOT_MEASURED(self):
        """Message constant 'MPATH_NOT_MEASURED'."""
        return Metaclass_MeasxData.__constants['MPATH_NOT_MEASURED']

    @property
    def MPATH_LOW(self):
        """Message constant 'MPATH_LOW'."""
        return Metaclass_MeasxData.__constants['MPATH_LOW']

    @property
    def MPATH_MEDIUM(self):
        """Message constant 'MPATH_MEDIUM'."""
        return Metaclass_MeasxData.__constants['MPATH_MEDIUM']

    @property
    def MPATH_HIGH(self):
        """Message constant 'MPATH_HIGH'."""
        return Metaclass_MeasxData.__constants['MPATH_HIGH']


class MeasxData(metaclass=Metaclass_MeasxData):
    """
    Message class 'MeasxData'.

    Constants:
      MPATH_NOT_MEASURED
      MPATH_LOW
      MPATH_MEDIUM
      MPATH_HIGH
    """

    __slots__ = [
        '_gnss_id',
        '_sv_id',
        '_c_no',
        '_mpath_indic',
        '_doppler_ms',
        '_doppler_hz',
        '_whole_chips',
        '_frac_chips',
        '_code_phase',
        '_int_code_phase',
        '_pseu_range_rms_err',
    ]

    _fields_and_field_types = {
        'gnss_id': 'uint8',
        'sv_id': 'uint8',
        'c_no': 'uint8',
        'mpath_indic': 'uint8',
        'doppler_ms': 'int32',
        'doppler_hz': 'int32',
        'whole_chips': 'uint16',
        'frac_chips': 'uint16',
        'code_phase': 'uint32',
        'int_code_phase': 'uint8',
        'pseu_range_rms_err': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.gnss_id = kwargs.get('gnss_id', int())
        self.sv_id = kwargs.get('sv_id', int())
        self.c_no = kwargs.get('c_no', int())
        self.mpath_indic = kwargs.get('mpath_indic', int())
        self.doppler_ms = kwargs.get('doppler_ms', int())
        self.doppler_hz = kwargs.get('doppler_hz', int())
        self.whole_chips = kwargs.get('whole_chips', int())
        self.frac_chips = kwargs.get('frac_chips', int())
        self.code_phase = kwargs.get('code_phase', int())
        self.int_code_phase = kwargs.get('int_code_phase', int())
        self.pseu_range_rms_err = kwargs.get('pseu_range_rms_err', int())

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
        if self.c_no != other.c_no:
            return False
        if self.mpath_indic != other.mpath_indic:
            return False
        if self.doppler_ms != other.doppler_ms:
            return False
        if self.doppler_hz != other.doppler_hz:
            return False
        if self.whole_chips != other.whole_chips:
            return False
        if self.frac_chips != other.frac_chips:
            return False
        if self.code_phase != other.code_phase:
            return False
        if self.int_code_phase != other.int_code_phase:
            return False
        if self.pseu_range_rms_err != other.pseu_range_rms_err:
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
    def c_no(self):
        """Message field 'c_no'."""
        return self._c_no

    @c_no.setter
    def c_no(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'c_no' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'c_no' field must be an unsigned integer in [0, 255]"
        self._c_no = value

    @builtins.property
    def mpath_indic(self):
        """Message field 'mpath_indic'."""
        return self._mpath_indic

    @mpath_indic.setter
    def mpath_indic(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mpath_indic' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mpath_indic' field must be an unsigned integer in [0, 255]"
        self._mpath_indic = value

    @builtins.property
    def doppler_ms(self):
        """Message field 'doppler_ms'."""
        return self._doppler_ms

    @doppler_ms.setter
    def doppler_ms(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'doppler_ms' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'doppler_ms' field must be an integer in [-2147483648, 2147483647]"
        self._doppler_ms = value

    @builtins.property
    def doppler_hz(self):
        """Message field 'doppler_hz'."""
        return self._doppler_hz

    @doppler_hz.setter
    def doppler_hz(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'doppler_hz' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'doppler_hz' field must be an integer in [-2147483648, 2147483647]"
        self._doppler_hz = value

    @builtins.property
    def whole_chips(self):
        """Message field 'whole_chips'."""
        return self._whole_chips

    @whole_chips.setter
    def whole_chips(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'whole_chips' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'whole_chips' field must be an unsigned integer in [0, 65535]"
        self._whole_chips = value

    @builtins.property
    def frac_chips(self):
        """Message field 'frac_chips'."""
        return self._frac_chips

    @frac_chips.setter
    def frac_chips(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'frac_chips' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'frac_chips' field must be an unsigned integer in [0, 65535]"
        self._frac_chips = value

    @builtins.property
    def code_phase(self):
        """Message field 'code_phase'."""
        return self._code_phase

    @code_phase.setter
    def code_phase(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'code_phase' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'code_phase' field must be an unsigned integer in [0, 4294967295]"
        self._code_phase = value

    @builtins.property
    def int_code_phase(self):
        """Message field 'int_code_phase'."""
        return self._int_code_phase

    @int_code_phase.setter
    def int_code_phase(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'int_code_phase' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'int_code_phase' field must be an unsigned integer in [0, 255]"
        self._int_code_phase = value

    @builtins.property
    def pseu_range_rms_err(self):
        """Message field 'pseu_range_rms_err'."""
        return self._pseu_range_rms_err

    @pseu_range_rms_err.setter
    def pseu_range_rms_err(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pseu_range_rms_err' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'pseu_range_rms_err' field must be an unsigned integer in [0, 255]"
        self._pseu_range_rms_err = value
