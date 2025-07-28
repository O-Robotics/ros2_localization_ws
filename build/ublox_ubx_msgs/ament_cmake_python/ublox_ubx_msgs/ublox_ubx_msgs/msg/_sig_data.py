# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/SigData.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SigData(type):
    """Metaclass of message 'SigData'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'QUALITY_NO_SIGNAL': 0,
        'QUALITY_SEARCHING_SIGNAL': 1,
        'QUALITY_SIGNAL_ACQUIRED': 2,
        'QUALITY_SIGNAL_UNUSABLE': 3,
        'QUALITY_CODE_LOCKED': 4,
        'QUALITY_CODE_CARRIER_LOCKED': 5,
        'CORR_NONE': 0,
        'CORR_SBAS': 1,
        'CORR_BEIDOU': 2,
        'CORR_RTCM2': 3,
        'CORR_RTCM3_OSR': 4,
        'CORR_RTCM3_SSR': 5,
        'CORR_QZSS_SLAS': 6,
        'CORR_SPARTN': 7,
        'CORR_CLAS': 8,
        'IONO_NONE': 0,
        'IONO_KLOB_GPS': 1,
        'IONO_SBAS': 2,
        'IONO_KLOB_BEIDOU': 3,
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
                'ublox_ubx_msgs.msg.SigData')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sig_data
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sig_data
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sig_data
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sig_data
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sig_data

            from ublox_ubx_msgs.msg import SigFlags
            if SigFlags.__class__._TYPE_SUPPORT is None:
                SigFlags.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'QUALITY_NO_SIGNAL': cls.__constants['QUALITY_NO_SIGNAL'],
            'QUALITY_SEARCHING_SIGNAL': cls.__constants['QUALITY_SEARCHING_SIGNAL'],
            'QUALITY_SIGNAL_ACQUIRED': cls.__constants['QUALITY_SIGNAL_ACQUIRED'],
            'QUALITY_SIGNAL_UNUSABLE': cls.__constants['QUALITY_SIGNAL_UNUSABLE'],
            'QUALITY_CODE_LOCKED': cls.__constants['QUALITY_CODE_LOCKED'],
            'QUALITY_CODE_CARRIER_LOCKED': cls.__constants['QUALITY_CODE_CARRIER_LOCKED'],
            'CORR_NONE': cls.__constants['CORR_NONE'],
            'CORR_SBAS': cls.__constants['CORR_SBAS'],
            'CORR_BEIDOU': cls.__constants['CORR_BEIDOU'],
            'CORR_RTCM2': cls.__constants['CORR_RTCM2'],
            'CORR_RTCM3_OSR': cls.__constants['CORR_RTCM3_OSR'],
            'CORR_RTCM3_SSR': cls.__constants['CORR_RTCM3_SSR'],
            'CORR_QZSS_SLAS': cls.__constants['CORR_QZSS_SLAS'],
            'CORR_SPARTN': cls.__constants['CORR_SPARTN'],
            'CORR_CLAS': cls.__constants['CORR_CLAS'],
            'IONO_NONE': cls.__constants['IONO_NONE'],
            'IONO_KLOB_GPS': cls.__constants['IONO_KLOB_GPS'],
            'IONO_SBAS': cls.__constants['IONO_SBAS'],
            'IONO_KLOB_BEIDOU': cls.__constants['IONO_KLOB_BEIDOU'],
        }

    @property
    def QUALITY_NO_SIGNAL(self):
        """Message constant 'QUALITY_NO_SIGNAL'."""
        return Metaclass_SigData.__constants['QUALITY_NO_SIGNAL']

    @property
    def QUALITY_SEARCHING_SIGNAL(self):
        """Message constant 'QUALITY_SEARCHING_SIGNAL'."""
        return Metaclass_SigData.__constants['QUALITY_SEARCHING_SIGNAL']

    @property
    def QUALITY_SIGNAL_ACQUIRED(self):
        """Message constant 'QUALITY_SIGNAL_ACQUIRED'."""
        return Metaclass_SigData.__constants['QUALITY_SIGNAL_ACQUIRED']

    @property
    def QUALITY_SIGNAL_UNUSABLE(self):
        """Message constant 'QUALITY_SIGNAL_UNUSABLE'."""
        return Metaclass_SigData.__constants['QUALITY_SIGNAL_UNUSABLE']

    @property
    def QUALITY_CODE_LOCKED(self):
        """Message constant 'QUALITY_CODE_LOCKED'."""
        return Metaclass_SigData.__constants['QUALITY_CODE_LOCKED']

    @property
    def QUALITY_CODE_CARRIER_LOCKED(self):
        """Message constant 'QUALITY_CODE_CARRIER_LOCKED'."""
        return Metaclass_SigData.__constants['QUALITY_CODE_CARRIER_LOCKED']

    @property
    def CORR_NONE(self):
        """Message constant 'CORR_NONE'."""
        return Metaclass_SigData.__constants['CORR_NONE']

    @property
    def CORR_SBAS(self):
        """Message constant 'CORR_SBAS'."""
        return Metaclass_SigData.__constants['CORR_SBAS']

    @property
    def CORR_BEIDOU(self):
        """Message constant 'CORR_BEIDOU'."""
        return Metaclass_SigData.__constants['CORR_BEIDOU']

    @property
    def CORR_RTCM2(self):
        """Message constant 'CORR_RTCM2'."""
        return Metaclass_SigData.__constants['CORR_RTCM2']

    @property
    def CORR_RTCM3_OSR(self):
        """Message constant 'CORR_RTCM3_OSR'."""
        return Metaclass_SigData.__constants['CORR_RTCM3_OSR']

    @property
    def CORR_RTCM3_SSR(self):
        """Message constant 'CORR_RTCM3_SSR'."""
        return Metaclass_SigData.__constants['CORR_RTCM3_SSR']

    @property
    def CORR_QZSS_SLAS(self):
        """Message constant 'CORR_QZSS_SLAS'."""
        return Metaclass_SigData.__constants['CORR_QZSS_SLAS']

    @property
    def CORR_SPARTN(self):
        """Message constant 'CORR_SPARTN'."""
        return Metaclass_SigData.__constants['CORR_SPARTN']

    @property
    def CORR_CLAS(self):
        """Message constant 'CORR_CLAS'."""
        return Metaclass_SigData.__constants['CORR_CLAS']

    @property
    def IONO_NONE(self):
        """Message constant 'IONO_NONE'."""
        return Metaclass_SigData.__constants['IONO_NONE']

    @property
    def IONO_KLOB_GPS(self):
        """Message constant 'IONO_KLOB_GPS'."""
        return Metaclass_SigData.__constants['IONO_KLOB_GPS']

    @property
    def IONO_SBAS(self):
        """Message constant 'IONO_SBAS'."""
        return Metaclass_SigData.__constants['IONO_SBAS']

    @property
    def IONO_KLOB_BEIDOU(self):
        """Message constant 'IONO_KLOB_BEIDOU'."""
        return Metaclass_SigData.__constants['IONO_KLOB_BEIDOU']


class SigData(metaclass=Metaclass_SigData):
    """
    Message class 'SigData'.

    Constants:
      QUALITY_NO_SIGNAL
      QUALITY_SEARCHING_SIGNAL
      QUALITY_SIGNAL_ACQUIRED
      QUALITY_SIGNAL_UNUSABLE
      QUALITY_CODE_LOCKED
      QUALITY_CODE_CARRIER_LOCKED
      CORR_NONE
      CORR_SBAS
      CORR_BEIDOU
      CORR_RTCM2
      CORR_RTCM3_OSR
      CORR_RTCM3_SSR
      CORR_QZSS_SLAS
      CORR_SPARTN
      CORR_CLAS
      IONO_NONE
      IONO_KLOB_GPS
      IONO_SBAS
      IONO_KLOB_BEIDOU
    """

    __slots__ = [
        '_gnss_id',
        '_sv_id',
        '_sig_id',
        '_freq_id',
        '_pr_res',
        '_cno',
        '_quality_ind',
        '_corr_source',
        '_iono_model',
        '_sig_flags',
    ]

    _fields_and_field_types = {
        'gnss_id': 'uint8',
        'sv_id': 'uint8',
        'sig_id': 'uint8',
        'freq_id': 'uint8',
        'pr_res': 'int16',
        'cno': 'uint8',
        'quality_ind': 'uint8',
        'corr_source': 'uint8',
        'iono_model': 'uint8',
        'sig_flags': 'ublox_ubx_msgs/SigFlags',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('int16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'SigFlags'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.gnss_id = kwargs.get('gnss_id', int())
        self.sv_id = kwargs.get('sv_id', int())
        self.sig_id = kwargs.get('sig_id', int())
        self.freq_id = kwargs.get('freq_id', int())
        self.pr_res = kwargs.get('pr_res', int())
        self.cno = kwargs.get('cno', int())
        self.quality_ind = kwargs.get('quality_ind', int())
        self.corr_source = kwargs.get('corr_source', int())
        self.iono_model = kwargs.get('iono_model', int())
        from ublox_ubx_msgs.msg import SigFlags
        self.sig_flags = kwargs.get('sig_flags', SigFlags())

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
        if self.sig_id != other.sig_id:
            return False
        if self.freq_id != other.freq_id:
            return False
        if self.pr_res != other.pr_res:
            return False
        if self.cno != other.cno:
            return False
        if self.quality_ind != other.quality_ind:
            return False
        if self.corr_source != other.corr_source:
            return False
        if self.iono_model != other.iono_model:
            return False
        if self.sig_flags != other.sig_flags:
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
    def sig_id(self):
        """Message field 'sig_id'."""
        return self._sig_id

    @sig_id.setter
    def sig_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sig_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sig_id' field must be an unsigned integer in [0, 255]"
        self._sig_id = value

    @builtins.property
    def freq_id(self):
        """Message field 'freq_id'."""
        return self._freq_id

    @freq_id.setter
    def freq_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'freq_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'freq_id' field must be an unsigned integer in [0, 255]"
        self._freq_id = value

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
    def corr_source(self):
        """Message field 'corr_source'."""
        return self._corr_source

    @corr_source.setter
    def corr_source(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'corr_source' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'corr_source' field must be an unsigned integer in [0, 255]"
        self._corr_source = value

    @builtins.property
    def iono_model(self):
        """Message field 'iono_model'."""
        return self._iono_model

    @iono_model.setter
    def iono_model(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'iono_model' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'iono_model' field must be an unsigned integer in [0, 255]"
        self._iono_model = value

    @builtins.property
    def sig_flags(self):
        """Message field 'sig_flags'."""
        return self._sig_flags

    @sig_flags.setter
    def sig_flags(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import SigFlags
            assert \
                isinstance(value, SigFlags), \
                "The 'sig_flags' field must be a sub message of type 'SigFlags'"
        self._sig_flags = value
