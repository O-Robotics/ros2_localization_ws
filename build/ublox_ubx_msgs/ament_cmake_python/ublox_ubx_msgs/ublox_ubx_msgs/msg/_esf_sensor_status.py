# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/ESFSensorStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ESFSensorStatus(type):
    """Metaclass of message 'ESFSensorStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'CALIB_STATUS_NOT_CALIBRATED': 0,
        'CALIB_STATUS_CALIBRATING': 1,
        'CALIB_STATUS_CALIBRATED0': 2,
        'CALIB_STATUS_CALIBRATED1': 3,
        'TIME_STATUS_NO_DATA': 0,
        'TIME_STATUS_FIRST_BYTE_USED': 1,
        'TIME_STATUS_TTAG_PROVIDED': 3,
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
                'ublox_ubx_msgs.msg.ESFSensorStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__esf_sensor_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__esf_sensor_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__esf_sensor_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__esf_sensor_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__esf_sensor_status

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'CALIB_STATUS_NOT_CALIBRATED': cls.__constants['CALIB_STATUS_NOT_CALIBRATED'],
            'CALIB_STATUS_CALIBRATING': cls.__constants['CALIB_STATUS_CALIBRATING'],
            'CALIB_STATUS_CALIBRATED0': cls.__constants['CALIB_STATUS_CALIBRATED0'],
            'CALIB_STATUS_CALIBRATED1': cls.__constants['CALIB_STATUS_CALIBRATED1'],
            'TIME_STATUS_NO_DATA': cls.__constants['TIME_STATUS_NO_DATA'],
            'TIME_STATUS_FIRST_BYTE_USED': cls.__constants['TIME_STATUS_FIRST_BYTE_USED'],
            'TIME_STATUS_TTAG_PROVIDED': cls.__constants['TIME_STATUS_TTAG_PROVIDED'],
        }

    @property
    def CALIB_STATUS_NOT_CALIBRATED(self):
        """Message constant 'CALIB_STATUS_NOT_CALIBRATED'."""
        return Metaclass_ESFSensorStatus.__constants['CALIB_STATUS_NOT_CALIBRATED']

    @property
    def CALIB_STATUS_CALIBRATING(self):
        """Message constant 'CALIB_STATUS_CALIBRATING'."""
        return Metaclass_ESFSensorStatus.__constants['CALIB_STATUS_CALIBRATING']

    @property
    def CALIB_STATUS_CALIBRATED0(self):
        """Message constant 'CALIB_STATUS_CALIBRATED0'."""
        return Metaclass_ESFSensorStatus.__constants['CALIB_STATUS_CALIBRATED0']

    @property
    def CALIB_STATUS_CALIBRATED1(self):
        """Message constant 'CALIB_STATUS_CALIBRATED1'."""
        return Metaclass_ESFSensorStatus.__constants['CALIB_STATUS_CALIBRATED1']

    @property
    def TIME_STATUS_NO_DATA(self):
        """Message constant 'TIME_STATUS_NO_DATA'."""
        return Metaclass_ESFSensorStatus.__constants['TIME_STATUS_NO_DATA']

    @property
    def TIME_STATUS_FIRST_BYTE_USED(self):
        """Message constant 'TIME_STATUS_FIRST_BYTE_USED'."""
        return Metaclass_ESFSensorStatus.__constants['TIME_STATUS_FIRST_BYTE_USED']

    @property
    def TIME_STATUS_TTAG_PROVIDED(self):
        """Message constant 'TIME_STATUS_TTAG_PROVIDED'."""
        return Metaclass_ESFSensorStatus.__constants['TIME_STATUS_TTAG_PROVIDED']


class ESFSensorStatus(metaclass=Metaclass_ESFSensorStatus):
    """
    Message class 'ESFSensorStatus'.

    Constants:
      CALIB_STATUS_NOT_CALIBRATED
      CALIB_STATUS_CALIBRATING
      CALIB_STATUS_CALIBRATED0
      CALIB_STATUS_CALIBRATED1
      TIME_STATUS_NO_DATA
      TIME_STATUS_FIRST_BYTE_USED
      TIME_STATUS_TTAG_PROVIDED
    """

    __slots__ = [
        '_sensor_data_type',
        '_used',
        '_ready',
        '_calib_status',
        '_time_status',
        '_freq',
        '_fault_bad_meas',
        '_fault_bad_ttag',
        '_fault_missing_meas',
        '_fault_noisy_meas',
    ]

    _fields_and_field_types = {
        'sensor_data_type': 'uint8',
        'used': 'boolean',
        'ready': 'boolean',
        'calib_status': 'uint8',
        'time_status': 'uint8',
        'freq': 'uint8',
        'fault_bad_meas': 'boolean',
        'fault_bad_ttag': 'boolean',
        'fault_missing_meas': 'boolean',
        'fault_noisy_meas': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.sensor_data_type = kwargs.get('sensor_data_type', int())
        self.used = kwargs.get('used', bool())
        self.ready = kwargs.get('ready', bool())
        self.calib_status = kwargs.get('calib_status', int())
        self.time_status = kwargs.get('time_status', int())
        self.freq = kwargs.get('freq', int())
        self.fault_bad_meas = kwargs.get('fault_bad_meas', bool())
        self.fault_bad_ttag = kwargs.get('fault_bad_ttag', bool())
        self.fault_missing_meas = kwargs.get('fault_missing_meas', bool())
        self.fault_noisy_meas = kwargs.get('fault_noisy_meas', bool())

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
        if self.sensor_data_type != other.sensor_data_type:
            return False
        if self.used != other.used:
            return False
        if self.ready != other.ready:
            return False
        if self.calib_status != other.calib_status:
            return False
        if self.time_status != other.time_status:
            return False
        if self.freq != other.freq:
            return False
        if self.fault_bad_meas != other.fault_bad_meas:
            return False
        if self.fault_bad_ttag != other.fault_bad_ttag:
            return False
        if self.fault_missing_meas != other.fault_missing_meas:
            return False
        if self.fault_noisy_meas != other.fault_noisy_meas:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def sensor_data_type(self):
        """Message field 'sensor_data_type'."""
        return self._sensor_data_type

    @sensor_data_type.setter
    def sensor_data_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sensor_data_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'sensor_data_type' field must be an unsigned integer in [0, 255]"
        self._sensor_data_type = value

    @builtins.property
    def used(self):
        """Message field 'used'."""
        return self._used

    @used.setter
    def used(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'used' field must be of type 'bool'"
        self._used = value

    @builtins.property
    def ready(self):
        """Message field 'ready'."""
        return self._ready

    @ready.setter
    def ready(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ready' field must be of type 'bool'"
        self._ready = value

    @builtins.property
    def calib_status(self):
        """Message field 'calib_status'."""
        return self._calib_status

    @calib_status.setter
    def calib_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'calib_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'calib_status' field must be an unsigned integer in [0, 255]"
        self._calib_status = value

    @builtins.property
    def time_status(self):
        """Message field 'time_status'."""
        return self._time_status

    @time_status.setter
    def time_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'time_status' field must be an unsigned integer in [0, 255]"
        self._time_status = value

    @builtins.property
    def freq(self):
        """Message field 'freq'."""
        return self._freq

    @freq.setter
    def freq(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'freq' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'freq' field must be an unsigned integer in [0, 255]"
        self._freq = value

    @builtins.property
    def fault_bad_meas(self):
        """Message field 'fault_bad_meas'."""
        return self._fault_bad_meas

    @fault_bad_meas.setter
    def fault_bad_meas(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fault_bad_meas' field must be of type 'bool'"
        self._fault_bad_meas = value

    @builtins.property
    def fault_bad_ttag(self):
        """Message field 'fault_bad_ttag'."""
        return self._fault_bad_ttag

    @fault_bad_ttag.setter
    def fault_bad_ttag(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fault_bad_ttag' field must be of type 'bool'"
        self._fault_bad_ttag = value

    @builtins.property
    def fault_missing_meas(self):
        """Message field 'fault_missing_meas'."""
        return self._fault_missing_meas

    @fault_missing_meas.setter
    def fault_missing_meas(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fault_missing_meas' field must be of type 'bool'"
        self._fault_missing_meas = value

    @builtins.property
    def fault_noisy_meas(self):
        """Message field 'fault_noisy_meas'."""
        return self._fault_noisy_meas

    @fault_noisy_meas.setter
    def fault_noisy_meas(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'fault_noisy_meas' field must be of type 'bool'"
        self._fault_noisy_meas = value
