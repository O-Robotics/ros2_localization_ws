# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXEsfStatus.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXEsfStatus(type):
    """Metaclass of message 'UBXEsfStatus'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'WT_INIT_STATUS_OFF': 0,
        'WT_INIT_STATUS_INITIALIZING': 1,
        'WT_INIT_STATUS_INITIALIZED': 2,
        'MBT_ALG_STATUS_OFF': 0,
        'MBT_ALG_STATUS_INITIALIZING': 1,
        'MBT_ALG_STATUS_INITIALIZED0': 2,
        'MBT_ALG_STATUS_INITIALIZED1': 3,
        'INS_INIT_STATUS_OFF': 0,
        'INS_INIT_STATUS_INITIALIZING': 1,
        'INS_INIT_STATUS_INITIALIZED': 2,
        'IMU_INIT_STATUS_OFF': 0,
        'IMU_INIT_STATUS_INITIALIZING': 1,
        'IMU_INIT_STATUS_INITIALIZED': 2,
        'FUSION_MODE_INITIALIZATION': 0,
        'FUSION_MODE_WORKING': 1,
        'FUSION_MODE_SUSPENDED': 2,
        'FUSION_MODE_DISABLED': 3,
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
                'ublox_ubx_msgs.msg.UBXEsfStatus')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_esf_status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_esf_status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_esf_status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_esf_status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_esf_status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

            from ublox_ubx_msgs.msg import ESFSensorStatus
            if ESFSensorStatus.__class__._TYPE_SUPPORT is None:
                ESFSensorStatus.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'WT_INIT_STATUS_OFF': cls.__constants['WT_INIT_STATUS_OFF'],
            'WT_INIT_STATUS_INITIALIZING': cls.__constants['WT_INIT_STATUS_INITIALIZING'],
            'WT_INIT_STATUS_INITIALIZED': cls.__constants['WT_INIT_STATUS_INITIALIZED'],
            'MBT_ALG_STATUS_OFF': cls.__constants['MBT_ALG_STATUS_OFF'],
            'MBT_ALG_STATUS_INITIALIZING': cls.__constants['MBT_ALG_STATUS_INITIALIZING'],
            'MBT_ALG_STATUS_INITIALIZED0': cls.__constants['MBT_ALG_STATUS_INITIALIZED0'],
            'MBT_ALG_STATUS_INITIALIZED1': cls.__constants['MBT_ALG_STATUS_INITIALIZED1'],
            'INS_INIT_STATUS_OFF': cls.__constants['INS_INIT_STATUS_OFF'],
            'INS_INIT_STATUS_INITIALIZING': cls.__constants['INS_INIT_STATUS_INITIALIZING'],
            'INS_INIT_STATUS_INITIALIZED': cls.__constants['INS_INIT_STATUS_INITIALIZED'],
            'IMU_INIT_STATUS_OFF': cls.__constants['IMU_INIT_STATUS_OFF'],
            'IMU_INIT_STATUS_INITIALIZING': cls.__constants['IMU_INIT_STATUS_INITIALIZING'],
            'IMU_INIT_STATUS_INITIALIZED': cls.__constants['IMU_INIT_STATUS_INITIALIZED'],
            'FUSION_MODE_INITIALIZATION': cls.__constants['FUSION_MODE_INITIALIZATION'],
            'FUSION_MODE_WORKING': cls.__constants['FUSION_MODE_WORKING'],
            'FUSION_MODE_SUSPENDED': cls.__constants['FUSION_MODE_SUSPENDED'],
            'FUSION_MODE_DISABLED': cls.__constants['FUSION_MODE_DISABLED'],
        }

    @property
    def WT_INIT_STATUS_OFF(self):
        """Message constant 'WT_INIT_STATUS_OFF'."""
        return Metaclass_UBXEsfStatus.__constants['WT_INIT_STATUS_OFF']

    @property
    def WT_INIT_STATUS_INITIALIZING(self):
        """Message constant 'WT_INIT_STATUS_INITIALIZING'."""
        return Metaclass_UBXEsfStatus.__constants['WT_INIT_STATUS_INITIALIZING']

    @property
    def WT_INIT_STATUS_INITIALIZED(self):
        """Message constant 'WT_INIT_STATUS_INITIALIZED'."""
        return Metaclass_UBXEsfStatus.__constants['WT_INIT_STATUS_INITIALIZED']

    @property
    def MBT_ALG_STATUS_OFF(self):
        """Message constant 'MBT_ALG_STATUS_OFF'."""
        return Metaclass_UBXEsfStatus.__constants['MBT_ALG_STATUS_OFF']

    @property
    def MBT_ALG_STATUS_INITIALIZING(self):
        """Message constant 'MBT_ALG_STATUS_INITIALIZING'."""
        return Metaclass_UBXEsfStatus.__constants['MBT_ALG_STATUS_INITIALIZING']

    @property
    def MBT_ALG_STATUS_INITIALIZED0(self):
        """Message constant 'MBT_ALG_STATUS_INITIALIZED0'."""
        return Metaclass_UBXEsfStatus.__constants['MBT_ALG_STATUS_INITIALIZED0']

    @property
    def MBT_ALG_STATUS_INITIALIZED1(self):
        """Message constant 'MBT_ALG_STATUS_INITIALIZED1'."""
        return Metaclass_UBXEsfStatus.__constants['MBT_ALG_STATUS_INITIALIZED1']

    @property
    def INS_INIT_STATUS_OFF(self):
        """Message constant 'INS_INIT_STATUS_OFF'."""
        return Metaclass_UBXEsfStatus.__constants['INS_INIT_STATUS_OFF']

    @property
    def INS_INIT_STATUS_INITIALIZING(self):
        """Message constant 'INS_INIT_STATUS_INITIALIZING'."""
        return Metaclass_UBXEsfStatus.__constants['INS_INIT_STATUS_INITIALIZING']

    @property
    def INS_INIT_STATUS_INITIALIZED(self):
        """Message constant 'INS_INIT_STATUS_INITIALIZED'."""
        return Metaclass_UBXEsfStatus.__constants['INS_INIT_STATUS_INITIALIZED']

    @property
    def IMU_INIT_STATUS_OFF(self):
        """Message constant 'IMU_INIT_STATUS_OFF'."""
        return Metaclass_UBXEsfStatus.__constants['IMU_INIT_STATUS_OFF']

    @property
    def IMU_INIT_STATUS_INITIALIZING(self):
        """Message constant 'IMU_INIT_STATUS_INITIALIZING'."""
        return Metaclass_UBXEsfStatus.__constants['IMU_INIT_STATUS_INITIALIZING']

    @property
    def IMU_INIT_STATUS_INITIALIZED(self):
        """Message constant 'IMU_INIT_STATUS_INITIALIZED'."""
        return Metaclass_UBXEsfStatus.__constants['IMU_INIT_STATUS_INITIALIZED']

    @property
    def FUSION_MODE_INITIALIZATION(self):
        """Message constant 'FUSION_MODE_INITIALIZATION'."""
        return Metaclass_UBXEsfStatus.__constants['FUSION_MODE_INITIALIZATION']

    @property
    def FUSION_MODE_WORKING(self):
        """Message constant 'FUSION_MODE_WORKING'."""
        return Metaclass_UBXEsfStatus.__constants['FUSION_MODE_WORKING']

    @property
    def FUSION_MODE_SUSPENDED(self):
        """Message constant 'FUSION_MODE_SUSPENDED'."""
        return Metaclass_UBXEsfStatus.__constants['FUSION_MODE_SUSPENDED']

    @property
    def FUSION_MODE_DISABLED(self):
        """Message constant 'FUSION_MODE_DISABLED'."""
        return Metaclass_UBXEsfStatus.__constants['FUSION_MODE_DISABLED']


class UBXEsfStatus(metaclass=Metaclass_UBXEsfStatus):
    """
    Message class 'UBXEsfStatus'.

    Constants:
      WT_INIT_STATUS_OFF
      WT_INIT_STATUS_INITIALIZING
      WT_INIT_STATUS_INITIALIZED
      MBT_ALG_STATUS_OFF
      MBT_ALG_STATUS_INITIALIZING
      MBT_ALG_STATUS_INITIALIZED0
      MBT_ALG_STATUS_INITIALIZED1
      INS_INIT_STATUS_OFF
      INS_INIT_STATUS_INITIALIZING
      INS_INIT_STATUS_INITIALIZED
      IMU_INIT_STATUS_OFF
      IMU_INIT_STATUS_INITIALIZING
      IMU_INIT_STATUS_INITIALIZED
      FUSION_MODE_INITIALIZATION
      FUSION_MODE_WORKING
      FUSION_MODE_SUSPENDED
      FUSION_MODE_DISABLED
    """

    __slots__ = [
        '_header',
        '_itow',
        '_version',
        '_wt_init_status',
        '_mnt_alg_status',
        '_ins_init_status',
        '_imu_init_status',
        '_fusion_mode',
        '_num_sens',
        '_sensor_statuses',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'itow': 'uint32',
        'version': 'uint8',
        'wt_init_status': 'uint8',
        'mnt_alg_status': 'uint8',
        'ins_init_status': 'uint8',
        'imu_init_status': 'uint8',
        'fusion_mode': 'uint8',
        'num_sens': 'uint8',
        'sensor_statuses': 'sequence<ublox_ubx_msgs/ESFSensorStatus>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['ublox_ubx_msgs', 'msg'], 'ESFSensorStatus')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.itow = kwargs.get('itow', int())
        self.version = kwargs.get('version', int())
        self.wt_init_status = kwargs.get('wt_init_status', int())
        self.mnt_alg_status = kwargs.get('mnt_alg_status', int())
        self.ins_init_status = kwargs.get('ins_init_status', int())
        self.imu_init_status = kwargs.get('imu_init_status', int())
        self.fusion_mode = kwargs.get('fusion_mode', int())
        self.num_sens = kwargs.get('num_sens', int())
        self.sensor_statuses = kwargs.get('sensor_statuses', [])

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
        if self.wt_init_status != other.wt_init_status:
            return False
        if self.mnt_alg_status != other.mnt_alg_status:
            return False
        if self.ins_init_status != other.ins_init_status:
            return False
        if self.imu_init_status != other.imu_init_status:
            return False
        if self.fusion_mode != other.fusion_mode:
            return False
        if self.num_sens != other.num_sens:
            return False
        if self.sensor_statuses != other.sensor_statuses:
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
    def wt_init_status(self):
        """Message field 'wt_init_status'."""
        return self._wt_init_status

    @wt_init_status.setter
    def wt_init_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'wt_init_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'wt_init_status' field must be an unsigned integer in [0, 255]"
        self._wt_init_status = value

    @builtins.property
    def mnt_alg_status(self):
        """Message field 'mnt_alg_status'."""
        return self._mnt_alg_status

    @mnt_alg_status.setter
    def mnt_alg_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'mnt_alg_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'mnt_alg_status' field must be an unsigned integer in [0, 255]"
        self._mnt_alg_status = value

    @builtins.property
    def ins_init_status(self):
        """Message field 'ins_init_status'."""
        return self._ins_init_status

    @ins_init_status.setter
    def ins_init_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ins_init_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'ins_init_status' field must be an unsigned integer in [0, 255]"
        self._ins_init_status = value

    @builtins.property
    def imu_init_status(self):
        """Message field 'imu_init_status'."""
        return self._imu_init_status

    @imu_init_status.setter
    def imu_init_status(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'imu_init_status' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'imu_init_status' field must be an unsigned integer in [0, 255]"
        self._imu_init_status = value

    @builtins.property
    def fusion_mode(self):
        """Message field 'fusion_mode'."""
        return self._fusion_mode

    @fusion_mode.setter
    def fusion_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'fusion_mode' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'fusion_mode' field must be an unsigned integer in [0, 255]"
        self._fusion_mode = value

    @builtins.property
    def num_sens(self):
        """Message field 'num_sens'."""
        return self._num_sens

    @num_sens.setter
    def num_sens(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_sens' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'num_sens' field must be an unsigned integer in [0, 255]"
        self._num_sens = value

    @builtins.property
    def sensor_statuses(self):
        """Message field 'sensor_statuses'."""
        return self._sensor_statuses

    @sensor_statuses.setter
    def sensor_statuses(self, value):
        if __debug__:
            from ublox_ubx_msgs.msg import ESFSensorStatus
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, ESFSensorStatus) for v in value) and
                 True), \
                "The 'sensor_statuses' field must be a set or sequence and each value of type 'ESFSensorStatus'"
        self._sensor_statuses = value
