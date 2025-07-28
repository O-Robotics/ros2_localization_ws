# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/SigLogEvent.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SigLogEvent(type):
    """Metaclass of message 'SigLogEvent'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'DETECTION_SIMULATED_SIGNAL': 0,
        'DETECTION_ABNORMAL_SIGNAL': 1,
        'DETECTION_INS_GNSS_MISMATCH': 2,
        'DETECTION_ABRUPT_CHANGES': 3,
        'DETECTION_BROADBAND_JAMMING': 4,
        'DETECTION_NARROWBAND_JAMMING': 5,
        'EVENT_STARTED': 0,
        'EVENT_STOPPED': 1,
        'EVENT_TRIGGERED': 2,
        'EVENT_TIMED_OUT': 3,
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
                'ublox_ubx_msgs.msg.SigLogEvent')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sig_log_event
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sig_log_event
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sig_log_event
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sig_log_event
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sig_log_event

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'DETECTION_SIMULATED_SIGNAL': cls.__constants['DETECTION_SIMULATED_SIGNAL'],
            'DETECTION_ABNORMAL_SIGNAL': cls.__constants['DETECTION_ABNORMAL_SIGNAL'],
            'DETECTION_INS_GNSS_MISMATCH': cls.__constants['DETECTION_INS_GNSS_MISMATCH'],
            'DETECTION_ABRUPT_CHANGES': cls.__constants['DETECTION_ABRUPT_CHANGES'],
            'DETECTION_BROADBAND_JAMMING': cls.__constants['DETECTION_BROADBAND_JAMMING'],
            'DETECTION_NARROWBAND_JAMMING': cls.__constants['DETECTION_NARROWBAND_JAMMING'],
            'EVENT_STARTED': cls.__constants['EVENT_STARTED'],
            'EVENT_STOPPED': cls.__constants['EVENT_STOPPED'],
            'EVENT_TRIGGERED': cls.__constants['EVENT_TRIGGERED'],
            'EVENT_TIMED_OUT': cls.__constants['EVENT_TIMED_OUT'],
        }

    @property
    def DETECTION_SIMULATED_SIGNAL(self):
        """Message constant 'DETECTION_SIMULATED_SIGNAL'."""
        return Metaclass_SigLogEvent.__constants['DETECTION_SIMULATED_SIGNAL']

    @property
    def DETECTION_ABNORMAL_SIGNAL(self):
        """Message constant 'DETECTION_ABNORMAL_SIGNAL'."""
        return Metaclass_SigLogEvent.__constants['DETECTION_ABNORMAL_SIGNAL']

    @property
    def DETECTION_INS_GNSS_MISMATCH(self):
        """Message constant 'DETECTION_INS_GNSS_MISMATCH'."""
        return Metaclass_SigLogEvent.__constants['DETECTION_INS_GNSS_MISMATCH']

    @property
    def DETECTION_ABRUPT_CHANGES(self):
        """Message constant 'DETECTION_ABRUPT_CHANGES'."""
        return Metaclass_SigLogEvent.__constants['DETECTION_ABRUPT_CHANGES']

    @property
    def DETECTION_BROADBAND_JAMMING(self):
        """Message constant 'DETECTION_BROADBAND_JAMMING'."""
        return Metaclass_SigLogEvent.__constants['DETECTION_BROADBAND_JAMMING']

    @property
    def DETECTION_NARROWBAND_JAMMING(self):
        """Message constant 'DETECTION_NARROWBAND_JAMMING'."""
        return Metaclass_SigLogEvent.__constants['DETECTION_NARROWBAND_JAMMING']

    @property
    def EVENT_STARTED(self):
        """Message constant 'EVENT_STARTED'."""
        return Metaclass_SigLogEvent.__constants['EVENT_STARTED']

    @property
    def EVENT_STOPPED(self):
        """Message constant 'EVENT_STOPPED'."""
        return Metaclass_SigLogEvent.__constants['EVENT_STOPPED']

    @property
    def EVENT_TRIGGERED(self):
        """Message constant 'EVENT_TRIGGERED'."""
        return Metaclass_SigLogEvent.__constants['EVENT_TRIGGERED']

    @property
    def EVENT_TIMED_OUT(self):
        """Message constant 'EVENT_TIMED_OUT'."""
        return Metaclass_SigLogEvent.__constants['EVENT_TIMED_OUT']


class SigLogEvent(metaclass=Metaclass_SigLogEvent):
    """
    Message class 'SigLogEvent'.

    Constants:
      DETECTION_SIMULATED_SIGNAL
      DETECTION_ABNORMAL_SIGNAL
      DETECTION_INS_GNSS_MISMATCH
      DETECTION_ABRUPT_CHANGES
      DETECTION_BROADBAND_JAMMING
      DETECTION_NARROWBAND_JAMMING
      EVENT_STARTED
      EVENT_STOPPED
      EVENT_TRIGGERED
      EVENT_TIMED_OUT
    """

    __slots__ = [
        '_time_elapsed',
        '_detection_type',
        '_event_type',
    ]

    _fields_and_field_types = {
        'time_elapsed': 'uint32',
        'detection_type': 'uint8',
        'event_type': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.time_elapsed = kwargs.get('time_elapsed', int())
        self.detection_type = kwargs.get('detection_type', int())
        self.event_type = kwargs.get('event_type', int())

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
        if self.time_elapsed != other.time_elapsed:
            return False
        if self.detection_type != other.detection_type:
            return False
        if self.event_type != other.event_type:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def time_elapsed(self):
        """Message field 'time_elapsed'."""
        return self._time_elapsed

    @time_elapsed.setter
    def time_elapsed(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time_elapsed' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'time_elapsed' field must be an unsigned integer in [0, 4294967295]"
        self._time_elapsed = value

    @builtins.property
    def detection_type(self):
        """Message field 'detection_type'."""
        return self._detection_type

    @detection_type.setter
    def detection_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'detection_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'detection_type' field must be an unsigned integer in [0, 255]"
        self._detection_type = value

    @builtins.property
    def event_type(self):
        """Message field 'event_type'."""
        return self._event_type

    @event_type.setter
    def event_type(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'event_type' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'event_type' field must be an unsigned integer in [0, 255]"
        self._event_type = value
