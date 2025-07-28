# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/UBXSecSig.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UBXSecSig(type):
    """Metaclass of message 'UBXSecSig'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'JAM_UNKNOWN': 0,
        'JAM_NO_JAMMING': 1,
        'JAM_WARNING': 2,
        'JAM_CRITICAL': 3,
        'SPF_UNKNOWN': 0,
        'SPF_NO_SPOOFING': 1,
        'SPF_SPOOFING_INDICATED': 2,
        'SPF_SPOOFING_AFFIRMED': 3,
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
                'ublox_ubx_msgs.msg.UBXSecSig')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ubx_sec_sig
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ubx_sec_sig
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ubx_sec_sig
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ubx_sec_sig
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ubx_sec_sig

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'JAM_UNKNOWN': cls.__constants['JAM_UNKNOWN'],
            'JAM_NO_JAMMING': cls.__constants['JAM_NO_JAMMING'],
            'JAM_WARNING': cls.__constants['JAM_WARNING'],
            'JAM_CRITICAL': cls.__constants['JAM_CRITICAL'],
            'SPF_UNKNOWN': cls.__constants['SPF_UNKNOWN'],
            'SPF_NO_SPOOFING': cls.__constants['SPF_NO_SPOOFING'],
            'SPF_SPOOFING_INDICATED': cls.__constants['SPF_SPOOFING_INDICATED'],
            'SPF_SPOOFING_AFFIRMED': cls.__constants['SPF_SPOOFING_AFFIRMED'],
        }

    @property
    def JAM_UNKNOWN(self):
        """Message constant 'JAM_UNKNOWN'."""
        return Metaclass_UBXSecSig.__constants['JAM_UNKNOWN']

    @property
    def JAM_NO_JAMMING(self):
        """Message constant 'JAM_NO_JAMMING'."""
        return Metaclass_UBXSecSig.__constants['JAM_NO_JAMMING']

    @property
    def JAM_WARNING(self):
        """Message constant 'JAM_WARNING'."""
        return Metaclass_UBXSecSig.__constants['JAM_WARNING']

    @property
    def JAM_CRITICAL(self):
        """Message constant 'JAM_CRITICAL'."""
        return Metaclass_UBXSecSig.__constants['JAM_CRITICAL']

    @property
    def SPF_UNKNOWN(self):
        """Message constant 'SPF_UNKNOWN'."""
        return Metaclass_UBXSecSig.__constants['SPF_UNKNOWN']

    @property
    def SPF_NO_SPOOFING(self):
        """Message constant 'SPF_NO_SPOOFING'."""
        return Metaclass_UBXSecSig.__constants['SPF_NO_SPOOFING']

    @property
    def SPF_SPOOFING_INDICATED(self):
        """Message constant 'SPF_SPOOFING_INDICATED'."""
        return Metaclass_UBXSecSig.__constants['SPF_SPOOFING_INDICATED']

    @property
    def SPF_SPOOFING_AFFIRMED(self):
        """Message constant 'SPF_SPOOFING_AFFIRMED'."""
        return Metaclass_UBXSecSig.__constants['SPF_SPOOFING_AFFIRMED']


class UBXSecSig(metaclass=Metaclass_UBXSecSig):
    """
    Message class 'UBXSecSig'.

    Constants:
      JAM_UNKNOWN
      JAM_NO_JAMMING
      JAM_WARNING
      JAM_CRITICAL
      SPF_UNKNOWN
      SPF_NO_SPOOFING
      SPF_SPOOFING_INDICATED
      SPF_SPOOFING_AFFIRMED
    """

    __slots__ = [
        '_header',
        '_version',
        '_jam_det_enabled',
        '_jamming_state',
        '_spf_det_enabled',
        '_spoofing_state',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'version': 'uint8',
        'jam_det_enabled': 'uint8',
        'jamming_state': 'uint8',
        'spf_det_enabled': 'uint8',
        'spoofing_state': 'uint8',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.version = kwargs.get('version', int())
        self.jam_det_enabled = kwargs.get('jam_det_enabled', int())
        self.jamming_state = kwargs.get('jamming_state', int())
        self.spf_det_enabled = kwargs.get('spf_det_enabled', int())
        self.spoofing_state = kwargs.get('spoofing_state', int())

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
        if self.jam_det_enabled != other.jam_det_enabled:
            return False
        if self.jamming_state != other.jamming_state:
            return False
        if self.spf_det_enabled != other.spf_det_enabled:
            return False
        if self.spoofing_state != other.spoofing_state:
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
    def jam_det_enabled(self):
        """Message field 'jam_det_enabled'."""
        return self._jam_det_enabled

    @jam_det_enabled.setter
    def jam_det_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'jam_det_enabled' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'jam_det_enabled' field must be an unsigned integer in [0, 255]"
        self._jam_det_enabled = value

    @builtins.property
    def jamming_state(self):
        """Message field 'jamming_state'."""
        return self._jamming_state

    @jamming_state.setter
    def jamming_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'jamming_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'jamming_state' field must be an unsigned integer in [0, 255]"
        self._jamming_state = value

    @builtins.property
    def spf_det_enabled(self):
        """Message field 'spf_det_enabled'."""
        return self._spf_det_enabled

    @spf_det_enabled.setter
    def spf_det_enabled(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'spf_det_enabled' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'spf_det_enabled' field must be an unsigned integer in [0, 255]"
        self._spf_det_enabled = value

    @builtins.property
    def spoofing_state(self):
        """Message field 'spoofing_state'."""
        return self._spoofing_state

    @spoofing_state.setter
    def spoofing_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'spoofing_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'spoofing_state' field must be an unsigned integer in [0, 255]"
        self._spoofing_state = value
