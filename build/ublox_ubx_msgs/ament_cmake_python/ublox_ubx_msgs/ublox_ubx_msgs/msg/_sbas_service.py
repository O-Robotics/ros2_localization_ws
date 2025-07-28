# generated from rosidl_generator_py/resource/_idl.py.em
# with input from ublox_ubx_msgs:msg/SBASService.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SBASService(type):
    """Metaclass of message 'SBASService'."""

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
                'ublox_ubx_msgs.msg.SBASService')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__sbas_service
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__sbas_service
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__sbas_service
            cls._TYPE_SUPPORT = module.type_support_msg__msg__sbas_service
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__sbas_service

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SBASService(metaclass=Metaclass_SBASService):
    """Message class 'SBASService'."""

    __slots__ = [
        '_ranging',
        '_corrections',
        '_integrity',
        '_test_mode',
        '_bad',
    ]

    _fields_and_field_types = {
        'ranging': 'boolean',
        'corrections': 'boolean',
        'integrity': 'boolean',
        'test_mode': 'boolean',
        'bad': 'boolean',
    }

    SLOT_TYPES = (
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
        self.ranging = kwargs.get('ranging', bool())
        self.corrections = kwargs.get('corrections', bool())
        self.integrity = kwargs.get('integrity', bool())
        self.test_mode = kwargs.get('test_mode', bool())
        self.bad = kwargs.get('bad', bool())

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
        if self.ranging != other.ranging:
            return False
        if self.corrections != other.corrections:
            return False
        if self.integrity != other.integrity:
            return False
        if self.test_mode != other.test_mode:
            return False
        if self.bad != other.bad:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def ranging(self):
        """Message field 'ranging'."""
        return self._ranging

    @ranging.setter
    def ranging(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'ranging' field must be of type 'bool'"
        self._ranging = value

    @builtins.property
    def corrections(self):
        """Message field 'corrections'."""
        return self._corrections

    @corrections.setter
    def corrections(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'corrections' field must be of type 'bool'"
        self._corrections = value

    @builtins.property
    def integrity(self):
        """Message field 'integrity'."""
        return self._integrity

    @integrity.setter
    def integrity(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'integrity' field must be of type 'bool'"
        self._integrity = value

    @builtins.property
    def test_mode(self):
        """Message field 'test_mode'."""
        return self._test_mode

    @test_mode.setter
    def test_mode(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'test_mode' field must be of type 'bool'"
        self._test_mode = value

    @builtins.property
    def bad(self):
        """Message field 'bad'."""
        return self._bad

    @bad.setter
    def bad(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'bad' field must be of type 'bool'"
        self._bad = value
