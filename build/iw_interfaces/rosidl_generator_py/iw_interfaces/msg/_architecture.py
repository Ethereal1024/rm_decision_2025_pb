# generated from rosidl_generator_py/resource/_idl.py.em
# with input from iw_interfaces:msg/Architecture.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Architecture(type):
    """Metaclass of message 'Architecture'."""

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
            module = import_type_support('iw_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'iw_interfaces.msg.Architecture')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__architecture
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__architecture
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__architecture
            cls._TYPE_SUPPORT = module.type_support_msg__msg__architecture
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__architecture

            from iw_interfaces.msg import PlaneCoordinate
            if PlaneCoordinate.__class__._TYPE_SUPPORT is None:
                PlaneCoordinate.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Architecture(metaclass=Metaclass_Architecture):
    """Message class 'Architecture'."""

    __slots__ = [
        '_label',
        '_vertices',
        '_hp',
        '_faction',
    ]

    _fields_and_field_types = {
        'label': 'string',
        'vertices': 'sequence<iw_interfaces/PlaneCoordinate>',
        'hp': 'uint32',
        'faction': 'uint32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['iw_interfaces', 'msg'], 'PlaneCoordinate')),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.label = kwargs.get('label', str())
        self.vertices = kwargs.get('vertices', [])
        self.hp = kwargs.get('hp', int())
        self.faction = kwargs.get('faction', int())

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
        if self.label != other.label:
            return False
        if self.vertices != other.vertices:
            return False
        if self.hp != other.hp:
            return False
        if self.faction != other.faction:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def label(self):
        """Message field 'label'."""
        return self._label

    @label.setter
    def label(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'label' field must be of type 'str'"
        self._label = value

    @builtins.property
    def vertices(self):
        """Message field 'vertices'."""
        return self._vertices

    @vertices.setter
    def vertices(self, value):
        if __debug__:
            from iw_interfaces.msg import PlaneCoordinate
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
                 all(isinstance(v, PlaneCoordinate) for v in value) and
                 True), \
                "The 'vertices' field must be a set or sequence and each value of type 'PlaneCoordinate'"
        self._vertices = value

    @builtins.property
    def hp(self):
        """Message field 'hp'."""
        return self._hp

    @hp.setter
    def hp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'hp' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'hp' field must be an unsigned integer in [0, 4294967295]"
        self._hp = value

    @builtins.property
    def faction(self):
        """Message field 'faction'."""
        return self._faction

    @faction.setter
    def faction(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'faction' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'faction' field must be an unsigned integer in [0, 4294967295]"
        self._faction = value
