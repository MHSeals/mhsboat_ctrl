# generated from rosidl_generator_py/resource/_idl.py.em
# with input from boat_interfaces:msg/AiOutput.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'confidences'
# Member 'lefts'
# Member 'tops'
# Member 'widths'
# Member 'heights'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_AiOutput(type):
    """Metaclass of message 'AiOutput'."""

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
            module = import_type_support('boat_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'boat_interfaces.msg.AiOutput')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__ai_output
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__ai_output
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__ai_output
            cls._TYPE_SUPPORT = module.type_support_msg__msg__ai_output
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__ai_output

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class AiOutput(metaclass=Metaclass_AiOutput):
    """Message class 'AiOutput'."""

    __slots__ = [
        '_num',
        '_img_width',
        '_img_height',
        '_types',
        '_confidences',
        '_lefts',
        '_tops',
        '_widths',
        '_heights',
    ]

    _fields_and_field_types = {
        'num': 'int32',
        'img_width': 'int32',
        'img_height': 'int32',
        'types': 'sequence<string>',
        'confidences': 'sequence<int32>',
        'lefts': 'sequence<int32>',
        'tops': 'sequence<int32>',
        'widths': 'sequence<int32>',
        'heights': 'sequence<int32>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.num = kwargs.get('num', int())
        self.img_width = kwargs.get('img_width', int())
        self.img_height = kwargs.get('img_height', int())
        self.types = kwargs.get('types', [])
        self.confidences = array.array('i', kwargs.get('confidences', []))
        self.lefts = array.array('i', kwargs.get('lefts', []))
        self.tops = array.array('i', kwargs.get('tops', []))
        self.widths = array.array('i', kwargs.get('widths', []))
        self.heights = array.array('i', kwargs.get('heights', []))

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
        if self.num != other.num:
            return False
        if self.img_width != other.img_width:
            return False
        if self.img_height != other.img_height:
            return False
        if self.types != other.types:
            return False
        if self.confidences != other.confidences:
            return False
        if self.lefts != other.lefts:
            return False
        if self.tops != other.tops:
            return False
        if self.widths != other.widths:
            return False
        if self.heights != other.heights:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def num(self):
        """Message field 'num'."""
        return self._num

    @num.setter
    def num(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'num' field must be an integer in [-2147483648, 2147483647]"
        self._num = value

    @builtins.property
    def img_width(self):
        """Message field 'img_width'."""
        return self._img_width

    @img_width.setter
    def img_width(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'img_width' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'img_width' field must be an integer in [-2147483648, 2147483647]"
        self._img_width = value

    @builtins.property
    def img_height(self):
        """Message field 'img_height'."""
        return self._img_height

    @img_height.setter
    def img_height(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'img_height' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'img_height' field must be an integer in [-2147483648, 2147483647]"
        self._img_height = value

    @builtins.property
    def types(self):
        """Message field 'types'."""
        return self._types

    @types.setter
    def types(self, value):
        if __debug__:
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'types' field must be a set or sequence and each value of type 'str'"
        self._types = value

    @builtins.property
    def confidences(self):
        """Message field 'confidences'."""
        return self._confidences

    @confidences.setter
    def confidences(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'confidences' array.array() must have the type code of 'i'"
            self._confidences = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'confidences' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._confidences = array.array('i', value)

    @builtins.property
    def lefts(self):
        """Message field 'lefts'."""
        return self._lefts

    @lefts.setter
    def lefts(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'lefts' array.array() must have the type code of 'i'"
            self._lefts = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'lefts' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._lefts = array.array('i', value)

    @builtins.property
    def tops(self):
        """Message field 'tops'."""
        return self._tops

    @tops.setter
    def tops(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'tops' array.array() must have the type code of 'i'"
            self._tops = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'tops' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._tops = array.array('i', value)

    @builtins.property
    def widths(self):
        """Message field 'widths'."""
        return self._widths

    @widths.setter
    def widths(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'widths' array.array() must have the type code of 'i'"
            self._widths = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'widths' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._widths = array.array('i', value)

    @builtins.property
    def heights(self):
        """Message field 'heights'."""
        return self._heights

    @heights.setter
    def heights(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'heights' array.array() must have the type code of 'i'"
            self._heights = value
            return
        if __debug__:
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'heights' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._heights = array.array('i', value)
