# generated from rosidl_generator_py/resource/_idl.py.em
# with input from my_msgs:msg/YoloDetection.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_YoloDetection(type):
    """Metaclass of message 'YoloDetection'."""

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
            module = import_type_support('my_msgs')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'my_msgs.msg.YoloDetection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__yolo_detection
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__yolo_detection
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__yolo_detection
            cls._TYPE_SUPPORT = module.type_support_msg__msg__yolo_detection
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__yolo_detection

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class YoloDetection(metaclass=Metaclass_YoloDetection):
    """Message class 'YoloDetection'."""

    __slots__ = [
        '_label',
        '_screen_width',
        '_screen_height',
        '_xmax',
        '_ymax',
        '_xmin',
        '_ymin',
    ]

    _fields_and_field_types = {
        'label': 'string',
        'screen_width': 'float',
        'screen_height': 'float',
        'xmax': 'float',
        'ymax': 'float',
        'xmin': 'float',
        'ymin': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
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
        self.label = kwargs.get('label', str())
        self.screen_width = kwargs.get('screen_width', float())
        self.screen_height = kwargs.get('screen_height', float())
        self.xmax = kwargs.get('xmax', float())
        self.ymax = kwargs.get('ymax', float())
        self.xmin = kwargs.get('xmin', float())
        self.ymin = kwargs.get('ymin', float())

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
        if self.screen_width != other.screen_width:
            return False
        if self.screen_height != other.screen_height:
            return False
        if self.xmax != other.xmax:
            return False
        if self.ymax != other.ymax:
            return False
        if self.xmin != other.xmin:
            return False
        if self.ymin != other.ymin:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @property
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

    @property
    def screen_width(self):
        """Message field 'screen_width'."""
        return self._screen_width

    @screen_width.setter
    def screen_width(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'screen_width' field must be of type 'float'"
        self._screen_width = value

    @property
    def screen_height(self):
        """Message field 'screen_height'."""
        return self._screen_height

    @screen_height.setter
    def screen_height(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'screen_height' field must be of type 'float'"
        self._screen_height = value

    @property
    def xmax(self):
        """Message field 'xmax'."""
        return self._xmax

    @xmax.setter
    def xmax(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'xmax' field must be of type 'float'"
        self._xmax = value

    @property
    def ymax(self):
        """Message field 'ymax'."""
        return self._ymax

    @ymax.setter
    def ymax(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ymax' field must be of type 'float'"
        self._ymax = value

    @property
    def xmin(self):
        """Message field 'xmin'."""
        return self._xmin

    @xmin.setter
    def xmin(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'xmin' field must be of type 'float'"
        self._xmin = value

    @property
    def ymin(self):
        """Message field 'ymin'."""
        return self._ymin

    @ymin.setter
    def ymin(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'ymin' field must be of type 'float'"
        self._ymin = value
