"""autogenerated by genpy from mrp1_operator/Task_list.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import mrp1_operator.msg

class Task_list(genpy.Message):
  _md5sum = "460d07b8af670c9162f2c38ac9a4b5b2"
  _type = "mrp1_operator/Task_list"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """mrp1_operator/Task[] list
================================================================================
MSG: mrp1_operator/Task
string tag
string task
int64 time
bool isDone
"""
  __slots__ = ['list']
  _slot_types = ['mrp1_operator/Task[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       list

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Task_list, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.list is None:
        self.list = []
    else:
      self.list = []

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      length = len(self.list)
      buff.write(_struct_I.pack(length))
      for val1 in self.list:
        _x = val1.tag
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.task
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_qB.pack(_x.time, _x.isDone))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      if self.list is None:
        self.list = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.list = []
      for i in range(0, length):
        val1 = mrp1_operator.msg.Task()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.tag = str[start:end].decode('utf-8')
        else:
          val1.tag = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.task = str[start:end].decode('utf-8')
        else:
          val1.task = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.time, _x.isDone,) = _struct_qB.unpack(str[start:end])
        val1.isDone = bool(val1.isDone)
        self.list.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      length = len(self.list)
      buff.write(_struct_I.pack(length))
      for val1 in self.list:
        _x = val1.tag
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1.task
        length = len(_x)
        if python3 or type(_x) == unicode:
          _x = _x.encode('utf-8')
          length = len(_x)
        buff.write(struct.pack('<I%ss'%length, length, _x))
        _x = val1
        buff.write(_struct_qB.pack(_x.time, _x.isDone))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      if self.list is None:
        self.list = None
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      self.list = []
      for i in range(0, length):
        val1 = mrp1_operator.msg.Task()
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.tag = str[start:end].decode('utf-8')
        else:
          val1.tag = str[start:end]
        start = end
        end += 4
        (length,) = _struct_I.unpack(str[start:end])
        start = end
        end += length
        if python3:
          val1.task = str[start:end].decode('utf-8')
        else:
          val1.task = str[start:end]
        _x = val1
        start = end
        end += 9
        (_x.time, _x.isDone,) = _struct_qB.unpack(str[start:end])
        val1.isDone = bool(val1.isDone)
        self.list.append(val1)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_qB = struct.Struct("<qB")