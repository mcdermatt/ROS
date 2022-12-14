# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from ICET/Num.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import genpy

class Num(genpy.Message):
  _md5sum = "93042447ed01f85739c5e6e8683f8ec7"
  _type = "ICET/Num"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """time timestamp
bool restart
int32 frame
string status
float32[] true_transform
"""
  __slots__ = ['timestamp','restart','frame','status','true_transform']
  _slot_types = ['time','bool','int32','string','float32[]']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       timestamp,restart,frame,status,true_transform

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(Num, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      if self.restart is None:
        self.restart = False
      if self.frame is None:
        self.frame = 0
      if self.status is None:
        self.status = ''
      if self.true_transform is None:
        self.true_transform = []
    else:
      self.timestamp = genpy.Time()
      self.restart = False
      self.frame = 0
      self.status = ''
      self.true_transform = []

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
      _x = self
      buff.write(_get_struct_2IBi().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.restart, _x.frame))
      _x = self.status
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.true_transform)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(struct.Struct(pattern).pack(*self.true_transform))
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 13
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.restart, _x.frame,) = _get_struct_2IBi().unpack(str[start:end])
      self.restart = bool(self.restart)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.status = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.true_transform = s.unpack(str[start:end])
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_get_struct_2IBi().pack(_x.timestamp.secs, _x.timestamp.nsecs, _x.restart, _x.frame))
      _x = self.status
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.Struct('<I%ss'%length).pack(length, _x))
      length = len(self.true_transform)
      buff.write(_struct_I.pack(length))
      pattern = '<%sf'%length
      buff.write(self.true_transform.tostring())
    except struct.error as se: self._check_types(struct.error("%s: '%s' when writing '%s'" % (type(se), str(se), str(locals().get('_x', self)))))
    except TypeError as te: self._check_types(ValueError("%s: '%s' when writing '%s'" % (type(te), str(te), str(locals().get('_x', self)))))

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    if python3:
      codecs.lookup_error("rosmsg").msg_type = self._type
    try:
      if self.timestamp is None:
        self.timestamp = genpy.Time()
      end = 0
      _x = self
      start = end
      end += 13
      (_x.timestamp.secs, _x.timestamp.nsecs, _x.restart, _x.frame,) = _get_struct_2IBi().unpack(str[start:end])
      self.restart = bool(self.restart)
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.status = str[start:end].decode('utf-8', 'rosmsg')
      else:
        self.status = str[start:end]
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      pattern = '<%sf'%length
      start = end
      s = struct.Struct(pattern)
      end += s.size
      self.true_transform = numpy.frombuffer(str[start:end], dtype=numpy.float32, count=length)
      self.timestamp.canon()
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2IBi = None
def _get_struct_2IBi():
    global _struct_2IBi
    if _struct_2IBi is None:
        _struct_2IBi = struct.Struct("<2IBi")
    return _struct_2IBi
