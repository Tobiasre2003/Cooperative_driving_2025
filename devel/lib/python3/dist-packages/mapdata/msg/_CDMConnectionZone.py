# This Python file uses the following encoding: utf-8
"""autogenerated by genpy from mapdata/CDMConnectionZone.msg. Do not edit."""
import codecs
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct

import mapdata.msg

class CDMConnectionZone(genpy.Message):
  _md5sum = "49ee6d673b89f273a2281d8ddb4755f9"
  _type = "mapdata/CDMConnectionZone"
  _has_header = False  # flag to mark the presence of a Header object
  _full_text = """Position pos
uint32 width
uint32 height

================================================================================
MSG: mapdata/Position
int32 x
int32 y
"""
  __slots__ = ['pos','width','height']
  _slot_types = ['mapdata/Position','uint32','uint32']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       pos,width,height

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(CDMConnectionZone, self).__init__(*args, **kwds)
      # message fields cannot be None, assign default values for those that are
      if self.pos is None:
        self.pos = mapdata.msg.Position()
      if self.width is None:
        self.width = 0
      if self.height is None:
        self.height = 0
    else:
      self.pos = mapdata.msg.Position()
      self.width = 0
      self.height = 0

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
      buff.write(_get_struct_2i2I().pack(_x.pos.x, _x.pos.y, _x.width, _x.height))
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
      if self.pos is None:
        self.pos = mapdata.msg.Position()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.pos.x, _x.pos.y, _x.width, _x.height,) = _get_struct_2i2I().unpack(str[start:end])
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
      buff.write(_get_struct_2i2I().pack(_x.pos.x, _x.pos.y, _x.width, _x.height))
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
      if self.pos is None:
        self.pos = mapdata.msg.Position()
      end = 0
      _x = self
      start = end
      end += 16
      (_x.pos.x, _x.pos.y, _x.width, _x.height,) = _get_struct_2i2I().unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e)  # most likely buffer underfill

_struct_I = genpy.struct_I
def _get_struct_I():
    global _struct_I
    return _struct_I
_struct_2i2I = None
def _get_struct_2i2I():
    global _struct_2i2I
    if _struct_2i2I is None:
        _struct_2i2I = struct.Struct("<2i2I")
    return _struct_2i2I
