"""autogenerated by genpy from bender_head_arduino/ServoState.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class ServoState(genpy.Message):
  _md5sum = "416cf11bef5430ddf566421886e047af"
  _type = "bender_head_arduino/ServoState"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """string lastCommand
int16 servo0
int16 servo1
int16 servo2
int16 servo3
int16 servo4

"""
  __slots__ = ['lastCommand','servo0','servo1','servo2','servo3','servo4']
  _slot_types = ['string','int16','int16','int16','int16','int16']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       lastCommand,servo0,servo1,servo2,servo3,servo4

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(ServoState, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.lastCommand is None:
        self.lastCommand = ''
      if self.servo0 is None:
        self.servo0 = 0
      if self.servo1 is None:
        self.servo1 = 0
      if self.servo2 is None:
        self.servo2 = 0
      if self.servo3 is None:
        self.servo3 = 0
      if self.servo4 is None:
        self.servo4 = 0
    else:
      self.lastCommand = ''
      self.servo0 = 0
      self.servo1 = 0
      self.servo2 = 0
      self.servo3 = 0
      self.servo4 = 0

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
      _x = self.lastCommand
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_5h.pack(_x.servo0, _x.servo1, _x.servo2, _x.servo3, _x.servo4))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.lastCommand = str[start:end].decode('utf-8')
      else:
        self.lastCommand = str[start:end]
      _x = self
      start = end
      end += 10
      (_x.servo0, _x.servo1, _x.servo2, _x.servo3, _x.servo4,) = _struct_5h.unpack(str[start:end])
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
      _x = self.lastCommand
      length = len(_x)
      if python3 or type(_x) == unicode:
        _x = _x.encode('utf-8')
        length = len(_x)
      buff.write(struct.pack('<I%ss'%length, length, _x))
      _x = self
      buff.write(_struct_5h.pack(_x.servo0, _x.servo1, _x.servo2, _x.servo3, _x.servo4))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      start = end
      end += 4
      (length,) = _struct_I.unpack(str[start:end])
      start = end
      end += length
      if python3:
        self.lastCommand = str[start:end].decode('utf-8')
      else:
        self.lastCommand = str[start:end]
      _x = self
      start = end
      end += 10
      (_x.servo0, _x.servo1, _x.servo2, _x.servo3, _x.servo4,) = _struct_5h.unpack(str[start:end])
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_5h = struct.Struct("<5h")
