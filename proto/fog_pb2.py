# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: fog.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import color_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='fog.proto',
  package='gazebo.msgs',
  serialized_pb='\n\tfog.proto\x12\x0bgazebo.msgs\x1a\x0b\x63olor.proto\"\xc1\x01\n\x03\x46og\x12&\n\x04type\x18\x01 \x01(\x0e\x32\x18.gazebo.msgs.Fog.FogType\x12!\n\x05\x63olor\x18\x02 \x01(\x0b\x32\x12.gazebo.msgs.Color\x12\x0f\n\x07\x64\x65nsity\x18\x03 \x01(\x02\x12\r\n\x05start\x18\x04 \x01(\x02\x12\x0b\n\x03\x65nd\x18\x05 \x01(\x02\"B\n\x07\x46ogType\x12\x08\n\x04NONE\x10\x01\x12\n\n\x06LINEAR\x10\x02\x12\x0f\n\x0b\x45XPONENTIAL\x10\x03\x12\x10\n\x0c\x45XPONENTIAL2\x10\x04')



_FOG_FOGTYPE = _descriptor.EnumDescriptor(
  name='FogType',
  full_name='gazebo.msgs.Fog.FogType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='NONE', index=0, number=1,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='LINEAR', index=1, number=2,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='EXPONENTIAL', index=2, number=3,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='EXPONENTIAL2', index=3, number=4,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=167,
  serialized_end=233,
)


_FOG = _descriptor.Descriptor(
  name='Fog',
  full_name='gazebo.msgs.Fog',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='gazebo.msgs.Fog.type', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='color', full_name='gazebo.msgs.Fog.color', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='density', full_name='gazebo.msgs.Fog.density', index=2,
      number=3, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='start', full_name='gazebo.msgs.Fog.start', index=3,
      number=4, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='end', full_name='gazebo.msgs.Fog.end', index=4,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _FOG_FOGTYPE,
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=40,
  serialized_end=233,
)

_FOG.fields_by_name['type'].enum_type = _FOG_FOGTYPE
_FOG.fields_by_name['color'].message_type = color_pb2._COLOR
_FOG_FOGTYPE.containing_type = _FOG;
DESCRIPTOR.message_types_by_name['Fog'] = _FOG

class Fog(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _FOG

  # @@protoc_insertion_point(class_scope:gazebo.msgs.Fog)


# @@protoc_insertion_point(module_scope)
