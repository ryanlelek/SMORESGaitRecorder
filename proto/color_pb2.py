# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: color.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)




DESCRIPTOR = _descriptor.FileDescriptor(
  name='color.proto',
  package='gazebo.msgs',
  serialized_pb='\n\x0b\x63olor.proto\x12\x0bgazebo.msgs\"6\n\x05\x43olor\x12\t\n\x01r\x18\x02 \x02(\x02\x12\t\n\x01g\x18\x03 \x02(\x02\x12\t\n\x01\x62\x18\x04 \x02(\x02\x12\x0c\n\x01\x61\x18\x05 \x01(\x02:\x01\x31')




_COLOR = _descriptor.Descriptor(
  name='Color',
  full_name='gazebo.msgs.Color',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='r', full_name='gazebo.msgs.Color.r', index=0,
      number=2, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='g', full_name='gazebo.msgs.Color.g', index=1,
      number=3, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='b', full_name='gazebo.msgs.Color.b', index=2,
      number=4, type=2, cpp_type=6, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='a', full_name='gazebo.msgs.Color.a', index=3,
      number=5, type=2, cpp_type=6, label=1,
      has_default_value=True, default_value=1,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
  ],
  options=None,
  is_extendable=False,
  extension_ranges=[],
  serialized_start=28,
  serialized_end=82,
)

DESCRIPTOR.message_types_by_name['Color'] = _COLOR

class Color(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _COLOR

  # @@protoc_insertion_point(class_scope:gazebo.msgs.Color)


# @@protoc_insertion_point(module_scope)
