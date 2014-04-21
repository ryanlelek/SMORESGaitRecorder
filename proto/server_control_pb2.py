# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: server_control.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import header_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='server_control.proto',
  package='gazebo.msgs',
  serialized_pb='\n\x14server_control.proto\x12\x0bgazebo.msgs\x1a\x0cheader.proto\"i\n\rServerControl\x12\x17\n\x0fsave_world_name\x18\x01 \x01(\t\x12\x15\n\rsave_filename\x18\x02 \x01(\t\x12\x15\n\ropen_filename\x18\x03 \x01(\t\x12\x11\n\tnew_world\x18\x04 \x01(\x08')




_SERVERCONTROL = _descriptor.Descriptor(
  name='ServerControl',
  full_name='gazebo.msgs.ServerControl',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='save_world_name', full_name='gazebo.msgs.ServerControl.save_world_name', index=0,
      number=1, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='save_filename', full_name='gazebo.msgs.ServerControl.save_filename', index=1,
      number=2, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='open_filename', full_name='gazebo.msgs.ServerControl.open_filename', index=2,
      number=3, type=9, cpp_type=9, label=1,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='new_world', full_name='gazebo.msgs.ServerControl.new_world', index=3,
      number=4, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
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
  serialized_start=51,
  serialized_end=156,
)

DESCRIPTOR.message_types_by_name['ServerControl'] = _SERVERCONTROL

class ServerControl(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _SERVERCONTROL

  # @@protoc_insertion_point(class_scope:gazebo.msgs.ServerControl)


# @@protoc_insertion_point(module_scope)
