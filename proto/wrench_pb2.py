# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: wrench.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import vector3d_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='wrench.proto',
  package='gazebo.msgs',
  serialized_pb='\n\x0cwrench.proto\x12\x0bgazebo.msgs\x1a\x0evector3d.proto\"U\n\x06Wrench\x12$\n\x05\x66orce\x18\x01 \x02(\x0b\x32\x15.gazebo.msgs.Vector3d\x12%\n\x06torque\x18\x02 \x02(\x0b\x32\x15.gazebo.msgs.Vector3d')




_WRENCH = _descriptor.Descriptor(
  name='Wrench',
  full_name='gazebo.msgs.Wrench',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='force', full_name='gazebo.msgs.Wrench.force', index=0,
      number=1, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='torque', full_name='gazebo.msgs.Wrench.torque', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
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
  serialized_start=45,
  serialized_end=130,
)

_WRENCH.fields_by_name['force'].message_type = vector3d_pb2._VECTOR3D
_WRENCH.fields_by_name['torque'].message_type = vector3d_pb2._VECTOR3D
DESCRIPTOR.message_types_by_name['Wrench'] = _WRENCH

class Wrench(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _WRENCH

  # @@protoc_insertion_point(class_scope:gazebo.msgs.Wrench)


# @@protoc_insertion_point(module_scope)