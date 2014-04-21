# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: visual.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import pose_pb2
import geometry_pb2
import material_pb2
import plugin_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='visual.proto',
  package='gazebo.msgs',
  serialized_pb='\n\x0cvisual.proto\x12\x0bgazebo.msgs\x1a\npose.proto\x1a\x0egeometry.proto\x1a\x0ematerial.proto\x1a\x0cplugin.proto\"\xbb\x02\n\x06Visual\x12\x0c\n\x04name\x18\x01 \x02(\t\x12\x13\n\x0bparent_name\x18\x02 \x02(\t\x12\x14\n\x0c\x63\x61st_shadows\x18\x03 \x01(\x08\x12\x14\n\x0ctransparency\x18\x04 \x01(\x01\x12\x13\n\x0blaser_retro\x18\x05 \x01(\x01\x12\x1f\n\x04pose\x18\x06 \x01(\x0b\x32\x11.gazebo.msgs.Pose\x12\'\n\x08geometry\x18\x07 \x01(\x0b\x32\x15.gazebo.msgs.Geometry\x12\'\n\x08material\x18\x08 \x01(\x0b\x32\x15.gazebo.msgs.Material\x12\x0f\n\x07visible\x18\t \x01(\x08\x12\x11\n\tdelete_me\x18\x0b \x01(\x08\x12\x11\n\tis_static\x18\x0c \x01(\x08\x12#\n\x06plugin\x18\r \x01(\x0b\x32\x13.gazebo.msgs.Plugin')




_VISUAL = _descriptor.Descriptor(
  name='Visual',
  full_name='gazebo.msgs.Visual',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='name', full_name='gazebo.msgs.Visual.name', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='parent_name', full_name='gazebo.msgs.Visual.parent_name', index=1,
      number=2, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='cast_shadows', full_name='gazebo.msgs.Visual.cast_shadows', index=2,
      number=3, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='transparency', full_name='gazebo.msgs.Visual.transparency', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='laser_retro', full_name='gazebo.msgs.Visual.laser_retro', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='pose', full_name='gazebo.msgs.Visual.pose', index=5,
      number=6, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='geometry', full_name='gazebo.msgs.Visual.geometry', index=6,
      number=7, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='material', full_name='gazebo.msgs.Visual.material', index=7,
      number=8, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='visible', full_name='gazebo.msgs.Visual.visible', index=8,
      number=9, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='delete_me', full_name='gazebo.msgs.Visual.delete_me', index=9,
      number=11, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='is_static', full_name='gazebo.msgs.Visual.is_static', index=10,
      number=12, type=8, cpp_type=7, label=1,
      has_default_value=False, default_value=False,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='plugin', full_name='gazebo.msgs.Visual.plugin', index=11,
      number=13, type=11, cpp_type=10, label=1,
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
  serialized_start=88,
  serialized_end=403,
)

_VISUAL.fields_by_name['pose'].message_type = pose_pb2._POSE
_VISUAL.fields_by_name['geometry'].message_type = geometry_pb2._GEOMETRY
_VISUAL.fields_by_name['material'].message_type = material_pb2._MATERIAL
_VISUAL.fields_by_name['plugin'].message_type = plugin_pb2._PLUGIN
DESCRIPTOR.message_types_by_name['Visual'] = _VISUAL

class Visual(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _VISUAL

  # @@protoc_insertion_point(class_scope:gazebo.msgs.Visual)


# @@protoc_insertion_point(module_scope)
