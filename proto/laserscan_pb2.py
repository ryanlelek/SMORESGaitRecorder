# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: laserscan.proto

from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)


import pose_pb2


DESCRIPTOR = _descriptor.FileDescriptor(
  name='laserscan.proto',
  package='gazebo.msgs',
  serialized_pb='\n\x0flaserscan.proto\x12\x0bgazebo.msgs\x1a\npose.proto\"\xc6\x01\n\tLaserScan\x12\r\n\x05\x66rame\x18\x01 \x02(\t\x12%\n\nworld_pose\x18\x02 \x02(\x0b\x32\x11.gazebo.msgs.Pose\x12\x11\n\tangle_min\x18\x03 \x02(\x01\x12\x11\n\tangle_max\x18\x04 \x02(\x01\x12\x12\n\nangle_step\x18\x05 \x02(\x01\x12\x11\n\trange_min\x18\x06 \x02(\x01\x12\x11\n\trange_max\x18\x07 \x02(\x01\x12\x0e\n\x06ranges\x18\x08 \x03(\x01\x12\x13\n\x0bintensities\x18\t \x03(\x01')




_LASERSCAN = _descriptor.Descriptor(
  name='LaserScan',
  full_name='gazebo.msgs.LaserScan',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='frame', full_name='gazebo.msgs.LaserScan.frame', index=0,
      number=1, type=9, cpp_type=9, label=2,
      has_default_value=False, default_value=unicode("", "utf-8"),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='world_pose', full_name='gazebo.msgs.LaserScan.world_pose', index=1,
      number=2, type=11, cpp_type=10, label=2,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='angle_min', full_name='gazebo.msgs.LaserScan.angle_min', index=2,
      number=3, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='angle_max', full_name='gazebo.msgs.LaserScan.angle_max', index=3,
      number=4, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='angle_step', full_name='gazebo.msgs.LaserScan.angle_step', index=4,
      number=5, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='range_min', full_name='gazebo.msgs.LaserScan.range_min', index=5,
      number=6, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='range_max', full_name='gazebo.msgs.LaserScan.range_max', index=6,
      number=7, type=1, cpp_type=5, label=2,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='ranges', full_name='gazebo.msgs.LaserScan.ranges', index=7,
      number=8, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='intensities', full_name='gazebo.msgs.LaserScan.intensities', index=8,
      number=9, type=1, cpp_type=5, label=3,
      has_default_value=False, default_value=[],
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
  serialized_end=243,
)

_LASERSCAN.fields_by_name['world_pose'].message_type = pose_pb2._POSE
DESCRIPTOR.message_types_by_name['LaserScan'] = _LASERSCAN

class LaserScan(_message.Message):
  __metaclass__ = _reflection.GeneratedProtocolMessageType
  DESCRIPTOR = _LASERSCAN

  # @@protoc_insertion_point(class_scope:gazebo.msgs.LaserScan)


# @@protoc_insertion_point(module_scope)
