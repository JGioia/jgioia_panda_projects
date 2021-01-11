# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: movement.proto

import sys
_b=sys.version_info[0]<3 and (lambda x:x) or (lambda x:x.encode('latin1'))
from google.protobuf import descriptor as _descriptor
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
from google.protobuf import descriptor_pb2
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor.FileDescriptor(
  name='movement.proto',
  package='Movement',
  syntax='proto2',
  serialized_pb=_b('\n\x0emovement.proto\x12\x08Movement\"\x1a\n\x04Wait\x12\x12\n\x07seconds\x18\x01 \x01(\x01:\x01\x30\"+\n\x08PoseGoal\x12\t\n\x01x\x18\x01 \x01(\x01\x12\t\n\x01y\x18\x02 \x01(\x01\x12\t\n\x01z\x18\x03 \x01(\x01\"{\n\tJointGoal\x12\x0e\n\x06joint1\x18\x01 \x01(\x01\x12\x0e\n\x06joint2\x18\x02 \x01(\x01\x12\x0e\n\x06joint3\x18\x03 \x01(\x01\x12\x0e\n\x06joint4\x18\x04 \x01(\x01\x12\x0e\n\x06joint5\x18\x05 \x01(\x01\x12\x0e\n\x06joint6\x18\x06 \x01(\x01\x12\x0e\n\x06joint7\x18\x07 \x01(\x01\"V\n\x08GripGoal\x12)\n\x04type\x18\x01 \x01(\x0e\x32\x1b.Movement.GripGoal.GripType\"\x1f\n\x08GripType\x12\x08\n\x04OPEN\x10\x00\x12\t\n\x05\x43LOSE\x10\x01\"\xa1\x01\n\x11MovementOperation\x12\x1c\n\x04wait\x18\x01 \x01(\x0b\x32\x0e.Movement.Wait\x12$\n\x08poseGoal\x18\x02 \x01(\x0b\x32\x12.Movement.PoseGoal\x12&\n\tjointGoal\x18\x03 \x01(\x0b\x32\x13.Movement.JointGoal\x12 \n\x04grip\x18\x04 \x01(\x0b\x32\x12.Movement.GripGoal\"E\n\x12MovementCollection\x12/\n\noperations\x18\x01 \x03(\x0b\x32\x1b.Movement.MovementOperation')
)
_sym_db.RegisterFileDescriptor(DESCRIPTOR)



_GRIPGOAL_GRIPTYPE = _descriptor.EnumDescriptor(
  name='GripType',
  full_name='Movement.GripGoal.GripType',
  filename=None,
  file=DESCRIPTOR,
  values=[
    _descriptor.EnumValueDescriptor(
      name='OPEN', index=0, number=0,
      options=None,
      type=None),
    _descriptor.EnumValueDescriptor(
      name='CLOSE', index=1, number=1,
      options=None,
      type=None),
  ],
  containing_type=None,
  options=None,
  serialized_start=281,
  serialized_end=312,
)
_sym_db.RegisterEnumDescriptor(_GRIPGOAL_GRIPTYPE)


_WAIT = _descriptor.Descriptor(
  name='Wait',
  full_name='Movement.Wait',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='seconds', full_name='Movement.Wait.seconds', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=True, default_value=float(0),
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=28,
  serialized_end=54,
)


_POSEGOAL = _descriptor.Descriptor(
  name='PoseGoal',
  full_name='Movement.PoseGoal',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='x', full_name='Movement.PoseGoal.x', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='y', full_name='Movement.PoseGoal.y', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='z', full_name='Movement.PoseGoal.z', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=56,
  serialized_end=99,
)


_JOINTGOAL = _descriptor.Descriptor(
  name='JointGoal',
  full_name='Movement.JointGoal',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='joint1', full_name='Movement.JointGoal.joint1', index=0,
      number=1, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='joint2', full_name='Movement.JointGoal.joint2', index=1,
      number=2, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='joint3', full_name='Movement.JointGoal.joint3', index=2,
      number=3, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='joint4', full_name='Movement.JointGoal.joint4', index=3,
      number=4, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='joint5', full_name='Movement.JointGoal.joint5', index=4,
      number=5, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='joint6', full_name='Movement.JointGoal.joint6', index=5,
      number=6, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='joint7', full_name='Movement.JointGoal.joint7', index=6,
      number=7, type=1, cpp_type=5, label=1,
      has_default_value=False, default_value=float(0),
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=101,
  serialized_end=224,
)


_GRIPGOAL = _descriptor.Descriptor(
  name='GripGoal',
  full_name='Movement.GripGoal',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='type', full_name='Movement.GripGoal.type', index=0,
      number=1, type=14, cpp_type=8, label=1,
      has_default_value=False, default_value=0,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
  ],
  extensions=[
  ],
  nested_types=[],
  enum_types=[
    _GRIPGOAL_GRIPTYPE,
  ],
  options=None,
  is_extendable=False,
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=226,
  serialized_end=312,
)


_MOVEMENTOPERATION = _descriptor.Descriptor(
  name='MovementOperation',
  full_name='Movement.MovementOperation',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='wait', full_name='Movement.MovementOperation.wait', index=0,
      number=1, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='poseGoal', full_name='Movement.MovementOperation.poseGoal', index=1,
      number=2, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='jointGoal', full_name='Movement.MovementOperation.jointGoal', index=2,
      number=3, type=11, cpp_type=10, label=1,
      has_default_value=False, default_value=None,
      message_type=None, enum_type=None, containing_type=None,
      is_extension=False, extension_scope=None,
      options=None),
    _descriptor.FieldDescriptor(
      name='grip', full_name='Movement.MovementOperation.grip', index=3,
      number=4, type=11, cpp_type=10, label=1,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=315,
  serialized_end=476,
)


_MOVEMENTCOLLECTION = _descriptor.Descriptor(
  name='MovementCollection',
  full_name='Movement.MovementCollection',
  filename=None,
  file=DESCRIPTOR,
  containing_type=None,
  fields=[
    _descriptor.FieldDescriptor(
      name='operations', full_name='Movement.MovementCollection.operations', index=0,
      number=1, type=11, cpp_type=10, label=3,
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
  syntax='proto2',
  extension_ranges=[],
  oneofs=[
  ],
  serialized_start=478,
  serialized_end=547,
)

_GRIPGOAL.fields_by_name['type'].enum_type = _GRIPGOAL_GRIPTYPE
_GRIPGOAL_GRIPTYPE.containing_type = _GRIPGOAL
_MOVEMENTOPERATION.fields_by_name['wait'].message_type = _WAIT
_MOVEMENTOPERATION.fields_by_name['poseGoal'].message_type = _POSEGOAL
_MOVEMENTOPERATION.fields_by_name['jointGoal'].message_type = _JOINTGOAL
_MOVEMENTOPERATION.fields_by_name['grip'].message_type = _GRIPGOAL
_MOVEMENTCOLLECTION.fields_by_name['operations'].message_type = _MOVEMENTOPERATION
DESCRIPTOR.message_types_by_name['Wait'] = _WAIT
DESCRIPTOR.message_types_by_name['PoseGoal'] = _POSEGOAL
DESCRIPTOR.message_types_by_name['JointGoal'] = _JOINTGOAL
DESCRIPTOR.message_types_by_name['GripGoal'] = _GRIPGOAL
DESCRIPTOR.message_types_by_name['MovementOperation'] = _MOVEMENTOPERATION
DESCRIPTOR.message_types_by_name['MovementCollection'] = _MOVEMENTCOLLECTION

Wait = _reflection.GeneratedProtocolMessageType('Wait', (_message.Message,), dict(
  DESCRIPTOR = _WAIT,
  __module__ = 'movement_pb2'
  # @@protoc_insertion_point(class_scope:Movement.Wait)
  ))
_sym_db.RegisterMessage(Wait)

PoseGoal = _reflection.GeneratedProtocolMessageType('PoseGoal', (_message.Message,), dict(
  DESCRIPTOR = _POSEGOAL,
  __module__ = 'movement_pb2'
  # @@protoc_insertion_point(class_scope:Movement.PoseGoal)
  ))
_sym_db.RegisterMessage(PoseGoal)

JointGoal = _reflection.GeneratedProtocolMessageType('JointGoal', (_message.Message,), dict(
  DESCRIPTOR = _JOINTGOAL,
  __module__ = 'movement_pb2'
  # @@protoc_insertion_point(class_scope:Movement.JointGoal)
  ))
_sym_db.RegisterMessage(JointGoal)

GripGoal = _reflection.GeneratedProtocolMessageType('GripGoal', (_message.Message,), dict(
  DESCRIPTOR = _GRIPGOAL,
  __module__ = 'movement_pb2'
  # @@protoc_insertion_point(class_scope:Movement.GripGoal)
  ))
_sym_db.RegisterMessage(GripGoal)

MovementOperation = _reflection.GeneratedProtocolMessageType('MovementOperation', (_message.Message,), dict(
  DESCRIPTOR = _MOVEMENTOPERATION,
  __module__ = 'movement_pb2'
  # @@protoc_insertion_point(class_scope:Movement.MovementOperation)
  ))
_sym_db.RegisterMessage(MovementOperation)

MovementCollection = _reflection.GeneratedProtocolMessageType('MovementCollection', (_message.Message,), dict(
  DESCRIPTOR = _MOVEMENTCOLLECTION,
  __module__ = 'movement_pb2'
  # @@protoc_insertion_point(class_scope:Movement.MovementCollection)
  ))
_sym_db.RegisterMessage(MovementCollection)


# @@protoc_insertion_point(module_scope)
