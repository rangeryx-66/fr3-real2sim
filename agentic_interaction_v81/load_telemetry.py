"""Opt-in copies of already sampled native load feedback; never motor torque.

This records row values only. It does not query a sensor, submit a command,
change sampling, or feed the controller, protection, or Physics-ID worker.
Install during construction, before the recorder's first sample.
"""
from copy import deepcopy

import numpy as np


def _copy_field(value, shape):
    """A missing/malformed channel cannot create a ragged/object NPZ column."""
    try:
        array = np.asarray(value, dtype=np.float64)
        valid = array.shape == shape and bool(np.isfinite(array).all())
    except (TypeError, ValueError, OverflowError):
        valid = False
    if not valid:
        return np.full(shape, np.nan, dtype=np.float64), np.uint8(0)
    return array.copy(), np.uint8(1)


def instrument(runtime):
    """Wrap coordinate_fields only for explicit ``v83_load_telemetry=True``.

    Constructor-only installation prevents missing columns in earlier chunks.
    The initialized WristFT joint mapping supplies the fixed incoming-reaction
    shape, including fixed joints and the root row. No physical model is read.
    """
    enabled = runtime.job.get('v83_load_telemetry', False)
    if type(enabled) is not bool:
        raise ValueError('BOOLEAN_V83_LOAD_TELEMETRY_OPT_IN_REQUIRED')
    if not enabled:
        return dict(enabled=False)
    record = runtime.record
    if getattr(record, '_v83_load_telemetry_installed', False):
        return deepcopy(record.metadata['v83_load_telemetry'])
    if record.count != 0:
        raise ValueError('LOAD_TELEMETRY_REQUIRES_CONSTRUCTOR_BEFORE_FIRST_RECORD')
    names = list(runtime.names)
    arm = [int(i) for i in runtime.arm]
    mapping = deepcopy(runtime.sensor.mapping)
    joint_names = list(mapping['joint_names'])
    joint_indices = dict(mapping['joint_indices'])
    if (not names or len(set(names)) != len(names) or not arm
            or len(set(arm)) != len(arm) or min(arm) < 0 or max(arm) >= len(names)
            or set(joint_names) != set(joint_indices)
            or sorted(joint_indices.values()) != list(range(len(joint_names)))):
        raise ValueError('INITIALIZED_NATIVE_DOF_AND_INCOMING_JOINT_MAPPING_REQUIRED')
    channels = {
        'projected_joint_efforts_native': ('projected_joint_efforts', (len(names),)),
        'explicit_applied_joint_efforts_native': ('explicit_applied_efforts', (len(names),)),
        'incoming_joint_reactions_native': ('joint_reactions', (len(joint_names)+1, 6)),
        'actuator_position_target_arm_rad': ('actuator_position_target_arm', (len(arm),)),
        'actuator_gravity_bias_arm_nm': ('actuator_gravity_bias_nm', (len(arm),)),
        'native_G1_master_effort_cap_nm': ('native_G1_master_effort_cap_nm', ()),
    }
    metadata = dict(
        schema='v83-already-sampled-load-telemetry-v1', enabled=True,
        installed_before_step=record.count, native_dof_order=names, arm_indices=arm,
        incoming_joint_mapping=mapping,
        columns={name: dict(source_row_field=source, shape=list(shape),
                           available_column=name+'_valid')
                 for name, (source, shape) in channels.items()},
        missing_policy='Missing, wrong-shape, or nonfinite channel: all NaN and uint8 valid=0; no zero substitution.',
        alignment='Same step/t and original recorder retention as all other row channels; no additional sample or independent clock.',
        projected_efforts='Native projected joint reaction feedback, not independently measured implicit drive/motor output.',
        explicit_applied_efforts='Native API applied-effort channel; may describe explicit external commands. It is not proof of total implicit motor output.',
        incoming_reactions='Raw native incoming-joint spatial reactions [Fx,Fy,Fz,Tx,Ty,Tz]; N and N m, native joint frame; row 0 is root, joint index+1 is incoming row. Not motor torque.',
        native_effort_units='Native DOF effort units: N for prismatic and N m for revolute; full DOF order is retained, including XY carriage.',
        actuator_position_target='Implicit drive position command including original gravity compensation, radians for arm; not measured motor output.',
        actuator_gravity_bias='Original authored arm gravity-compensation command, N m; not measured motor output.',
        master_effort_cap='Commanded/native drive authority cap, N m; not measured clamp force or motor output.',
        feeds_protection_controller_or_estimator=False,
        additional_sensor_queries=False, original_sampling_unchanged=True,
    )
    previous = record.coordinate_fields

    def fields(row):
        values = dict(previous(row))
        for name, (source, shape) in channels.items():
            if name in values or name+'_valid' in values:
                raise ValueError('LOAD_TELEMETRY_COLUMN_COLLISION:'+name)
            values[name], values[name+'_valid'] = _copy_field(row.get(source), shape)
        return values

    record.coordinate_fields = fields
    record.metadata['v83_load_telemetry'] = metadata
    record._v83_load_telemetry_installed = True
    return deepcopy(metadata)
