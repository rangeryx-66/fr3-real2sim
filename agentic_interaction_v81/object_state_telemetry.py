"""Read-only all-DOF object evidence for single-target interaction studies.

Constructor-only, opt-in instrumentation. No object command, model input,
recovery permission, or protection threshold is changed by these channels.
"""
from copy import deepcopy
import numpy as np

from .load_telemetry import _copy_field


def instrument(runtime):
    enabled = runtime.job.get('v86_object_state_telemetry', False)
    if type(enabled) is not bool:
        raise ValueError('BOOLEAN_OBJECT_STATE_TELEMETRY_REQUIRED')
    if not enabled:
        return dict(enabled=False)
    record = runtime.record
    if getattr(record, '_v86_object_state_installed', False):
        return deepcopy(record.metadata['v86_object_state'])
    if record.count != 0:
        raise ValueError('OBJECT_STATE_REQUIRES_CONSTRUCTOR_BEFORE_FIRST_RECORD')
    articulation = runtime.scene['articulation']
    names = list(articulation.dof_names)
    selected = runtime.scene.get('selected_asset_dof', 0)
    if (not names or len(set(names)) != len(names)
            or type(selected) is not int or not 0 <= selected < len(names)):
        raise ValueError('INITIALIZED_OBJECT_DOF_MAPPING_REQUIRED')
    previous_measurement = runtime.enrich_measurement
    previous_fields = record.coordinate_fields

    def measurement(row, pre_contact_poses):
        previous_measurement(row, pre_contact_poses)
        # Same completed physics step as the original selected-coordinate row;
        # getter errors are unavailable evidence, never a new motion veto.
        for channel, getter in [('object_all_q_native', articulation.get_joint_positions),
                                ('object_all_dq_native', articulation.get_joint_velocities)]:
            try:
                value = getter()
            except Exception:
                value = None
            row[channel], row[channel+'_valid'] = _copy_field(value, (len(names),))

    def fields(row):
        values = dict(previous_fields(row))
        for channel in ('object_all_q_native', 'object_all_dq_native'):
            if channel in values or channel+'_valid' in values:
                raise ValueError('OBJECT_STATE_COLUMN_COLLISION:'+channel)
            values[channel], valid = _copy_field(row.get(channel), (len(names),))
            values[channel+'_valid'] = np.uint8(bool(valid) and bool(row.get(channel+'_valid', False)))
        return values

    metadata = dict(schema='v86-readonly-object-all-dof-v1', enabled=True,
                    dof_names=names, selected_dof=selected,
                    passive_dofs=[i for i in range(len(names)) if i != selected],
                    units='native SI per named DOF: prismatic m/m s^-1; revolute rad/rad s^-1',
                    alignment='same completed physics step/t as original selected-coordinate record',
                    unavailable='fixed-shape NaN and valid=0; no interpolation or zero substitution',
                    additional_readonly_queries_per_record=2,
                    object_actuation=False, feeds_controller_or_estimator=False,
                    missing_channel_changes_motion_permission=False)
    runtime.enrich_measurement = measurement
    record.coordinate_fields = fields
    record.metadata['v86_object_state'] = metadata
    record._v86_object_state_installed = True
    return deepcopy(metadata)
