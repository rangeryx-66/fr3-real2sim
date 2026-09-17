"""Offline estimator and audit report for one PayloadID v2 acquisition.

This command reads only the measured NPZ captures and the reconstructed visual
mesh (if supplied).  A GT material file can be passed solely for a final error
report; it is never opened by the estimator itself.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import numpy as np

from real2sim.payload_id_v2 import (
    assemble_physical_parameters,
    dynamic_inertia_identify,
    load_record,
    static_gravity_identify,
)


def _arr(value: Any) -> np.ndarray:
    return np.asarray(value, dtype=float)


def _gt_summary(path: Path) -> dict[str, Any]:
    """Read Arena material only after estimation, for evaluation output."""
    data = json.loads(path.read_text())
    material = data.get("runtime_material", {}).get("material", data.get("material", data))
    mass = material.get("target_mass_kg", material.get("mass"))
    com = material.get("target_COM_local", material.get("center_of_mass"))
    inertia = material.get("target_inertia", material.get("inertia_matrix"))
    if mass is None or com is None or inertia is None:
        raise ValueError("GT material file does not contain mass/COM/inertia")
    com = _arr(com).reshape(-1); com = com[:3]
    inertia = _arr(inertia); inertia = np.diag(inertia[:3]) if inertia.size == 3 else inertia.reshape(3,3)
    return {"mass_kg":float(_arr(mass).reshape(-1)[0]),"center_of_mass_m":com.tolist(),"inertia_matrix":inertia.tolist(),"evaluation_only":True,"source":str(path)}


def _errors(estimate: dict[str, Any], gt: dict[str, Any]) -> dict[str, Any]:
    m=float(estimate["mass"]); mt=float(gt["mass_kg"]); c=_arr(estimate["center_of_mass"]); ct=_arr(gt["center_of_mass_m"]); I=_arr(estimate["inertia_matrix"]); It=_arr(gt["inertia_matrix"])
    return {"mass_relative_error_pct":abs(m-mt)/max(abs(mt),1e-12)*100.0,"com_error_mm":float(np.linalg.norm(c-ct)*1000.0),"inertia_relative_frobenius_pct":float(np.linalg.norm(I-It)/max(np.linalg.norm(It),1e-12)*100.0)}


def main() -> None:
    p=argparse.ArgumentParser();p.add_argument("--run",type=Path,required=True);p.add_argument("--mesh",type=Path);p.add_argument("--gt",type=Path);p.add_argument("--nominal-mass",type=float);a=p.parse_args();run=a.run.resolve()
    protocol_path=run/"payload_id_v2_protocol.json"; protocol=json.loads(protocol_path.read_text()) if protocol_path.exists() else None
    def read(path: Path):
        return load_record(path) if path.exists() else None
    baseline_static=read(run/"system_id_baseline_static.npz"); payload_static=read(run/"system_id_payload_static.npz")
    baseline_dynamic=read(run/"system_id_baseline_dynamic.npz"); payload_dynamic=read(run/"system_id_payload_dynamic.npz")
    if baseline_static is None or payload_static is None:
        static={"schema":"payload_id_v2/static_gravity/v1","stage":"STATIC_GRAVITY_ID","source":"UNOBSERVABLE","accepted":False,"reason":"NO_RECORD","guard_passed":False}
    else:
        static=static_gravity_identify(baseline_static,payload_static,protocol.get("static") if protocol else None)
    if baseline_dynamic is None or payload_dynamic is None:
        dynamic={"schema":"payload_id_v2/dynamic_inertia/v1","stage":"DYNAMIC_ID","source":"UNOBSERVABLE","accepted":False,"reason":"NO_RECORD","guard_passed":False}
    else:
        dynamic=dynamic_inertia_identify(baseline_dynamic,payload_dynamic,static,protocol)
    (run/"static_gravity_estimate.json").write_text(json.dumps(static,indent=2))
    (run/"dynamic_inertia_estimate.json").write_text(json.dumps(dynamic,indent=2))
    physical=assemble_physical_parameters(static,dynamic,a.mesh,run/"physical_parameters_v2.json",nominal_mass_kg=a.nominal_mass)
    report={"schema":"payload_id_v2/audit/v1","run":str(run),"static":static,"dynamic":dynamic,"physical_parameters":physical,"gt_used_for_estimation":False}
    if a.gt:
        gt=_gt_summary(a.gt); report["gt_evaluation_only"]=gt
        if (physical["mass"] is not None and physical["center_of_mass"] is not None and physical["inertia_matrix"] is not None
                and np.isfinite(float(physical["mass"])) and np.isfinite(_arr(physical["center_of_mass"])).all()
                and np.isfinite(_arr(physical["inertia_matrix"])).all()): report["errors"]=_errors(physical,gt)
    (run/"payload_id_v2_report.json").write_text(json.dumps(report,indent=2,allow_nan=False));print(json.dumps(report,indent=2))


if __name__=="__main__":main()
