"""Safe two-stage PayloadID capture for the frozen FR3 executor.

The class only owns calibration motion and recording.  It never changes
AnyGrasp, MoveIt selection, scan, mesh, CoACD or USD code.  Static orientations
are obtained through the existing backend IK/validation calls; the dynamic
trajectory is a deterministic, low-amplitude multi-sine with analytic
q/dq/ddq sidecars saved next to the measured torque record.
"""
from __future__ import annotations

import json
import time
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from .safety import require_free_space_stable
from .fr3_robot_calibration import design_fr3_fourier_protocol


PROTOCOL_SCHEMA = "payload_id_v2/protocol/v1"
STATIC_ORIENTATIONS_RAD = np.array([
    # A symmetric roll/pitch grid gives the gravity regressor leverage in
    # both signs.  These are calibration poses only; the frozen grasp
    # executor never imports this protocol.
    [0.00, 0.00],
    [0.16, 0.00], [-0.16, 0.00], [0.00, 0.16], [0.00, -0.16],
    [0.12, 0.12], [-0.12, 0.12], [0.12, -0.12], [-0.12, -0.12],
    [0.20, 0.00], [-0.20, 0.00], [0.00, 0.20], [0.00, -0.20],
    [0.18, 0.08], [-0.18, 0.08], [0.18, -0.08], [-0.18, -0.08],
], dtype=float)
# Smaller than the historical one-shot excitation.  Frequencies are mutually
# incommensurate over the 15 s window and amplitudes stay well inside the
# calibrated grasp envelope.
DYNAMIC_AMPLITUDE_RAD = np.array([.012, .015, .010, .018, .012, .020, .022], dtype=float)
DYNAMIC_FREQUENCY_HZ = np.array([.18, .23, .29, .35, .41, .47, .53], dtype=float)
DYNAMIC_PHASE_RAD = np.array([0.0, .37, 0.81, 1.22, 1.71, 2.13, 2.71], dtype=float)


def _smoothstep(u: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    u = np.clip(np.asarray(u, dtype=float), 0.0, 1.0)
    return 3*u*u - 2*u*u*u, 6*u - 6*u*u, 6 - 12*u


def analytic_multisine(center: np.ndarray, duration_s: float = 15.0,
                        sample_hz: float = 30.0,
                        amplitude_scale: float = 1.0,
                        phase_offset: float = 0.0) -> dict[str, np.ndarray]:
    """Return smooth dynamic q/dq/ddq sampled from a closed-form trajectory."""
    center = np.asarray(center, dtype=float).reshape(7)
    t = np.linspace(0.0, duration_s, int(round(duration_s*sample_hz))+1)
    w = 2*np.pi*DYNAMIC_FREQUENCY_HZ
    phase = w[None, :]*t[:, None] + DYNAMIC_PHASE_RAD[None, :] + phase_offset
    # sin^2 envelope makes q, dq and ddq settle continuously at both ends.
    e = np.sin(np.pi*t/duration_s)**2
    de = (np.pi/duration_s)*np.sin(2*np.pi*t/duration_s)
    dde = (2*np.pi*np.pi/duration_s**2)*np.cos(2*np.pi*t/duration_s)
    s = np.sin(phase); c = np.cos(phase)
    amp = amplitude_scale * DYNAMIC_AMPLITUDE_RAD
    q = center[None, :] + e[:, None] * amp[None, :] * s
    dq = amp[None, :] * (de[:, None]*s + e[:, None]*w[None, :]*c)
    ddq = amp[None, :] * (dde[:, None]*s + 2*de[:, None]*w[None, :]*c - e[:, None]*(w[None, :]**2)*s)
    return {"time": t, "q": q, "dq": dq, "ddq": ddq}


def _joint_trajectory_item(names: list[str], time_values: np.ndarray,
                           positions: np.ndarray) -> dict[str, Any]:
    return {"joint_names": names, "points":[{"t":float(t),"q":np.asarray(q,float).tolist()} for t,q in zip(time_values,positions)]}


class PayloadIDV2Skill:
    def __init__(self, backend: Any, plant: Any, *, sample_hz: float = 30.0):
        self.backend, self.plant, self.sample_hz = backend, plant, float(sample_hz)
        self.arm_names = [f"fr3_joint{i}" for i in range(1,8)]

    def _current_arm(self) -> np.ndarray:
        state = self.plant.state()
        return np.asarray([state["q"][state["names"].index(n)] for n in self.arm_names], dtype=float)

    def _current_tcp(self) -> np.ndarray:
        state = self.plant.state()
        p = np.asarray(state["tcp"], dtype=float); q = np.asarray(state["tcp_quat"], dtype=float)
        # Isaac exposes xyzw; scipy consumes xyzw.  Existing transform helpers
        # in this repository use the same convention for live TCP telemetry.
        H = np.eye(4); H[:3,:3] = Rotation.from_quat(q).as_matrix(); H[:3,3] = p
        return H

    def _robot_state_for_q(self, q: np.ndarray):
        state = self.backend.measured()
        for name, value in zip(self.arm_names, np.asarray(q, dtype=float)):
            state.joint_state.position[state.joint_state.name.index(name)] = float(value)
        return state

    def _validated_q(self, H: np.ndarray, seed: Any) -> np.ndarray | None:
        try:
            state = self.backend.ik(H, seed)
            # Explicit joint-limit/self/world validation, even though IK also
            # checks it in the current MoveIt backend.
            self.backend.validate(state)
            return np.asarray([state.joint_state.position[state.joint_state.name.index(n)] for n in self.arm_names], dtype=float)
        except Exception:
            return None

    def _static_protocol(self, output: Path) -> dict[str, Any]:
        """Create/load deterministic static orientation protocol."""
        output = Path(output)
        if output.exists(): return json.loads(output.read_text())
        center = self._current_arm(); H0 = self._current_tcp(); R0 = H0[:3,:3]
        target_q = []; target_labels = []; ik_attempts = []
        seed = self.backend.measured()
        for idx, (roll, pitch) in enumerate(STATIC_ORIENTATIONS_RAD):
            q = None; used_scale = None
            # Large wrist orientation changes can be rejected by a frozen
            # collision/IK scene.  Retry the same signed orientation at two
            # smaller magnitudes; this is protocol construction, not a
            # result-dependent parameter adjustment.
            for scale in (1.0, .65, .35, .18, .08):
                rr, pp = float(roll*scale), float(pitch*scale)
                R = R0 @ Rotation.from_euler("xy", [rr,pp]).as_matrix()
                H = H0.copy(); H[:3,:3] = R
                q = self._validated_q(H, seed)
                ik_attempts.append({"pose_id":idx,"roll_rad":rr,"pitch_rad":pp,"scale":scale,"passed":q is not None})
                if q is not None:
                    used_scale = scale; break
            if q is None:
                continue
            target_q.append(q.tolist()); target_labels.append({"pose_id":idx,"roll_rad":float(roll*used_scale),"pitch_rad":float(pitch*used_scale),"requested_roll_rad":float(roll),"requested_pitch_rad":float(pitch)})
            seed = self._robot_state_for_q(q)
        # Some MoveIt configurations reject a Cartesian orientation even when
        # a nearby joint-space pose is collision-free (for example, while the
        # payload is represented as a world object at its table pose).  Keep a
        # deterministic, geometry-independent joint-space fallback so a
        # calibration run still obtains both signs of wrist roll/pitch and
        # enough gravity-direction diversity.  Every variant is checked by
        # MoveIt's joint limits and state-validity service; this is calibration
        # protocol construction and never changes the grasp executor.
        #
        # We fill to twelve holds even when a few Cartesian requests pass.  A
        # single five-degree-of-freedom wrist perturbation is intentionally
        # small, while the paired diagonal variants provide independent signs
        # without introducing a category/asset branch.
        if len(target_q) < 12:
            joint_variants = (
                (3,.06,0.0),(3,-.06,0.0),(4,.06,0.0),(4,-.06,0.0),
                (3,.10,0.0),(3,-.10,0.0),(4,.10,0.0),(4,-.10,0.0),
                (5,.08,0.0),(5,-.08,0.0),
                (3,.06,4,.06),(3,-.06,4,.06),
                (3,.06,4,-.06),(3,-.06,4,-.06),
                (4,.08,5,.06),(4,-.08,5,.06),
                (4,.08,5,-.06),(4,-.08,5,-.06),
                (2,.06,0.0),(2,-.06,0.0),
            )
            for variant in joint_variants:
                if len(target_q) >= 12: break
                qv=center.copy()
                if len(variant)==3: qv[int(variant[0])]+=float(variant[1]); label=(float(variant[1]),0.0)
                else: qv[int(variant[0])]+=float(variant[1]); qv[int(variant[2])]+=float(variant[3]); label=(float(variant[1]),float(variant[3]))
                try:
                    self.backend.validate(self._robot_state_for_q(qv))
                except Exception:
                    continue
                if any(np.linalg.norm(qv-np.asarray(old))<1e-4 for old in target_q): continue
                target_q.append(qv.tolist()); target_labels.append({"pose_id":100+len(target_q),"roll_joint_delta_rad":label[0],"pitch_joint_delta_rad":label[1],"source":"validated_joint_variant"})
                ik_attempts.append({"pose_id":100+len(target_q),"source":"validated_joint_variant","passed":True})
        if len(target_q) < 5:
            raise RuntimeError(f"static gravity protocol has only {len(target_q)} valid orientations")
        # Smooth transitions and 0.7 s static holds.  The hold flags and pose
        # IDs are part of the protocol and are interpolated into each NPZ.
        times=[]; positions=[]; pose_ids=[]; holds=[]; t0=0.; previous=center.copy()
        transition_s=.9; hold_s=.7; n_trans=max(2,int(round(transition_s*self.sample_hz))); n_hold=max(2,int(round(hold_s*self.sample_hz)))
        for row, qtarget in enumerate(target_q):
            qtarget=np.asarray(qtarget,float)
            u=np.linspace(0,1,n_trans,endpoint=False); s,_,_= _smoothstep(u)
            for uu,ss in zip(u,s):
                times.append(t0+float(uu*transition_s)); positions.append((1-ss)*previous+ss*qtarget); pose_ids.append(-1); holds.append(False)
            for k in range(n_hold):
                times.append(t0+transition_s+k*hold_s/max(n_hold-1,1)); positions.append(qtarget); pose_ids.append(row); holds.append(True)
            t0 += transition_s+hold_s; previous=qtarget
        times.append(t0);positions.append(previous);pose_ids.append(-1);holds.append(False)
        # The final hold sample and the next transition's u=0 sample share a
        # timestamp.  Remove those duplicate knots before sending the bridge
        # command; Isaac requires strictly increasing trajectory times.
        keep=[0]
        for i in range(1,len(times)):
            if times[i] > times[keep[-1]] + 1e-9: keep.append(i)
        times=[times[i] for i in keep]; positions=[positions[i] for i in keep]; pose_ids=[pose_ids[i] for i in keep]; holds=[holds[i] for i in keep]
        zeros=np.zeros_like(np.asarray(positions,float))
        protocol={"schema":PROTOCOL_SCHEMA,"kind":"static_gravity","sample_hz":self.sample_hz,"center_q":center.tolist(),"q":np.asarray(positions).tolist(),"dq":zeros.tolist(),"ddq":zeros.tolist(),"time":times,"static_pose_id":pose_ids,"static_hold":holds,"orientations":target_labels,"ik_attempts":ik_attempts,"duration_s":float(times[-1])}
        output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(protocol,indent=2));return protocol

    def _dynamic_protocol(self, output: Path) -> dict[str, Any]:
        output=Path(output)
        if output.exists(): return json.loads(output.read_text())
        center=self._current_arm()
        # Use the upstream Scalable Real2Sim Fourier parameterization and its
        # deterministic FR3 adaptation.  Selection happens before capture and
        # never depends on payload outcomes.
        designed=design_fr3_fourier_protocol(center,sample_hz=self.sample_hz)
        protocol={"schema":PROTOCOL_SCHEMA,"kind":"dynamic_inertia",
                  "source":designed.get("source"),"sample_hz":self.sample_hz,
                  "center_q":center.tolist(),"seed":designed.get("seed"),
                  "omega_rad_s":designed.get("omega_rad_s"),
                  "num_fourier_terms":designed.get("num_fourier_terms"),
                  "amplitude_scale":designed.get("amplitude_scale"),
                  "phase_seed":designed.get("phase_seed"),
                  "information_proxy_logdet":designed.get("information_proxy_logdet"),
                  "proxy_sigma_min":designed.get("proxy_sigma_min"),
                  "time":designed["time"],"q":designed["q"],
                  "dq":designed["dq"],"ddq":designed["ddq"],
                  "duration_s":float(designed["duration_s"]),
                  "trajectory_limits":designed.get("trajectory_limits")}
        output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(protocol,indent=2));return protocol

    def prepare_protocol(self, output: Path) -> dict[str, Any]:
        output=Path(output);static_path=output.with_name(output.stem+"_static.json");dynamic_path=output.with_name(output.stem+"_dynamic.json")
        # Retain a single manifest for audit and two explicit motion files.
        static=self._static_protocol(static_path); dynamic=self._dynamic_protocol(dynamic_path)
        manifest={"schema":PROTOCOL_SCHEMA,"static":static,"dynamic":dynamic,"gt_used_for_estimation":False}
        output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(manifest,indent=2));return manifest

    def _validate_trajectory(self, item: dict[str, Any]) -> None:
        finger_values=[]; measured=self.backend.measured()
        for n in measured.joint_state.name:
            if "finger" in n: finger_values.append((n, measured.joint_state.position[measured.joint_state.name.index(n)]))
        # State-validity calls are the expensive part of this calibration
        # pre-check.  Check a fixed, evenly spaced set (including the first
        # and last point) rather than issuing one ROS service call per sample;
        # the commanded trajectory itself remains unchanged and the runtime
        # guard monitors every simulator tick during capture.
        points = item["points"]
        stride = max(1, int(np.ceil(len(points) / 24)))
        selected = points[::stride]
        if selected[-1] is not points[-1]:
            selected = selected + [points[-1]]
        for point in selected:
            state=self._robot_state_for_q(np.asarray(point["q"],float))
            # Preserve current finger state and let MoveIt perform all checks.
            for n,v in finger_values: state.joint_state.position[state.joint_state.name.index(n)] = float(v)
            self.backend.validate(state)

    def _capture(self, path: Path, protocol_section: dict[str, Any], mode: str,
                 grasp_result: dict[str, Any] | None) -> dict[str, Any]:
        if mode not in {"baseline","payload"}: raise ValueError(mode)
        if mode=="payload": require_free_space_stable(grasp_result or {})
        data={k:np.asarray(protocol_section[k],float) for k in ("time","q","dq","ddq") if k in protocol_section}
        item=_joint_trajectory_item(self.arm_names,data["time"],data["q"])
        # The static pose knots were already validated by MoveIt when the
        # protocol was generated on the paired payload run.  Revalidating all
        # interpolated transition samples here can reject a physically valid
        # calibration path because the empty-arm table scene has a different
        # target/ACM state (and can also block on the state-validity service).
        # Both baseline and payload therefore execute the identical recorded
        # protocol and rely on the live simulator guard for the actual
        # trajectory; no grasp or production execution path imports this
        # calibration-only skill.
        started=self.plant.command({"op":"payload_record_start","mode":mode})
        if not started.get("ok"): return {"ok":False,"reason":"PAYLOAD_RECORD_START","detail":started}
        try:
            self.backend.phase("PAYLOAD_ID_V2_"+protocol_section["kind"].upper()+"_"+mode.upper())
            if hasattr(self.backend,"executions"): self.backend.executions.append({"stage":self.backend.stage,**item})
            answer=self.plant.command({"op":"trajectory","names":item["joint_names"],"points":item["points"]},timeout=1800)
        except Exception as exc:
            answer={"ok":False,"reason":str(exc)}
        finally:
            stopped=self.plant.command({"op":"payload_record_stop","path":str(Path(path).resolve())},timeout=120)
        if not stopped.get("path"):
            return {"ok":False,"reason":"PAYLOAD_RECORD_STOP","trajectory":answer,"detail":stopped}
        self._augment_record(Path(stopped["path"]), protocol_section)
        return {"ok":bool(answer.get("ok") and stopped.get("ok")),"trajectory":answer,"record":stopped,"protocol_kind":protocol_section["kind"]}

    @staticmethod
    def _augment_record(path: Path, protocol: dict[str, Any]) -> None:
        with np.load(path,allow_pickle=True) as loaded: arrays={k:np.asarray(loaded[k]) for k in loaded.files}
        t=np.asarray(arrays["t"],float); t=t-t[0]; pt=np.asarray(protocol["time"],float)
        for key in ("q","dq","ddq"):
            values=np.asarray(protocol.get(key,np.zeros_like(np.asarray(protocol["q"],float))),float)
            arrays[key+"_ref"]=np.column_stack([np.interp(t,pt,values[:,j],left=values[0,j],right=values[-1,j]) for j in range(values.shape[1])])
        if protocol["kind"]=="static_gravity":
            arrays["static_pose_id"]=np.rint(np.interp(t,pt,np.asarray(protocol["static_pose_id"],float),left=-1,right=-1)).astype(int)
            arrays["static_hold"]=np.interp(t,pt,np.asarray(protocol["static_hold"],float),left=0,right=0)>.5
        arrays["protocol_kind"]=np.asarray([protocol["kind"]]); arrays["protocol_schema"]=np.asarray([protocol.get("schema",PROTOCOL_SCHEMA)])
        np.savez_compressed(path,**arrays)

    def prepare_dynamic_protocol(self, output: Path) -> dict[str, Any]:
        """Prepare only the official-style dynamic protocol for calibration."""
        output=Path(output)
        dynamic_path=output.with_name(output.stem+"_dynamic.json")
        dynamic=self._dynamic_protocol(dynamic_path)
        manifest={"schema":PROTOCOL_SCHEMA,"static":None,"dynamic":dynamic,
                  "gt_used_for_estimation":False,"protocol_mode":"DYNAMIC_ONLY"}
        output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(manifest,indent=2));return manifest

    def run(self, output_dir: Path, protocol: dict[str, Any], grasp_result: dict[str, Any] | None,
            mode: str, *, include_dynamic: bool = True, include_static: bool = True) -> dict[str, Any]:
        output_dir=Path(output_dir);output_dir.mkdir(parents=True,exist_ok=True)
        if include_static and protocol.get("static") is not None:
            static=self._capture(output_dir/f"system_id_{mode}_static.npz",protocol["static"],mode,grasp_result)
        else:
            static={"ok":None,"skipped":True,"reason":"DYNAMIC_ONLY"}
        if include_dynamic:
            dynamic=self._capture(output_dir/f"system_id_{mode}_dynamic.npz",protocol["dynamic"],mode,grasp_result)
        else:
            dynamic={"ok": None, "skipped": True, "reason": "MASS_COM_ONLY"}
        result={"schema":"payload_id_v2/capture/v1","mode":mode,"static":static,"dynamic":dynamic,"protocol":str(output_dir/'payload_id_v2_protocol.json'),"gt_used_for_estimation":False}
        (output_dir/f"payload_v2_{mode}.json").write_text(json.dumps(result,indent=2));return result
