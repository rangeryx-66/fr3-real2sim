# Explicitly authorized orchestration wall credit

Status: offline regression PASS; **not used in the current B02 physical
experiment**. B02 remains an unchanged `a8c8d1a` run and its closed record is not
reopened. This is a new-start frontend only; no actor was launched.

Branch `codex/v83-orchestration-extension` starts at
`a8c8d1ac75c4f12bfe013928969227e4953b5586`. New opt-in profile
`v83_revolute_orchestration_extension_v1` derives the existing revolute
validation profile and retains5Hz private video and the original controls.
Other profiles retain their original timing behavior. The original initial
resource schema still rejects wall budgets above7200s.

## Launch schema

Use the existing V8.1 launcher with a unique episode and separate output root.
Retain scene, physics commitment, environment, shared actor lock, initial
resource values and original exit reserves. Add:

```json
{
  "integration_profile": "v83_revolute_orchestration_extension_v1",
  "orchestration_authorization": {
    "human_authorization_ref": "human:2026-10-25/continue-while-promising",
    "max_total_extra_wall_s": 3600,
    "parked_decision_renewal_s": 600
  }
}
```

The reference must identify the actual human authorization in the registration.
It is provenance under the trusted operator/tool model, not cryptographic proof
of human identity. The total is an explicit finite integer cap1–86400s; each
increment is1–1800 integer seconds. `parked_decision_renewal_s` defaults to0,
disabled. A positive value explicitly authorizes renewal of an otherwise
expiring parked Agent wait, consuming the same wall-credit cap. Standing renewal
can consume authorized credit while waiting for a command; it does not establish
physical feasibility or authorize interaction.

## Gates and timing

Only the new profile installs `orchestration_extension.py` at actor construction.
Acceptance requires verified native parking, released jaws, unloaded pads, no
grasp reference, no latch, settled force history, stationary robot/object,
finite measured positions/velocities/wrench/TCP and valid FT. Original station
and parking checks run again without stepping or moving. Closed/stopped actors,
stale manual requests and loaded/untrusted states cannot receive credit.

Manual operation `extend_orchestration_wall` requires the exact `binding` from
the decision packet (episode/source/actor/session nonce), a fresh32-hex
`request_nonce`, the matching registered human reference, `delta_wall_s`, and a
reason. All other field changes are refused. Existing transport still rejects
expired manual publication.

At parked wait expiry, a positive standing policy rechecks the same gates and
can add exactly its delta before closing. It cannot revive closed episodes.
Insufficient credit, missing park evidence or failed gates retain the original
timeout/exit and record the refusal. Neither actor elapsed time nor the current
waiting-clock origin is reset:

`remaining = max(0, original_wall + approved_total - original_elapsed - exit_wall)`.

Credit is not added to an already-clamped remaining value. Simulation time,
acquisition limits, original600s decision duration, exit300/120s reserves and
all force/collision/joint/lead/actuator/sensor protections remain unchanged.
Standing credit adds finite seconds to the current wait and synchronizes the
waiting/status deadlines and decision packet. Manual acceptance lets the next
ordinary decision use effective wall resources.

## Launcher agreement and audit

Accepted records publish once atomically via an exclusive final hard link. They
bind the unique request, cumulative credit, original budget/start/reserves,
human reference/reason and native evidence. A hash chain is authenticated with
an actor/launcher session HMAC key absent from job files, commands and reports.
The launcher polls against **original launcher deadline + authenticated credit**.
It does not keep a fixed old timeout that would kill an extended actor.

Invalid, replayed or forged records cannot grant credit. On ledger faults the
launcher records the error, retains already verified credit and remains owner
of the original timeout/cleanup. It does not abandon the actor. This is an
auditable frontend capability, not an OS security boundary.

Current B02 never installed this seam. No B02 source, model or record changed;
editing files, injection, hot reload or reopening that closed actor is not a
supported path. A new trial must record this source SHA and new profile.

## Verification

-13 new tests PASS: consumption after the original budget clamps; finite
  renewal at the task floor; human reference and all identity fields; nonce
  replay; single/cumulative limits; loaded/hard/closed/nonfinite refusals;
  fresh native-park/raw-FT checking; immutable authenticated ledger; launcher
  agreement and default timeout.
-Actual session wait-loop test with offline fake clock: authorized verified
  park renews600→1200s and finishes at650s; default profile exits at600s.
  Decision packet and waiting deadlines agree.
-172 existing V8.1 tests PASS (1 existing skip);13 V8 packaging/Physics-ID tests
  PASS. Baseline audit:606 physical,71 revolute and956 release sources match;
  no changed frozen paths or failures.

These are orchestration/software tests, not physical recovery or independent
Physics-ID validation evidence. No physical trial was started for this change.
