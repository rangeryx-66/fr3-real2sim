# Independent Franka Hand physics calibration

No AnyGrasp inference, score, pose, selection or mesh gate is used or modified.
The original execution source remains frozen. All changes are isolated here.

Four original Arena assets: soup, banana, bowl, mug. Each has one GT mesh-defined
pose and cached arm path, fixed before its formal trials. Rim grasps on bowl/mug
align the closing axis with the local wall normal; banana uses its centerline.
The robot is reset at pregrasp before each trial; the object is never welded.

Force denotes TOTAL measured contact-normal force of both fingers (10,20,30,40,
60 N). A common PI feedback controls both coupled prismatic joints, with finger
position gains disabled. Before contact it closes slowly; after first contact
it ramps force over 0.4 s. Individual normal forces and efforts are logged.

Pad static/dynamic friction: 0.3,0.5,0.7,1.0, restitution zero. Target friction
is 1.0 and the combine mode is multiply, yielding the requested pad-target
coefficient. Table friction remains 0.7. Runtime PhysX tensor readback verifies
pad changes; modifying USD alone was found insufficient during preflight.

Each of 20 force/friction combinations gets 3 repeats per asset. Another 3
repeats per asset use the original position controller at explicit friction
0.7. Cold-start friction blocks 0.7,0.3,1.0,0.5; fixed shuffled force/repeat order seed 20260910 within each block. Total 252 formal trials. Infrastructure
preflights are stored separately and excluded from formal rates.

Sequence: approach, close, 0.5 s hold, 5 mm micro lift, 0.2 s settle, stability
gate, 10 cm additional lift, 2.2 s hold. Gate: relative TCP translation <=2 mm,
rotation <=3 degrees, no bilateral-contact interruption >0.05 s and bilateral
contact at the end. Rejection is UNSTABLE_GRASP; no final lift is commanded.
Success retains >=8 cm elevation, >=2 s hold, bilateral contact >0.1 N,
and <=1 cm hold height range. No acceptance threshold is relaxed.

A repeatable force-unreached or gate-rejected trial counts as failure. Setup
errors or approach precontact stop the run for diagnosis rather than becoming
valid measurements. Fixed-grasp results only establish performance for these
four poses, not all possible grasps or all household geometries.

Live PhysX material switching failed repeatable readback checks. Formal trials
therefore cold-start each friction block with explicit material bindings and
verify runtime properties each episode. Aborted warm-switch attempts are
retained separately as infrastructure diagnostics. Force convergence uses the
controlled total normal force (within 20% or 2 N), with bilateral contact; it
does not demand equal left/right force while the target is table-supported.
