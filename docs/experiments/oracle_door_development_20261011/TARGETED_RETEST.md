# One targeted retest — shared closure command continuity

45415 development_01 exhausted the unchanged initial closure window without FORCE_HOLD. The correct mesh was physically touched by both fingers, but raw peaks remained below 0.094 N. No opening was credited and no joint/speed protection was triggered.

The archived 240 Hz commands show **188 aperture command increases** after initial contact. In the final seconds a CENTER transition repeatedly copied measured, contact-deflected opening back into the position command: typically a +0.103–0.108 mm reopening. This unloaded the contact, left filtered loads straddling the existing 0.05 N threshold, and repeated instead of attaining bilateral preload.

The only correction for the single allowed targeted retest is to retain the existing closing command during unilateral centering: `self.opening = min(self.opening, measured_opening)`. Centering motion, thresholds, sample windows, gains, clamp effort, 100 s closure budget, collision and all existing physical protections remain unchanged. The runner installs this one-line change in its own execution namespace; frozen closure sources remain byte-identical.

**Retest 45415 from a new normal closed state with the exact same selected geometric plan and station as development_01.** No pose tuning or re-planning. This isolates the implementation item. It is a shared correction, not an object-specific controller branch. Only this case is retested; no more trials follow.
