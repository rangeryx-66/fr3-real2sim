# Pre-contact infrastructure interruption

The initial actor completed three fresh wrist observations, incremental fusion and automatic planning, then exported safely at the unloaded Agent command-5 boundary after the fixed 120-second wait. No grasp/push command was sent; no contact event occurred. Private episode_00 remains unchanged. This is not a physical failure or consumed manipulation attempt.

Only communication handling is changed: the existing job now specifies a bounded 600-second unloaded Agent wait (default remains 120 seconds). The owner deadline remains 45 minutes with cleanup reserve. No loaded waiting, motion/force policy, scoring or estimator change. The launcher permits exactly one explicit pre-contact restart into episode_01 after verifying completed shutdown, BOUNDED_AGENT_STOP and no physical command/event. It keeps the identical committed private configuration. New observations and decisions are required; previous camera/grasp poses are not executable inputs.
