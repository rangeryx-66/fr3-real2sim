# Independent forward integration

Before opening any hidden dynamics, the final provisional fit had I=1.7991e-6 kg m² and b=0.89524 Nm s/rad; both are unobservable. At dt=1/240s, b*dt/I is about2073. Explicit velocity Euler is numerically unstable for this positive, passive model. Numerical instability must not be mistaken for a measured physical-model failure.

Use the same passive backward-Euler Coulomb inclusion for ALL saved models, without changing fitted parameters:

`v_next = soft_threshold(I*v + dt*tau, dt*tau_c) / (I + dt*b)`

`q_next = q + dt*v_next`.

Input is measured wrist-derived effort and the held-out initial state only. Following GT states are used solely for error evaluation. No GT dynamics or trajectory forcing enters integration. This is an offline numerical solver correction, not a controller/physics change or fit tuning. Original evaluator source remains in Git history. Near-degenerate inertia, parameter uncertainty and observability are reported even if one slow segment happens to predict well; such a result does not identify inertia or validate a full physical twin.
