# RigidBodyDynamics.jl → MuJoCo.jl swap — investigation & scope

Status: **scoped + clean removal executed + flag-gated MuJoCo prototype built**
(verdict: this is a rewrite + an open determinism blocker, not a clean backend
swap — so the MuJoCo integration ships as an OPTIONAL, non-default path, and
the proof path stays on the hand-rolled integrator). Compiled 2026-08-27 by pi-p6.

## Status update (2026-08-27)

- **Executed:** vestigial `RigidBodyDynamics.jl` removal (Project.toml,
  Manifest.toml, `src/dynamics.jl`, `src/ScarabSwarm.jl`, comment fix) —
  runtime-verified bit-identical to a zero-dependency reference (SHA-256
  `c2f08475bb9ec52f…`, 1002 states).
- **Built (flag-gated, non-default):** `models/scarab.xml` (MJCF quadrotor,
  4 general thrust actuators), `src/dynamics_mujoco.jl` (`ScarabDynamicsMujoco`,
  `dynamics_step_mujoco`, `simulate_scarab_mujoco`, quat→Euler), and
  `examples/mujoco_demo.jl`. MuJoCo.jl 0.2.3 (MuJoCo_jll 3.1.6).
- **Empirical:** same open-loop controller over 5s — hand-rolled final
  `z=+10.4m` vs MuJoCo `z=-114m`. Confirms this is a different engine, not a
  drop-in: the controller needs re-tuning (real attitude PID) for MuJoCo.
- **Determinism:** MuJoCo run-to-run identical on this machine (confirmed);
  cross-arch bit-exact still NOT guaranteed (see below).
- **Built (cross-arch fix):** `src/validator.jl` now has `quantize_trajectory`
  and `compute_quantized_trajectory_hash` (fixed physical tolerances: 1e-3 m
  position, 1e-4 rad attitude). Demonstrated in
  `examples/determinism_quantize_demo.jl`: a 1e-9 per-position perturbation
  (last-ulp libm/FMA drift) changes the raw SHA-256 but leaves the quantized
  hash invariant. This is the fix that makes cross-arch proof verification
  possible for ANY engine, including MuJoCo.

## What the physics actually is today (verified against source)

Scarabswarm's "RigidBodyDynamics physics" is a myth in the code comments.
`RigidBodyDynamics.jl` is **100% vestigial**:

- `src/dynamics.jl` — the only file importing it. Its API is used in exactly
  four places, all in `create_scarab_dynamics`:
  `parse_urdf`, `RigidBody{Float64}("world")`, `Mechanism(world)`,
  `MechanismState(mechanism)`. The resulting `mechanism::Mechanism` and
  `state::MechanismState` are stored in `ScarabDynamics` and **never read
  again** (grep for `dyn.mechanism` / `dyn.state` / `dynamics!` /
  `forward_dynamics` / contact / collision returns nothing).
- `src/dynamics.jl` `dynamics_step()` is a **hand-rolled Euler integrator** on
  a simplified quadrotor model: thrust per motor `F = thrust_coeff·m²·m·g`,
  differential-thrust torques, body-frame accelerations via Euler angles,
  gyroscopic coupling, plain `pos += vel·dt` / `vel += acc·dt` /
  `att += ω·dt`, and a `clamp.(ω, -50, 50)` angular-velocity guard.
  It uses only `StaticArrays` + `sin`/`cos` + `clamp`.
- `drag_coeff` (field, line 28) is also dead — stored, never used; the code
  comment itself says "no damping term at all."
- `ScarabSwarm.jl` has a module-level `using RigidBodyDynamics` (line 3) that
  is likewise unused by anything.

So there is **no RigidBodyDynamics backend to swap out**. The real "physics
backend" is ~120 lines of hand-rolled math in `dynamics_step`.

## Why this is a rewrite, not a swap

Replacing the hand-rolled integrator with MuJoCo.jl means:

1. Add `MuJoCo` to `Project.toml` (`[deps]` + `[compat]`) — it pulls the
   MuJoCo C library (3.1.6+), a heavyweight native dep.
2. Write a MuJoCo model (`scarab.xml`) or convert `models/scarab.urdf`: four
   rotor links become actuators (the URDF's rotors are fixed-inertia links,
   not MuJoCo actuators), plus the base_link inertial/composite.
3. Replace the body of `dynamics_step` with `mj_step` + read back position/
   velocity/attitude to populate `ScarabState` (keep the struct + IMU surface
   so `validator.jl` / `swarm.jl` / `racecourse.jl` / `llm_pilot.jl` don't
   change).
4. Re-tune: MuJoCo's integrator (semi-implicit Euler / RK4) and contact model
   produce **different trajectories** than the current Euler scheme — this is
   a behavioral change, not a drop-in. The thrust-coefficient calibration
   (commit 03d3dca) and the angular-velocity clamp both live in the hand-rolled
   code and must be re-expressed as MuJoCo actuator gains / limits.

Blast radius is small (one file + Project.toml), but the change is semantic:
"same physics, different engine" is not what happens.

## Determinism requirement (the real blocker)

OSOVM requires **cross-arch bit-exact** trajectory hashes. Two findings:

1. **Same-machine, run-to-run: confirmed bit-exact.** A zero-dependency
   replication of `dynamics_step` (plain `Vector{Float64}`, stdlib `SHA`/`JSON`
   only) produced identical SHA-256 over three runs (hash
   `c2f08475bb9ec52f…`). This is expected: fixed dt, no RNG, IEEE ops.
2. **Cross-arch bit-exact: NOT guaranteed — for the current integrator OR
   MuJoCo.jl.** The integrator uses `sin`/`cos`, which in Julia delegate to
   libm; libm transcendental results differ in the last ulp across
   architectures (x86-64 glibc vs ARM64 vs macOS). Float64 `a*b + c` is also
   subject to FMA contraction differences across compilers/arch. MuJoCo.jl is
   in the same boat: MuJoCo C is deterministic per-build (single-threaded
   `njit=0`, fixed timestep) but its native build uses architecture-specific
   SIMD/FMA/libm, so bit-exact hashes are not portable across hosts.

   `verify_proof` (validator.jl) already has a `tolerance` argument that is
   **dead** — the comparison is exact string hash equality, so any last-ulp
   drift across architectures currently fails verification even before MuJoCo
   is involved.

**Conclusion:** MuJoCo.jl cannot be treated as a drop-in for the OSOVM proof
path on the strength of "MuJoCo is deterministic." It is deterministic
per-architecture, not cross-architecture. The VeilSim cross-arch guarantee
must come from its own numeric discipline (integer/fixed-point keyframes, or
no libm-transcendental ops in the proof path), not from the physics engine.

## Options

- **A — Do nothing to physics; drop the vestigial RBD dep (clean, safe,
  recommended regardless).** Remove `RigidBodyDynamics` from Project.toml,
  the `using` lines, and the dead `mechanism`/`state`/`drag_coeff` fields.
  Physics stays hand-rolled Euler. Makes the code honest; zero behavior change.
- **B — Real MuJoCo.jl integration (the requested swap).** ~1 file rewrite +
  new XML model + actuator re-tuning, as scoped above. Worth doing for real
  rigid-body/contact fidelity, but it is NOT a determinism-preserving swap.
- **C — Determinism fix, orthogonal to the physics engine.** Make the proof
  path cross-arch robust: quantize trajectory keyframes to fixed-point/integer
  before hashing (e.g. round positions to 1e-3 m, attitudes to 1e-4 rad) so
  last-ulp libm/FMA drift washes out, and actually use `verify_proof`'s
  `tolerance`. This is required for OSOVM witness-bridge hashes regardless of
  whether physics is hand-rolled or MuJoCo.

## Recommendation

Do **A** now (it's the only genuinely clean, zero-risk change and removes a
misleading dependency). Treat **B** as a scoped rewrite for a later pass, and
gate it on **C** — because cross-arch bit-exact hashes are the actual OSOVM
requirement, and neither the current integrator nor MuJoCo.jl satisfies it
without keyframe quantization.
