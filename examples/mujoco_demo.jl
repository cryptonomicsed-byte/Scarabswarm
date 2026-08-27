#!/usr/bin/env julia
"""
MuJoCo-backed Scarab demo — the OPTIONAL, flag-gated physics backend.

Includes only the two dynamics modules (not the full ScarabSwarm package, so
no HTTP dependency is pulled in just to compare physics). Opts into the
MuJoCo.jl backend for one flight and shows the two engines diverge (different
physics), which is expected and intentional.

DETERMINISM NOTE: MuJoCo is deterministic run-to-run on this machine but NOT
bit-exact across architectures (native SIMD/FMA/libm) — so do NOT feed a
MuJoCo trajectory into compute_proof for the OSOVM witness bridge. See
docs/mujoco-swap-scope.md.

Run: julia --project=.. examples/mujoco_demo.jl   (from repo root)
"""

const _ROOT = joinpath(@__DIR__, "..")

using StaticArrays, SHA, JSON
include(joinpath(_ROOT, "src", "dynamics.jl"))        # ScarabState + hand-rolled Euler
include(joinpath(_ROOT, "src", "dynamics_mujoco.jl")) # MuJoCo backend (loads MuJoCo)

# Deterministic open-loop controller (hover + small sinusoidal attitude).
function ctrl(s)
    t = s.t
    thr = 0.5 + 0.03 * sin(0.7 * t)
    roll = 0.04 * sin(1.3 * t)
    pitch = 0.04 * cos(1.1 * t)
    yaw = 0.02 * sin(0.5 * t)
    m1 = clamp(thr + pitch + roll + yaw, 0.0, 1.0)
    m2 = clamp(thr + pitch - roll - yaw, 0.0, 1.0)
    m3 = clamp(thr - pitch - roll + yaw, 0.0, 1.0)
    m4 = clamp(thr - pitch + roll - yaw, 0.0, 1.0)
    return SVector(m1, m2, m3, m4)
end

function main()
    dyn_e = create_scarab_dynamics()
    s_e = simulate_scarab(dyn_e, initialize_state(), ctrl, 5.0, 0.01)

    xml = joinpath(_ROOT, "models", "scarab.xml")
    dyn_m = create_scarab_dynamics_mujoco(xml)
    s_m = simulate_scarab_mujoco(dyn_m, initialize_state(), ctrl, 5.0, 0.01)

    println("hand-rolled: final pos = ", round.(s_e[end].position, digits=3),
            ", n = ", length(s_e))
    println("mujoco:      final pos = ", round.(s_m[end].position, digits=3),
            ", n = ", length(s_m))
    println("trajectories differ (expected, different engines): ",
            s_e[end].position != s_m[end].position)

    # Run-to-run determinism of the MuJoCo backend alone.
    dyn2 = create_scarab_dynamics_mujoco(xml)
    s_m2 = simulate_scarab_mujoco(dyn2, initialize_state(), ctrl, 5.0, 0.01)
    h1 = bytes2hex(sha256(JSON.json([collect(s.position) for s in s_m])))
    h2 = bytes2hex(sha256(JSON.json([collect(s.position) for s in s_m2])))
    println("mujoco run-to-run hash identical: ", h1 == h2)
end

main()
