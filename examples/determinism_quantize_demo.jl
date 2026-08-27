#!/usr/bin/env julia
"""
Demonstrates why raw Float64 trajectory hashes are NOT cross-arch portable,
and that keyframe quantization makes them robust.

Simulates a flight with the hand-rolled integrator, then perturbs every
position by ~1e-9 (the size of last-ulp libm/FMA drift across CPU
architectures). The raw SHA-256 changes; the quantized hash (1e-3 m position
tolerance) does not.

Run: julia --project=.. examples/determinism_quantize_demo.jl
"""

const _ROOT = joinpath(@__DIR__, "..")

using StaticArrays, SHA, JSON
include(joinpath(_ROOT, "src", "dynamics.jl"))
include(joinpath(_ROOT, "src", "validator.jl"))

function ctrl(s)
    t = s.t
    thr = 0.5 + 0.03 * sin(0.7 * t)
    roll = 0.04 * sin(1.3 * t)
    pitch = 0.04 * cos(1.1 * t)
    yaw = 0.02 * sin(0.5 * t)
    return SVector(clamp(thr + pitch + roll + yaw, 0, 1), clamp(thr + pitch - roll - yaw, 0, 1),
                   clamp(thr - pitch - roll + yaw, 0, 1), clamp(thr - pitch + roll - yaw, 0, 1))
end

function perturb(cp::TrajectoryCheckpoint, eps::Float64)
    return TrajectoryCheckpoint(
        cp.t, cp.position .+ eps, cp.attitude, cp.imu_accel, cp.imu_gyro,
    )
end

function main()
    states = simulate_scarab(create_scarab_dynamics(), initialize_state(), ctrl, 10.0, 0.01)
    cps = downsample_trajectory(states, 10)

    raw = compute_trajectory_hash(cps)
    qnt = compute_quantized_trajectory_hash(cps)

    # Simulate cross-arch last-ulp drift on the raw Float64 positions.
    eps = 1e-9
    cps_perturbed = [perturb(c, eps * (i % 3 + 1)) for (i, c) in enumerate(cps)]

    raw_p = compute_trajectory_hash(cps_perturbed)
    qnt_p = compute_quantized_trajectory_hash(cps_perturbed)

    println("raw hash            = ", raw)
    println("raw hash (perturbed)= ", raw_p)
    println("raw changed under 1e-9 drift:        ", raw != raw_p)
    println("quantized hash            = ", qnt)
    println("quantized hash (perturbed)= ", qnt_p)
    println("quantized INVARIANT under 1e-9 drift: ", qnt == qnt_p)
end

main()
