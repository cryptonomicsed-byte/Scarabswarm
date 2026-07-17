#!/usr/bin/env julia
"""
LLM-Piloted Scarab Demo — the real "LLM Drone" starting point.

Queries a live local Ollama model (llama3.2:3b) for flight decisions
every 10 sim steps (throttled — LLM inference ~100s of ms, flight loop
~2ms), same real RigidBodyDynamics physics as race_demo.jl, no
simulated/faked responses.

IMPORTANT DETERMINISM NOTE, not addressed anywhere in the original
code: LLM sampling is not guaranteed deterministic even at temperature
0 (backend batching, hardware nondeterminism in matmul reduction order
can still vary token probabilities at the margin) -- so a trajectory
flown by an LLM pilot is NOT expected to hash-match on re-run the way
the pure-physics ScarabSwarm and MuJoCo tests are. This is the same
class of problem already flagged for NVIDIA Cosmos: a non-reproducible
decision source must never sit in the sim-regime PROOF path. An LLM-
piloted flight can still be METERED (its physics recorded, energy
counted, gates passed scored) but the resulting hash should NOT be
treated as a determinism-verifiable "proof of simulation" the way the
naive-controller race is. If this needs to be proof-bearing later, the
fix is querying the LLM once to produce a fixed decision *policy*
(e.g., a lookup table or small deterministic function), then replaying
that policy deterministically -- not re-querying the LLM live inside
the proof-verified loop.

Run: julia --project=.. examples/llm_drone_demo.jl
"""

push!(LOAD_PATH, "../src")
using ScarabSwarm
using StaticArrays

function main()
    println("🤖✈️  LLM-PILOTED SCARAB DEMO 🤖✈️\n")

    course = create_standard_course()
    println("Course: $(length(course.gates)) gates")

    # The real sovereign kernel (omokoda-core, :7777) as pilot -- not a raw
    # model call. This is the universal-copilot seam: ANY agent that can
    # take a text prompt and return a text decision can pilot, not just
    # whichever LLM happens to be configured. Verified live: Ọmọ Kọ́dà
    # answered a real piloting decision via her own /v1/think in ~11s.
    pilot = create_llm_pilot("localhost:7777", ""; backend=:omokoda)
    println("Pilot: $(pilot.backend) @ $(pilot.host)\n")

    dyn = create_scarab_dynamics()
    init_state = initialize_state(0.0, SVector(0.0, 0.0, 1.0))
    controller = create_llm_controller(pilot, course, 10)

    println("Flying 10s (LLM queried every 10 steps)...\n")
    @time states = simulate_scarab(dyn, init_state, controller, 10.0)

    score = compute_race_score(states, course)
    proof, checkpoints = compute_proof(states, 10.0)

    println("\n=== RESULT ===")
    println("Gates passed: $(score["gates_passed"])/$(length(course.gates))")
    println("Final position: $(round.(states[end].position, digits=2))")
    println("Trajectory hash: $(proof.trajectory_hash[1:16])... (LLM-driven, not expected to reproduce -- see note above)")
    println("Checkpoints: $(proof.checkpoint_count)")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
