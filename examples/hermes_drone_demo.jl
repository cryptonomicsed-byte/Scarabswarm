#!/usr/bin/env julia
"""
Hermes-Piloted Scarab Demo — the universal-copilot seam, third real
backend after :openai (OmniRoute) and :omokoda (the sovereign kernel).

Invokes a live Hermes Agent container (`hermes chat -q ... -Q`, Claude
Sonnet 4.6 via an already-authenticated Nexos.ai session) for flight
decisions. No HTTP: Hermes's web dashboard needs cookie auth and its
OpenAI-compatible `proxy` needs an interactive OAuth login, neither
scriptable headlessly -- `hermes chat` is the real scriptable surface.

Verified live before wiring: a real query with the exact piloting
prompt format returned a correctly-formatted decision
("THROTTLE:0.6 ROLL:-0.1 PITCH:0.8 YAW:0.0") in ~21s round-trip
(docker exec overhead + real Claude Sonnet 4.6 inference).

Hermes's per-call latency (~21s, an agentic tool-capable loop, not a
bare completion) is higher than :omokoda's /v1/think (~11s) or
:openai/OmniRoute (~2s) -- this demo deliberately uses a SHORT flight
(3s sim time, queried every 20 steps = 15 real Hermes calls, ~5 min
wall clock) to prove the wiring works, not to fly a full race. Same
determinism caveat as llm_drone_demo.jl applies: agent decisions are
not reproducible, so this trajectory's hash is metered, not
proof-bearing.

Run: julia --project=.. examples/hermes_drone_demo.jl
"""

push!(LOAD_PATH, "../src")
using ScarabSwarm
using StaticArrays

function main()
    println("⚕✈️  HERMES-PILOTED SCARAB DEMO ✈️⚕\n")

    course = create_standard_course()
    println("Course: $(length(course.gates)) gates")

    pilot = create_llm_pilot("hermes-agent-h0lk-hermes-agent-1", ""; backend=:hermes)
    println("Pilot: $(pilot.backend) @ container $(pilot.host)\n")

    dyn = create_scarab_dynamics()
    init_state = initialize_state(0.0, SVector(0.0, 0.0, 1.0))
    controller = create_llm_controller(pilot, course, 20)

    println("Flying 3s (Hermes queried every 20 steps -- ~15 real calls, ~5 min expected)...\n")
    @time states = simulate_scarab(dyn, init_state, controller, 3.0)

    score = compute_race_score(states, course)
    proof, checkpoints = compute_proof(states, 3.0)

    println("\n=== RESULT ===")
    println("Gates passed: $(score["gates_passed"])/$(length(course.gates))")
    println("Final position: $(round.(states[end].position, digits=2))")
    println("Trajectory hash: $(proof.trajectory_hash[1:16])... (agent-driven, not expected to reproduce)")
    println("Checkpoints: $(proof.checkpoint_count)")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
