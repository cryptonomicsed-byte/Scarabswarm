# Veil challenge runner — load veil, parameterize simulation, compute F1

using JSON
using SHA

const VEILS_PATH = joinpath(@__DIR__, "../../veilsim-studio/data/veils_1_200.json")

struct VeilResult
    veil_id::Int
    veil_name::String
    f1_score::Float64
    gates_passed::Int
    gates_total::Int
    trajectory_proof::TrajectoryProof
    env_hash::String       # SHA256 of veil_id + seed + params
    proof_eligible::Bool   # f1_score >= veil.min_score
end

"""
    run_veil_challenge(veil_number; seed=42, duration=30.0) -> VeilResult

Load veil `veil_number` from `veils_1_200.json`, build a parameterized course,
run the naive-controller simulation, compute the trajectory proof, score the
run, and return a `VeilResult`.
"""
function run_veil_challenge(veil_number::Int;
                            seed::Int=42,
                            duration::Float64=30.0)::VeilResult

    # ── 1. Load veil definition ──────────────────────────────────────────────
    if !isfile(VEILS_PATH)
        error("veils_1_200.json not found at: $VEILS_PATH")
    end

    veils_list = JSON.parsefile(VEILS_PATH)

    veil_idx = findfirst(v -> get(v, "veil_number", nothing) == veil_number, veils_list)
    if isnothing(veil_idx)
        error("Veil $veil_number not found in $(VEILS_PATH)")
    end
    veil = veils_list[veil_idx]

    # ── 2. Extract veil parameters ───────────────────────────────────────────
    min_score  = Float64(get(veil, "min_score",  0.777))
    tier_gate  = get(veil, "tier_gate",  "T1")
    difficulty = Float64(get(veil, "difficulty", 0.5))
    veil_name  = get(veil, "name", "veil_$(veil_number)")

    # ── 3. Build parameterized course ────────────────────────────────────────
    # Harder veils → tighter gate spacing (multiplier shrinks toward 0.5)
    spacing_mult = 1.0 - 0.5 * difficulty   # difficulty=0 → 1.0×; difficulty=1 → 0.5×
    base_spacing = 5.0                       # same as create_standard_course()
    sp = base_spacing * spacing_mult

    gates = [
        Gate(SVector(0.0,       0.0,  1.0), SVector(1.0,  0.0,  0.0), 0.5, 0.5, 1),
        Gate(SVector(sp,        0.0,  1.0), SVector(1.0,  0.0,  0.0), 0.5, 0.5, 2),
        Gate(SVector(2sp,  0.4*sp,   1.0), SVector(1.0,  0.5,  0.0), 0.5, 0.5, 3),
        Gate(SVector(3sp,       0.0,  1.0), SVector(1.0, -0.5,  0.0), 0.5, 0.5, 4),
        Gate(SVector(4sp,       0.0,  1.0), SVector(1.0,  0.0,  0.0), 0.5, 0.5, 5),
    ]

    start_pos  = SVector(0.0,   0.0, 0.5)
    finish_pos = SVector(4sp,   0.0, 1.0)
    course = RaceCourse(gates, SVector(0.5, 0.0, 0.0),
                        (start_pos, 1.0), (finish_pos, 1.5))

    # ── 4. Scarab dynamics + initial state ───────────────────────────────────
    dyn   = create_scarab_dynamics()
    state = initialize_state()

    # ── 5. Run simulation ────────────────────────────────────────────────────
    naive_ctrl = create_naive_controller(course)
    states     = simulate_scarab(dyn, state, naive_ctrl, duration)

    # ── 6. Compute trajectory proof ──────────────────────────────────────────
    proof, _checkpoints = compute_proof(states, duration)

    # ── 7. Score the run ─────────────────────────────────────────────────────
    score = compute_race_score(states, course)

    # ── 8. Environment hash ──────────────────────────────────────────────────
    hash_payload = JSON.json(Dict(
        "veil_id"   => veil_number,
        "seed"      => seed,
        "min_score" => min_score,
    ))
    env_hash = bytes2hex(sha256(hash_payload))

    # ── 9-10. Gates stats + F1 ───────────────────────────────────────────────
    gates_passed = score["gates_passed"]::Int
    gates_total  = length(course.gates)
    f1            = gates_total > 0 ? gates_passed / gates_total : 0.0

    # ── 11. Return ───────────────────────────────────────────────────────────
    return VeilResult(
        veil_number,
        veil_name,
        f1,
        gates_passed,
        gates_total,
        proof,
        env_hash,
        f1 >= min_score,
    )
end

function veil_result_to_dict(r::VeilResult)
    Dict("veil_id"           => r.veil_id,
         "veil_name"         => r.veil_name,
         "f1_score"          => r.f1_score,
         "gates_passed"      => r.gates_passed,
         "gates_total"       => r.gates_total,
         "trajectory_proof"  => proof_to_dict(r.trajectory_proof),
         "env_hash"          => r.env_hash,
         "proof_eligible"    => r.proof_eligible)
end
