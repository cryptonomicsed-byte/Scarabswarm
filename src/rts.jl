# Reality Transfer Score — sim→physical bridge
# Quantize both trajectories, compute cosine similarity, gate at 0.6

using LinearAlgebra

const MIN_RTS_FOR_PROOF = 0.6
const MIN_RTS_FOR_EXCELLENT = 0.85

struct RealityTransferScore
    flight_receipt_id::String
    sim_proof_id::Union{String, Nothing}
    rts::Float64                    # cosine similarity [0, 1]
    mission_transfer::Float64       # fraction of gates passed in physical
    physical_proof_eligible::Bool   # rts >= 0.6
    timestamp_utc::Int64
end

function cosine_similarity(a::Vector{Float64}, b::Vector{Float64})::Float64
    na = norm(a)
    nb = norm(b)
    if na == 0.0 || nb == 0.0
        return 0.0
    end
    raw = dot(a, b) / (na * nb)
    return clamp(raw, 0.0, 1.0)
end

function checkpoint_to_flat(c::TrajectoryCheckpoint)::Vector{Float64}
    # Concatenate position and attitude as a flat Float64 vector
    return [c.position[1], c.position[2], c.position[3],
            c.attitude[1],  c.attitude[2],  c.attitude[3]]
end

function flatten_trajectory(checkpoints::Vector{TrajectoryCheckpoint})::Vector{Float64}
    flat = Float64[]
    for c in checkpoints
        append!(flat, checkpoint_to_flat(c))
    end
    return flat
end

"""
    compute_rts(sim_checkpoints, physical_checkpoints, flight_receipt_id;
                sim_proof_id=nothing) -> RealityTransferScore

Compare quantized sim and physical trajectories via cosine similarity.
`rts >= MIN_RTS_FOR_PROOF` (0.6) marks the flight as physical-proof-eligible.
"""
function compute_rts(sim_checkpoints::Vector{TrajectoryCheckpoint},
                     physical_checkpoints::Vector{TrajectoryCheckpoint},
                     flight_receipt_id::String;
                     sim_proof_id::Union{String, Nothing}=nothing)::RealityTransferScore

    # Quantize both to cross-arch-stable keyframes
    q_sim  = quantize_trajectory(sim_checkpoints)
    q_phys = quantize_trajectory(physical_checkpoints)

    # Trim to the shorter length so vectors are the same size
    n = min(length(q_sim), length(q_phys))
    q_sim  = q_sim[1:n]
    q_phys = q_phys[1:n]

    flat_sim  = flatten_trajectory(q_sim)
    flat_phys = flatten_trajectory(q_phys)

    rts_val = cosine_similarity(flat_sim, flat_phys)

    # mission_transfer: did the physical drone reach the final sim position?
    # Simplified: check whether the last physical checkpoint is within 20% of
    # the total course "length" (measured as the distance from first to last
    # sim checkpoint).
    mission_transfer = if n >= 2
        sim_start    = q_sim[1].position
        sim_end      = q_sim[end].position
        course_len   = norm(sim_end - sim_start)
        phys_end     = q_phys[end].position
        dist_to_goal = norm(phys_end - sim_end)
        if course_len > 0.0 && dist_to_goal <= 0.2 * course_len
            1.0
        else
            # Use max z-position proxy: fraction of sim's peak z reached
            max_z_sim  = maximum(c.position[3] for c in q_sim)
            max_z_phys = maximum(c.position[3] for c in q_phys)
            max_z_sim > 0.0 ? clamp(max_z_phys / max_z_sim, 0.0, 1.0) : 0.0
        end
    else
        0.0
    end

    eligible = rts_val >= MIN_RTS_FOR_PROOF

    return RealityTransferScore(
        flight_receipt_id,
        sim_proof_id,
        rts_val,
        mission_transfer,
        eligible,
        round(Int64, time())
    )
end

function rts_to_dict(rts::RealityTransferScore)
    Dict("flight_receipt_id"      => rts.flight_receipt_id,
         "sim_proof_id"           => rts.sim_proof_id,
         "rts"                    => rts.rts,
         "mission_transfer"       => rts.mission_transfer,
         "physical_proof_eligible"=> rts.physical_proof_eligible,
         "timestamp_utc"          => rts.timestamp_utc)
end

"""
    test_rts_self_similarity() -> Bool

Self-similarity smoke test: comparing a trajectory to itself should yield RTS = 1.0.
"""
function test_rts_self_similarity()
    # Generate 10 fake checkpoints moving along the x-axis
    checkpoints = [
        TrajectoryCheckpoint(
            Float64(i) * 0.1,
            SVector(Float64(i) * 0.5, 0.0, 1.0),
            SVector(0.0, 0.0, 0.0),
            SVector(0.0, 0.0, 9.81),
            SVector(0.0, 0.0, 0.0)
        )
        for i in 1:10
    ]

    rts = compute_rts(checkpoints, checkpoints, "test-self-sim")
    result = rts.rts ≈ 1.0
    println("RTS self-similarity test: rts=", rts.rts, " pass=", result)
    return result
end
