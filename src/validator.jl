# Trajectory proof-of-execution validation
# Computes deterministic hash of trajectory + IMU data
# Validators re-run sim with same inputs → verify hash match

using SHA
using JSON

struct TrajectoryProof
    trajectory_hash::String          # SHA256 of trajectory keyframes
    imu_hash::String                 # SHA256 of IMU stream sample
    execution_time::Float64          # Wall-clock time (s)
    checkpoint_count::Int            # Number of validated checkpoints
    energy_used::Float64             # Est. compute joules (Pi 5 ~5W)
    timestamp::String                # ISO 8601
end

struct TrajectoryCheckpoint
    t::Float64
    position::SVector{3, Float64}
    attitude::SVector{3, Float64}
    imu_accel::SVector{3, Float64}
    imu_gyro::SVector{3, Float64}
end

function downsample_trajectory(states::Vector, target_frequency::Int=10)
    """
    Extract keyframes at target frequency (default 10 Hz).
    Reduces proof size from 1000s to 100s of points.
    """
    if isempty(states)
        return TrajectoryCheckpoint[]
    end
    
    dt = states[2].t - states[1].t
    sample_interval = Int(ceil(1.0 / (target_frequency * dt)))
    
    checkpoints = TrajectoryCheckpoint[]
    for i in 1:sample_interval:length(states)
        s = states[i]
        push!(checkpoints, TrajectoryCheckpoint(
            s.t, s.position, s.attitude, s.imu_accel, s.imu_gyro
        ))
    end
    
    return checkpoints
end

function compute_trajectory_hash(checkpoints::Vector{TrajectoryCheckpoint})
    """
    Deterministic hash: JSON serialize checkpoints, SHA256.
    Replicable by validators.
    """
    json_str = json(Dict(
        "checkpoints" => [Dict(
            "t" => c.t,
            "pos" => [c.position[i] for i in 1:3],
            "att" => [c.attitude[i] for i in 1:3],
            "accel" => [c.imu_accel[i] for i in 1:3],
            "gyro" => [c.imu_gyro[i] for i in 1:3]
        ) for c in checkpoints]
    ))
    
    return bytes2hex(sha256(json_str))
end

function compute_proof(states::Vector, execution_time::Float64)
    """
    Generate trajectory proof from simulation states.
    """
    checkpoints = downsample_trajectory(states, 10)
    traj_hash = compute_trajectory_hash(checkpoints)
    
    # IMU stream hash (sample every 5th checkpoint)
    imu_sample = [c.imu_accel for c in checkpoints[1:5:end]]
    imu_json = json(imu_sample)
    imu_hash = bytes2hex(sha256(imu_json))
    
    # Compute energy (Pi 5: ~5W peak, assume 80% utilization)
    energy_joules = execution_time * 5.0 * 0.8
    
    proof = TrajectoryProof(
        traj_hash,
        imu_hash,
        execution_time,
        length(checkpoints),
        energy_joules,
        Dates.format(Dates.now(), "yyyy-mm-ddTHH:MM:SSZ")
    )
    
    return proof, checkpoints
end

function verify_proof(states::Vector, proof::TrajectoryProof, tolerance::Float64=0.01)
    """
    Validator re-runs simulation, compares proofs.
    Returns (is_valid, error_message).
    """
    checkpoints = downsample_trajectory(states, 10)
    recomputed_hash = compute_trajectory_hash(checkpoints)
    
    if recomputed_hash != proof.trajectory_hash
        return false, "Trajectory hash mismatch"
    end
    
    # Check checkpoint count (allow ±1 due to rounding)
    if abs(length(checkpoints) - proof.checkpoint_count) > 1
        return false, "Checkpoint count mismatch"
    end
    
    # Verify energy estimate is reasonable
    if proof.energy_used < 0
        return false, "Invalid energy value"
    end
    
    return true, "Proof valid"
end

function proof_to_dict(proof::TrajectoryProof)
    """
    Serialize proof for blockchain/network transmission.
    """
    return Dict(
        "trajectory_hash" => proof.trajectory_hash,
        "imu_hash" => proof.imu_hash,
        "execution_time" => proof.execution_time,
        "checkpoint_count" => proof.checkpoint_count,
        "energy_used" => proof.energy_used,
        "timestamp" => proof.timestamp
    )
end

function dict_to_proof(d::Dict)
    """
    Deserialize proof from dict.
    """
    return TrajectoryProof(
        d["trajectory_hash"],
        d["imu_hash"],
        d["execution_time"],
        d["checkpoint_count"],
        d["energy_used"],
        d["timestamp"]
    )
end
