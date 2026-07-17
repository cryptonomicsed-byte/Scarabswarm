# Scarab racing simulation — gates, wind, scoring

using StaticArrays
using LinearAlgebra

struct Gate
    position::SVector{3, Float64}  # Center of gate (m)
    normal::SVector{3, Float64}    # Forward direction (unit vector)
    width::Float64                 # Gate width (m), default 0.5m
    height::Float64                # Gate height (m), default 0.5m
    gate_id::Int
end

struct RaceCourse
    gates::Vector{Gate}
    wind::SVector{3, Float64}      # Wind velocity (m/s)
    start_zone::Tuple{SVector{3}, Float64}  # (center, radius)
    finish_zone::Tuple{SVector{3}, Float64}
end

function create_standard_course()
    """
    5-gate race course.
    Linear gates 5m apart, 50m total course length.
    """
    gates = [
        Gate(SVector(0.0, 0.0, 1.0), SVector(1.0, 0.0, 0.0), 0.5, 0.5, 1),
        Gate(SVector(5.0, 0.0, 1.0), SVector(1.0, 0.0, 0.0), 0.5, 0.5, 2),
        Gate(SVector(10.0, 2.0, 1.0), SVector(1.0, 0.5, 0.0), 0.5, 0.5, 3),
        Gate(SVector(15.0, 0.0, 1.0), SVector(1.0, -0.5, 0.0), 0.5, 0.5, 4),
        Gate(SVector(20.0, 0.0, 1.0), SVector(1.0, 0.0, 0.0), 0.5, 0.5, 5),
    ]
    
    start = (SVector(0.0, 0.0, 0.5), 1.0)
    finish = (SVector(20.0, 0.0, 1.0), 1.5)
    wind = SVector(0.5, 0.0, 0.0)  # Light wind
    
    return RaceCourse(gates, wind, start, finish)
end

function point_in_gate(pos::SVector{3}, gate::Gate, tolerance::Float64=0.25)
    """
    Check if position passes through gate plane.
    tolerance: ±0.25m from gate edges.
    """
    to_gate = gate.position - pos
    distance_to_plane = dot(to_gate, gate.normal)
    
    # Gate passed if near plane
    if abs(distance_to_plane) < 0.5
        # Check if within width/height
        lateral = pos - gate.position - distance_to_plane * gate.normal
        lateral_dist = norm(lateral)
        
        # Approximate: rectangular gate
        max_lateral = max(gate.width, gate.height) / 2.0
        return lateral_dist <= (max_lateral + tolerance)
    end
    
    return false
end

function check_gates_passed(trajectory::Vector{<:SVector{3}}, course::RaceCourse)
    # `Vector{SVector{3}}` (unparameterized element type) never matches a
    # concrete `Vector{SVector{3,Float64}}` -- Vector is invariant in Julia,
    # so this needs the covariant `Vector{<:SVector{3}}` bound instead.
    """
    Track which gates drone has passed, in order.
    """
    gates_passed = Int[]
    last_passed = 0
    
    for pos in trajectory
        for (i, gate) in enumerate(course.gates)
            if i > last_passed && point_in_gate(pos, gate)
                push!(gates_passed, i)
                last_passed = i
                break
            end
        end
    end
    
    return gates_passed
end

function compute_race_score(states::Vector, course::RaceCourse)
    """
    Race scoring:
    - Time to complete (s)
    - Gates passed (count)
    - Energy efficiency (trajectory length / energy used)
    - Collision penalty (any Z < 0.1m = crash)
    """
    trajectory = [s.position for s in states]
    times = [s.t for s in states]
    
    # Check start zone
    start_pos, start_radius = course.start_zone
    race_started = false
    start_time = 0.0
    
    for (t, pos) in zip(times, trajectory)
        if norm(pos - start_pos) <= start_radius
            race_started = true
            start_time = t
            break
        end
    end
    
    if !race_started
        # Same shape as the full return below -- print_swarm_results
        # indexes every key unconditionally, so a partial Dict here was a
        # real crash bug whenever any drone never entered the start zone.
        return Dict(
            "status" => "incomplete",
            "time" => Inf,
            "gates_passed" => 0,
            "gates_list" => Int[],
            "total_gates" => length(course.gates),
            "trajectory_length" => 0.0,
            "efficiency" => 0.0,
            "crash" => false,
            "score" => Inf
        )
    end
    
    # Check gates
    gates_passed = check_gates_passed(trajectory, course)
    
    # Check finish zone
    finish_pos, finish_radius = course.finish_zone
    finish_time = Inf
    race_complete = false
    
    for (t, pos) in zip(times, trajectory)
        if t > start_time && norm(pos - finish_pos) <= finish_radius
            finish_time = t
            race_complete = true
            break
        end
    end
    
    # Check for crash (Z < 0.1m)
    crashed = any(pos[3] < 0.1 for pos in trajectory)
    
    # Scoring
    time_score = race_complete ? (finish_time - start_time) : Inf
    gates_score = length(gates_passed)
    crash_penalty = crashed ? 1000.0 : 0.0
    
    # Efficiency: trajectory length vs gates
    traj_length = sum(norm(trajectory[i+1] - trajectory[i]) 
                      for i in 1:(length(trajectory)-1))
    efficiency = gates_score > 0 ? traj_length / gates_score : 0.0
    
    # Final score: lower is better
    final_score = time_score + efficiency + crash_penalty
    
    return Dict(
        "status" => race_complete ? "complete" : "incomplete",
        "time" => time_score,
        "gates_passed" => gates_score,
        "gates_list" => gates_passed,
        "total_gates" => length(course.gates),
        "trajectory_length" => traj_length,
        "efficiency" => efficiency,
        "crash" => crashed,
        "score" => final_score
    )
end

function simulate_race(drones::Dict, course::RaceCourse, duration::Float64=30.0)
    """
    Run multiple drones through course, return results.
    drones: Dict{drone_id => (dynamics, initial_state, motor_callback)}
    """
    results = Dict()
    
    for (drone_id, (dyn, init_state, motor_fn)) in drones
        states = simulate_scarab(dyn, init_state, motor_fn, duration)
        score_dict = compute_race_score(states, course)
        
        results[drone_id] = Dict(
            "states" => states,
            "score" => score_dict,
            "proof" => compute_proof(states, duration)[1]
        )
    end
    
    return results
end
