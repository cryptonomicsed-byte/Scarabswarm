# Multi-drone coordination and collision avoidance

using StaticArrays
using LinearAlgebra

struct SwarmController
    drones::Dict{Int, NamedTuple}  # drone_id => (position, velocity, state)
    repulsion_radius::Float64      # Collision avoidance radius (m)
    formation_target::SVector{3, Float64}  # Desired formation center
end

function create_swarm_controller(formation_target::SVector{3}=SVector(0.0, 0.0, 2.0),
                                 repulsion_radius::Float64=1.0)
    """
    Initialize swarm coordination.
    """
    return SwarmController(
        Dict(),
        repulsion_radius,
        formation_target
    )
end

function compute_separation_force(my_pos::SVector{3}, other_pos::SVector{3}, 
                                  radius::Float64)
    """
    Compute repulsive force from neighboring drone.
    Force magnitude inversely proportional to distance.
    """
    delta = my_pos - other_pos
    dist = norm(delta)
    
    if dist < 0.1
        return SVector(0.0, 0.0, 0.0)  # Too close, fallback
    end
    
    if dist > radius
        return SVector(0.0, 0.0, 0.0)  # Outside interaction range
    end
    
    # Repulsive force: F = k * (1/d²)
    k = 1.0
    force_mag = k / (dist^2 + 0.1)
    direction = delta / dist
    
    return force_mag * direction
end

function compute_formation_force(my_pos::SVector{3}, 
                                 formation_target::SVector{3},
                                 gain::Float64=0.1)
    """
    Compute attractive force toward formation center.
    """
    delta = formation_target - my_pos
    return gain * delta
end

function create_swarm_controller_callback(swarm::SwarmController, 
                                          single_drone_controller::Function,
                                          all_states::Vector)
    """
    Wraps single-drone controller with collision avoidance.
    all_states: shared state of all drones in swarm.
    """
    
    function swarm_controller_fn(drone_id::Int, my_state::ScarabState)
        # Get baseline commands from single controller
        base_cmds = single_drone_controller(my_state)
        
        # Compute separation forces from neighbors
        sep_force = SVector(0.0, 0.0, 0.0)
        for (other_id, other_state) in all_states
            if other_id != drone_id
                sep_force += compute_separation_force(
                    my_state.position, 
                    other_state.position,
                    swarm.repulsion_radius
                )
            end
        end
        
        # Compute formation force
        form_force = compute_formation_force(my_state.position, 
                                              swarm.formation_target)
        
        # Combine forces into attitude commands
        total_force = sep_force + form_force
        
        # Simple mapping: force -> pitch/roll
        throttle = 0.5 + 0.1 * total_force[3]  # Altitude control
        roll = clamp(total_force[2] * 0.2, -0.3, 0.3)
        pitch = clamp(total_force[1] * 0.2, -0.3, 0.3)
        yaw = 0.0
        
        # Convert to motor mix
        m1 = throttle + pitch + roll
        m2 = throttle + pitch - roll
        m3 = throttle - pitch - roll
        m4 = throttle - pitch + roll
        
        return SVector(clamp(m1, 0, 1), clamp(m2, 0, 1),
                       clamp(m3, 0, 1), clamp(m4, 0, 1))
    end
    
    return swarm_controller_fn
end

function simulate_swarm_race(drones::Dict, course::RaceCourse, 
                             duration::Float64=30.0)
    """
    Run multi-drone race with collision avoidance.
    drones: Dict{drone_id => (dynamics, initial_state, controller)}
    """
    
    # Initialize all drone states
    all_states = Dict()
    for (drone_id, (dyn, init_state, ctrl)) in drones
        all_states[drone_id] = init_state
    end
    
    # Store trajectories. Seed each with the real t=0 initial state -- it
    # was already computed above into all_states but never recorded here,
    # so compute_race_score's start-zone check never saw a drone's true
    # starting position, only its position after the first physics step.
    trajectories = Dict{Int, Vector{ScarabState}}()
    for (drone_id, (dyn, init_state, ctrl)) in drones
        trajectories[drone_id] = [init_state]
    end
    
    # Simulation loop
    dt = 0.01
    t = 0.0
    
    while t < duration
        for (drone_id, (dyn, _, ctrl)) in drones
            state = all_states[drone_id]
            
            # Get motor commands from controller. Every controller in this
            # codebase (create_naive_controller, create_llm_controller) uses
            # the single-arg (state) convention -- this call site was the
            # only place assuming a (drone_id, state) signature.
            motor_cmds = ctrl(state)
            state = ScarabState(state.t, state.position, state.velocity,
                               state.attitude, state.angular_velocity,
                               motor_cmds, state.imu_accel, state.imu_gyro)
            
            # Update dynamics
            state = dynamics_step(dyn, state, dt)
            all_states[drone_id] = state
            
            # Record
            push!(trajectories[drone_id], state)
        end
        
        t += dt
    end
    
    # Score all drones
    results = Dict()
    for (drone_id, states) in trajectories
        score = compute_race_score(states, course)
        proof, checkpoints = compute_proof(states, duration)
        
        results[drone_id] = Dict(
            "score" => score,
            "states" => states,
            "proof" => proof
        )
    end
    
    return results
end

function print_swarm_results(results::Dict)
    """
    Pretty-print race results.
    """
    println("\n=== SCARAB SWARM RACE RESULTS ===\n")
    
    sorted_drones = sort(collect(results), by=x -> x[2]["score"]["score"])
    
    for (rank, (drone_id, result)) in enumerate(sorted_drones)
        score_dict = result["score"]
        println("Drone #$drone_id | Rank: $rank")
        println("  Status: $(score_dict["status"])")
        println("  Time: $(round(score_dict["time"], digits=2))s")
        println("  Gates: $(score_dict["gates_passed"])/$(score_dict["total_gates"])")
        println("  Efficiency: $(round(score_dict["efficiency"], digits=3))m/gate")
        println("  Crashed: $(score_dict["crash"])")
        println("  SCORE: $(round(score_dict["score"], digits=2))")
        println()
    end
end
