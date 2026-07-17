#!/usr/bin/env julia
"""
ScarabSwarm Race Demo
- 6 scarab drones
- 5-gate standard course
- Naive proportional control (no LLM, fast testing)
- Collision avoidance via repulsion forces
- Results + proofs

Run: julia --project=.. examples/race_demo.jl
"""

push!(LOAD_PATH, "../src")
using ScarabSwarm
using StaticArrays
using JSON
using Dates

function main()
    println("🔥 SCARABSWARM RACE DEMO 🔥\n")
    
    # Create standard race course
    course = create_standard_course()
    println("Course: $(length(course.gates)) gates, 20m linear")
    println("Wind: $(course.wind)\n")
    
    # Create 6 drones with staggered starting positions
    drones = Dict()
    
    for drone_id in 1:6
        # Stagger start positions
        offset = (drone_id - 1) * 0.3
        start_pos = SVector(0.0, offset, 1.0)
        
        # Create dynamics
        dyn = create_scarab_dynamics()
        init_state = initialize_state(0.0, start_pos)
        
        # Use naive proportional controller
        controller = create_naive_controller(course)
        
        drones[drone_id] = (dyn, init_state, controller)
    end
    
    println("Created $(length(drones)) scarabs\n")
    println("Starting race simulation... (30s duration)\n")
    
    # Simulate race
    @time results = simulate_swarm_race(drones, course, 30.0)
    
    # Print results
    print_swarm_results(results)
    
    # Print proofs
    println("\n=== TRAJECTORY PROOFS ===\n")
    for (drone_id, result) in sort(collect(results), by=x -> x[2]["score"]["score"])
        proof = result["proof"]
        println("Drone #$drone_id")
        println("  Trajectory Hash: $(proof.trajectory_hash[1:16])...")
        println("  Execution Time: $(round(proof.execution_time, digits=2))s")
        println("  Checkpoints: $(proof.checkpoint_count)")
        println("  Energy: $(round(proof.energy_used, digits=2)) J")
        println("  Timestamp: $(proof.timestamp)")
        println()
    end
    
    # Save results to JSON
    output = Dict(
        "race_metadata" => Dict(
            "course_gates" => length(course.gates),
            "num_drones" => length(drones),
            "duration_s" => 30.0,
            "timestamp" => Dates.format(Dates.now(), "yyyy-mm-ddTHH:MM:SSZ")
        ),
        "results" => Dict(
            drone_id => Dict(
                "score" => result["score"],
                "proof" => proof_to_dict(result["proof"])
            )
            for (drone_id, result) in results
        )
    )
    
    open("race_results.json", "w") do f
        # JSON.print(..., indent=2) has no matching method for this
        # Dict{String,Dict} type in the installed JSON.jl version -- use
        # the same JSON.json(...) pattern already verified working in
        # validator.jl instead.
        # allownan=true: `time`/`score` are legitimately Inf for any drone
        # that never reaches the finish zone in the race duration -- real
        # data, not a bug, and the JSON spec's ban on Inf shouldn't silently
        # crash a real result.
        write(f, JSON.json(output; allownan=true))
    end
    
    println("Results saved to race_results.json")
end

if abspath(PROGRAM_FILE) == @__FILE__
    main()
end
