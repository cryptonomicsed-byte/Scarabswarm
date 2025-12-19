module ScarabSwarm

using RigidBodyDynamics
using StaticArrays
using LinearAlgebra
using Distributed
using JSON

# Core modules
include("dynamics.jl")
include("validator.jl")
include("racecourse.jl")
include("llm_pilot.jl")
include("swarm.jl")

# Exports
export ScarabDynamics, ScarabState, simulate_scarab
export TrajectoryProof, compute_proof, verify_proof
export RaceCourse, Gate, simulate_race
export LLMPilot, query_ollama, parse_motor_commands
export SwarmController, coordinate_swarm

end # module ScarabSwarm
