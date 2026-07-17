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
export ScarabDynamics, ScarabState, simulate_scarab, create_scarab_dynamics, initialize_state
export TrajectoryProof, compute_proof, verify_proof, proof_to_dict
export RaceCourse, Gate, simulate_race, create_standard_course, compute_race_score
export LLMPilot, query_ollama, query_openai_compatible, query_omokoda, query_hermes, query_llm, parse_motor_commands
export create_llm_pilot, create_llm_controller, create_naive_controller
export SwarmController, simulate_swarm_race, print_swarm_results
# Note: `coordinate_swarm` was previously exported but was never actually
# defined anywhere in swarm.jl -- a phantom export. Removed rather than
# stubbed; swarm.jl's real API is create_swarm_controller +
# create_swarm_controller_callback + simulate_swarm_race.

end # module ScarabSwarm
