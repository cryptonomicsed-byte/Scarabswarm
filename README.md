# ScarabSwarm: Decentralized Autonomous Drone Racing Simulation

**ScarabSwarm** is a high-fidelity simulation engine for autonomous scarab drones racing through gate courses. Built in Julia with RigidBodyDynamics, it serves as the **core tech for Path 3** of the SimSwarm project—proving autonomous racing in simulation before moving to tokenized blockchain validation.

## Architecture

- **Flight Dynamics**: RigidBodyDynamics.jl for 6-DOF quadrotor physics
- **Collision Avoidance**: Repulsion-force swarm coordination  
- **Proof System**: SHA256 trajectory hashes for deterministic validation
- **Control**: Naive proportional (tested) + LLM pilot (Ollama-integrated)
- **Scoring**: Gates, speed, efficiency, energy

## Quick Start

### Requirements

- Julia 1.9+
- Dependencies in `Project.toml` (auto-installed)

### Setup

```bash
cd scarabswarm
julia --project=.
julia> using Pkg; Pkg.instantiate()
```

### Run Race Demo (6 drones, 5 gates, 30s)

```bash
julia --project=. examples/race_demo.jl
```

Output:
- Race results (time, gates, score)
- Trajectory proofs (SHA256 hashes)
- `race_results.json` (exportable for validation)

## Code Structure

```
scarabswarm/
├── src/
│   ├── ScarabSwarm.jl        # Module root
│   ├── dynamics.jl           # Flight physics (RigidBodyDynamics)
│   ├── validator.jl          # Trajectory proof + verification
│   ├── racecourse.jl         # Gate geometry + scoring
│   ├── llm_pilot.jl          # Ollama integration (optional)
│   └── swarm.jl              # Multi-drone coordination
├── examples/
│   └── race_demo.jl          # Runnable demo
├── models/
│   └── scarab.urdf           # Robot description
├── Project.toml              # Julia dependencies
└── README.md                 # This file
```

## Key Components

### 1. Flight Dynamics (`dynamics.jl`)

```julia
# Create scarab with default physics
dyn = create_scarab_dynamics()

# Initialize hovering state
state = initialize_state(0.0, SVector(0, 0, 1))

# Simulate with motor callback
states = simulate_scarab(dyn, state, controller_fn, duration=30.0)

# Extract trajectory
trajectory = get_trajectory(states)
```

**Model**: 65mm quadrotor (X-frame, 4 props, fixed wings)
- Mass: 500g
- Arm length: 32.5mm
- Physics loop: 10ms @ 100Hz

### 2. Trajectory Validator (`validator.jl`)

```julia
# Compute proof from simulation
proof, checkpoints = compute_proof(states, execution_time)

# Verify (validator re-runs sim)
is_valid, msg = verify_proof(recomputed_states, proof)

# Export for blockchain/network
proof_dict = proof_to_dict(proof)
```

**Proof contains**:
- Trajectory hash (SHA256 of keyframes)
- IMU hash (gyro sample verification)
- Execution time
- Checkpoint count
- Energy estimate

### 3. Race Course (`racecourse.jl`)

```julia
# Standard 5-gate course (20m linear)
course = create_standard_course()

# Score a drone's run
score_dict = compute_race_score(states, course)
# Returns: time, gates_passed, crash, efficiency, final_score
```

**Scoring**:
- **Time**: Seconds from start to finish (Inf if incomplete)
- **Gates**: Count of gates passed in order
- **Efficiency**: Trajectory length / gates passed
- **Crash**: Z < 0.1m penalty
- **Score**: time + efficiency + crash_penalty (lower is better)

### 4. LLM Pilot (`llm_pilot.jl`) — Optional

Requires Ollama running locally:

```bash
ollama pull llama2
ollama serve
```

Then in Julia:

```julia
# Create Ollama-powered pilot
pilot = create_llm_pilot("localhost:11434", "llama2")

# Create controller that queries LLM every 10 steps
controller = create_llm_controller(pilot, course, query_interval=10)

# Use in simulation
states = simulate_scarab(dyn, state, controller, 30.0)
```

**Note**: LLM inference (~500ms) slow compared to flight loop (10ms). Query sparingly.

### 5. Swarm Coordination (`swarm.jl`)

```julia
# Create swarm controller (repulsion radius 1m, formation center)
swarm = create_swarm_controller(
    formation_target=SVector(0, 0, 2),
    repulsion_radius=1.0
)

# Simulate multi-drone race with collision avoidance
results = simulate_swarm_race(drones, course, 30.0)

# Print results
print_swarm_results(results)
```

**Collision avoidance**: Inverse-distance repulsion forces between drones.

## Example: Custom Race

```julia
push!(LOAD_PATH, "src")
using ScarabSwarm
using StaticArrays

# Create one drone
dyn = create_scarab_dynamics()
state = initialize_state(0.0, SVector(0, 0, 1))
controller = create_naive_controller(create_standard_course())

# Simulate 30 seconds
states = simulate_scarab(dyn, state, controller, 30.0)

# Get proof for validation
proof, checkpoints = compute_proof(states, 30.0)

# Score
course = create_standard_course()
score = compute_race_score(states, course)

println("Score: $(score["score"])")
println("Gates: $(score["gates_passed"])/$(score["total_gates"])")
println("Proof: $(proof.trajectory_hash[1:16])...")
```

## Data Formats

### TrajectoryProof (JSON)

```json
{
  "trajectory_hash": "a1b2c3d4e5f6...",
  "imu_hash": "f1e2d3c4b5a6...",
  "execution_time": 30.0,
  "checkpoint_count": 300,
  "energy_used": 120.5,
  "timestamp": "2025-12-16T14:30:00Z"
}
```

### Race Results (`race_results.json`)

```json
{
  "race_metadata": {
    "course_gates": 5,
    "num_drones": 6,
    "duration_s": 30.0,
    "timestamp": "2025-12-16T14:30:00Z"
  },
  "results": {
    "1": {
      "score": {
        "status": "complete",
        "time": 12.5,
        "gates_passed": 5,
        "crash": false,
        "score": 18.75
      },
      "proof": { ... }
    }
  }
}
```

## Path to Blockchain (Phase 2)

Once Path 3 validates autonomous racing:

1. **Export proofs** → submit to Cosmos chain
2. **Validators re-run** sims on CosmWasm contracts
3. **Token rewards** for successful proofs (Proof-of-Simulation)
4. **Marketplace** for researchers to submit training jobs

## Performance

On a **Pi 5** (8-core ARM):
- Single drone 30s race: ~2 seconds wall-clock
- 6-drone swarm: ~15 seconds
- Trajectory proof generation: <50ms

On a **laptop** (Intel i7):
- Single drone: <500ms
- 6-drone swarm: ~3 seconds

## Future Work

- [ ] Flapping wing aerodynamics (Phase 2)
- [ ] Wind disturbance simulation
- [ ] LLM decision loop optimization
- [ ] Real-world Pi 5 flight integration
- [ ] Cosmos blockchain deployment
- [ ] SIM token launch

## License

Ritual computation for the decentralized swarm. 🍶🌶️🥂🐓🥖

## Contact

Bínò ÈL Guà @ ScarabSwarm
