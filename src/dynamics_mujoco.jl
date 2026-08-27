# MuJoCo.jl physics backend for ScarabSwarm — OPTIONAL, flag-gated alternative.
#
# Does NOT replace the default hand-rolled Euler integrator in dynamics.jl.
# To opt in, after `using ScarabSwarm` (which defines ScarabState), do:
#
#     include("/path/to/Scarabswarm/src/dynamics_mujoco.jl")
#     dyn = create_scarab_dynamics_mujoco()
#
# WHY NOT THE PROOF PATH (determinism, see docs/mujoco-swap-scope.md):
# MuJoCo is deterministic per-build / per-architecture (single-threaded, fixed
# timestep) but NOT bit-exact across hosts (native SIMD/FMA/libm differ). The
# OSOVM witness-bridge hash requirement is cross-arch bit-exact, so the proof
# hashing must stay on the hand-rolled integrator (or use keyframe
# quantization) — this backend is for richer rigid-body/contact simulation,
# not for proof generation.

using MuJoCo
using StaticArrays

struct ScarabDynamicsMujoco
    model::MuJoCo.Model
    data::MuJoCo.Data
    mass::Float64          # kg
    thrust_coeff::Float64  # F_per_motor = thrust_coeff * m_i^2 * mass * g
    g::Float64             # m/s²
end

function create_scarab_dynamics_mujoco(xml_path::String="models/scarab.xml")
    model = MuJoCo.load_model(xml_path)
    data = MuJoCo.init_data(model)
    # Match the hand-rolled model's physical constants (see dynamics.jl).
    return ScarabDynamicsMujoco(model, data, 0.5, 1.0, 9.81)
end

# MuJoCo free-joint orientation is a unit quaternion [w, x, y, z]; ScarabState
# carries Euler angles [roll, pitch, yaw]. Standard aerospace ZYX conversion.
function _quat_to_euler(q)
    w, x, y, z = q[1], q[2], q[3], q[4]
    roll  = atan(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = asin(clamp(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw   = atan(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return SVector(roll, pitch, yaw)
end

function dynamics_step_mujoco(dyn::ScarabDynamicsMujoco, state::ScarabState, dt::Float64)
    # MuJoCo's internal timestep comes from <option timestep> in the XML
    # (0.01s, matching the hand-rolled dt). The dt argument is used only for
    # the returned state's timestamp; a mismatched dt is a caller error.
    m = state.motor_commands
    f = dyn.thrust_coeff * dyn.mass * dyn.g   # full-throttle force per motor (N)
    c = dyn.data.ctrl
    c[1] = f * m[1]^2
    c[2] = f * m[2]^2
    c[3] = f * m[3]^2
    c[4] = f * m[4]^2

    MuJoCo.step!(dyn.model, dyn.data)

    qpos = dyn.data.qpos
    qvel = dyn.data.qvel
    pos = SVector(qpos[1], qpos[2], qpos[3])
    vel = SVector(qvel[1], qvel[2], qvel[3])
    ang = SVector(qvel[4], qvel[5], qvel[6])
    attitude = _quat_to_euler((qpos[4], qpos[5], qpos[6], qpos[7]))

    # IMU: body-frame acceleration is not directly exposed by MuJoCo.jl's
    # Data wrapper without differentiating qacc; report world-frame qacc and
    # gyro from qvel[4:6] as a first approximation (see note in demo).
    qacc = dyn.data.qacc
    imu_accel = SVector(qacc[1], qacc[2], qacc[3])
    imu_gyro = ang

    return ScarabState(
        state.t + dt, pos, vel, attitude, ang,
        m, imu_accel, imu_gyro,
    )
end

function simulate_scarab_mujoco(dyn::ScarabDynamicsMujoco, initial_state::ScarabState,
                                motor_callback::Function, duration::Float64=10.0, dt::Float64=0.01)
    states = [initial_state]
    state = initial_state
    while state.t < duration
        motor_cmds = motor_callback(state)
        state = ScarabState(
            state.t, state.position, state.velocity, state.attitude,
            state.angular_velocity, motor_cmds, state.imu_accel, state.imu_gyro,
        )
        state = dynamics_step_mujoco(dyn, state, dt)
        push!(states, state)
    end
    return states
end
