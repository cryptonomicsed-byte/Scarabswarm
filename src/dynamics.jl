# Scarab flight dynamics — RigidBodyDynamics integration
# 65mm fixed-wing quadrotor with scarab aerodynamic shell

using RigidBodyDynamics
using StaticArrays

struct ScarabState
    t::Float64              # Time (s)
    position::SVector{3, Float64}  # [x, y, z] (m)
    velocity::SVector{3, Float64}  # [vx, vy, vz] (m/s)
    attitude::SVector{3, Float64}  # [roll, pitch, yaw] (rad)
    angular_velocity::SVector{3, Float64}  # [p, q, r] (rad/s)
    motor_commands::SVector{4, Float64}  # [M1, M2, M3, M4] (0-1 thrust)
    imu_accel::SVector{3, Float64}  # Accelerometer reading (m/s²)
    imu_gyro::SVector{3, Float64}   # Gyroscope reading (rad/s)
end

struct ScarabDynamics
    mass::Float64
    Ixx::Float64  # Moment of inertia
    Iyy::Float64
    Izz::Float64
    arm_length::Float64  # Distance from center to prop (m)
    thrust_coeff::Float64  # Thrust = coeff * ω²
    drag_coeff::Float64
    mechanism::Mechanism  # RigidBodyDynamics mechanism
    state::MechanismState
end

function create_scarab_dynamics(urdf_path::String="models/scarab.urdf")
    # Load URDF or create default
    try
        mechanism = parse_urdf(urdf_path)
    catch
        # Fallback: simple 6-DOF rigid body
        world = RigidBody{Float64}("world")
        mechanism = Mechanism(world)
    end
    
    state = MechanismState(mechanism)
    
    # Physical parameters for 65mm quad
    dynamics = ScarabDynamics(
        mass=0.5,           # 500g
        Ixx=0.001,          # kg⋅m²
        Iyy=0.001,
        Izz=0.002,
        arm_length=0.0325,  # 65mm / 2
        thrust_coeff=1e-6,  # Empirical
        drag_coeff=0.01,
        mechanism=mechanism,
        state=state
    )
    
    return dynamics
end

function initialize_state(t0=0.0, pos=SVector(0.0, 0.0, 0.0))
    return ScarabState(
        t0,
        pos,
        SVector(0.0, 0.0, 0.0),      # velocity
        SVector(0.0, 0.0, 0.0),      # attitude (roll, pitch, yaw)
        SVector(0.0, 0.0, 0.0),      # angular velocity
        SVector(0.25, 0.25, 0.25, 0.25),  # idle motor commands
        SVector(0.0, 0.0, 9.81),     # gravity in accel
        SVector(0.0, 0.0, 0.0)       # gyro
    )
end

function rotate_to_body(v::SVector{3}, roll::Float64, pitch::Float64, yaw::Float64)
    # Rotation matrix from world to body (ZYX Euler angles)
    Rx = @SMatrix [1  0  0; 
                   0  cos(roll)  -sin(roll);
                   0  sin(roll)   cos(roll)]
    
    Ry = @SMatrix [cos(pitch)   0  sin(pitch);
                   0  1  0;
                   -sin(pitch)  0  cos(pitch)]
    
    Rz = @SMatrix [cos(yaw)  -sin(yaw)  0;
                   sin(yaw)   cos(yaw)  0;
                   0  0  1]
    
    return Rz * Ry * Rx * v
end

function dynamics_step(dyn::ScarabDynamics, state::ScarabState, dt::Float64)
    # Motor thrust allocation: convert 4 motor commands to forces/torques
    m1, m2, m3, m4 = state.motor_commands
    
    # Thrust per motor: F = coeff * ω²
    f1 = dyn.thrust_coeff * m1^2 * dyn.mass * 9.81
    f2 = dyn.thrust_coeff * m2^2 * dyn.mass * 9.81
    f3 = dyn.thrust_coeff * m3^2 * dyn.mass * 9.81
    f4 = dyn.thrust_coeff * m4^2 * dyn.mass * 9.81
    
    # Total thrust (body frame Z)
    total_thrust = f1 + f2 + f3 + f4
    
    # Torques from differential thrust (cross-configuration)
    #  1   3
    #   \ /
    #    X
    #   / \
    #  2   4
    L = dyn.arm_length
    τ_roll = (f3 + f4 - f1 - f2) * L
    τ_pitch = (f1 + f3 - f2 - f4) * L
    τ_yaw = (f1 + f2 - f3 - f4) * L * 0.1  # Yaw moment (less effective)
    
    # Gravity
    g = 9.81
    
    # Accelerations in body frame
    ax = (total_thrust / dyn.mass) * (sin(state.attitude[3]) * sin(state.attitude[1]) + cos(state.attitude[3]) * sin(state.attitude[2]))
    ay = (total_thrust / dyn.mass) * (sin(state.attitude[3]) * sin(state.attitude[2]) - cos(state.attitude[3]) * sin(state.attitude[1]))
    az = (total_thrust / dyn.mass) - g
    
    # Angular accelerations
    p, q, r = state.angular_velocity
    α_roll = (τ_roll / dyn.Ixx) - (dyn.Izz - dyn.Iyy) / dyn.Ixx * q * r
    α_pitch = (τ_pitch / dyn.Iyy) - (dyn.Ixx - dyn.Izz) / dyn.Iyy * p * r
    α_yaw = (τ_yaw / dyn.Izz) - (dyn.Iyy - dyn.Ixx) / dyn.Izz * p * q
    
    # Kinematic integration (Euler method for simplicity)
    new_pos = state.position + state.velocity * dt
    new_vel = state.velocity + SVector(ax, ay, az) * dt
    
    roll, pitch, yaw = state.attitude
    new_roll = roll + p * dt
    new_pitch = pitch + q * dt
    new_yaw = yaw + r * dt
    
    new_angular_vel = state.angular_velocity + SVector(α_roll, α_pitch, α_yaw) * dt
    
    # IMU readings (with noise in real case)
    imu_accel = SVector(ax, ay, az + g)  # Include gravity
    imu_gyro = state.angular_velocity
    
    return ScarabState(
        state.t + dt,
        new_pos,
        new_vel,
        SVector(new_roll, new_pitch, new_yaw),
        new_angular_vel,
        state.motor_commands,
        imu_accel,
        imu_gyro
    )
end

function simulate_scarab(dyn::ScarabDynamics, initial_state::ScarabState, 
                         motor_callback::Function, duration::Float64=10.0, dt::Float64=0.01)
    """
    Simulate scarab flight.
    motor_callback(state) -> SVector{4} motor commands (0-1 throttle)
    """
    states = [initial_state]
    state = initial_state
    
    while state.t < duration
        # Get motor commands from callback (LLM or controller)
        motor_cmds = motor_callback(state)
        state = ScarabState(
            state.t, state.position, state.velocity, state.attitude,
            state.angular_velocity, motor_cmds, state.imu_accel, state.imu_gyro
        )
        
        # Step dynamics
        state = dynamics_step(dyn, state, dt)
        push!(states, state)
    end
    
    return states
end

# Convenience: trajectory extraction
function get_trajectory(states::Vector{ScarabState})
    return [s.position for s in states]
end

function get_attitudes(states::Vector{ScarabState})
    return [s.attitude for s in states]
end

function get_timeline(states::Vector{ScarabState})
    return [s.t for s in states]
end
