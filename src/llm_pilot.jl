# LLM-based scarab pilot
# Query Ollama for flight decisions, parse into motor commands

using HTTP
using JSON

struct LLMPilot
    host::String         # Ollama host (default localhost:11434)
    model::String        # Model name (e.g., "llama2", "neural-chat")
    system_prompt::String
end

function create_llm_pilot(host="localhost:11434", model="llama2")
    """
    Initialize LLM pilot connected to Ollama.
    """
    system_prompt = """You are a scarab drone pilot. You control a tiny quadrotor racing through gates.
    
Given current state, respond with EXACTLY this format (no other text):
THROTTLE: 0.0-1.0
ROLL: -0.5 to 0.5 (radians)
PITCH: -0.5 to 0.5 (radians)
YAW: -0.5 to 0.5 (rad/s yaw rate)

Current state:
- Position: [x, y, z] meters
- Velocity: [vx, vy, vz] m/s
- Attitude: [roll, pitch, yaw] radians
- Next gate at: [gx, gy, gz]
- Distance to next gate: D meters

Decide quickly. Low latency wins."""
    
    return LLMPilot(host, model, system_prompt)
end

function query_ollama(pilot::LLMPilot, prompt::String)
    """
    Send prompt to Ollama, get response.
    Returns raw response string.
    """
    url = "http://$(pilot.host)/api/generate"
    
    payload = Dict(
        "model" => pilot.model,
        "prompt" => "$(pilot.system_prompt)\n\n$prompt",
        "stream" => false
    )
    
    try
        response = HTTP.post(url, ["Content-Type" => "application/json"], JSON.json(payload))
        body = JSON.parse(String(response.body))
        return body["response"]
    catch e
        @warn "Ollama query failed: $e"
        return ""  # Fallback to hover
    end
end

function parse_motor_commands(response::String)
    """
    Parse LLM response into motor commands.
    Format expected:
    THROTTLE: 0.5
    ROLL: 0.1
    PITCH: 0.05
    YAW: 0.1
    
    Returns SVector{4} motor commands [M1, M2, M3, M4] (0-1).
    """
    
    throttle = 0.4  # Hover default
    roll = 0.0
    pitch = 0.0
    yaw = 0.0
    
    lines = split(response, '\n')
    for line in lines
        line = strip(line)
        
        if startswith(line, "THROTTLE:")
            try
                val = parse(Float64, split(line, ':')[2])
                throttle = clamp(val, 0.0, 1.0)
            catch
            end
        elseif startswith(line, "ROLL:")
            try
                val = parse(Float64, split(line, ':')[2])
                roll = clamp(val, -0.5, 0.5)
            catch
            end
        elseif startswith(line, "PITCH:")
            try
                val = parse(Float64, split(line, ':')[2])
                pitch = clamp(val, -0.5, 0.5)
            catch
            end
        elseif startswith(line, "YAW:")
            try
                val = parse(Float64, split(line, ':')[2])
                yaw = clamp(val, -0.5, 0.5)
            catch
            end
        end
    end
    
    # Convert attitude commands to motor mix (simplified)
    # Quadrotor mixing: M1=front-left, M2=front-right, M3=back-right, M4=back-left
    m1 = throttle + pitch + roll + yaw
    m2 = throttle + pitch - roll - yaw
    m3 = throttle - pitch - roll + yaw
    m4 = throttle - pitch + roll - yaw
    
    # Clamp to 0-1
    m1 = clamp(m1, 0.0, 1.0)
    m2 = clamp(m2, 0.0, 1.0)
    m3 = clamp(m3, 0.0, 1.0)
    m4 = clamp(m4, 0.0, 1.0)
    
    return SVector(m1, m2, m3, m4)
end

function create_llm_controller(pilot::LLMPilot, course::RaceCourse, 
                              query_interval::Int=10)
    """
    Returns a closure that queries LLM every query_interval steps.
    Reduces LLM load (LLM inference ~500ms, flight loop ~10ms).
    """
    query_count = 0
    cached_commands = SVector(0.4, 0.4, 0.4, 0.4)
    
    function controller(state::ScarabState)
        query_count += 1
        
        if query_count % query_interval == 0
            # Find nearest gate ahead
            nearest_gate = course.gates[1]
            min_dist = Inf
            
            for gate in course.gates
                dist = norm(gate.position - state.position)
                if dist < min_dist
                    min_dist = dist
                    nearest_gate = gate
                end
            end
            
            # Format prompt
            prompt = """Position: [$(round(state.position[1], digits=2)), $(round(state.position[2], digits=2)), $(round(state.position[3], digits=2))]
Velocity: [$(round(state.velocity[1], digits=2)), $(round(state.velocity[2], digits=2)), $(round(state.velocity[3], digits=2))]
Attitude: [$(round(state.attitude[1], digits=2)), $(round(state.attitude[2], digits=2)), $(round(state.attitude[3], digits=2))]
Next gate: [$(round(nearest_gate.position[1], digits=2)), $(round(nearest_gate.position[2], digits=2)), $(round(nearest_gate.position[3], digits=2))]
Distance: $(round(min_dist, digits=2))m"""
            
            # Query LLM
            response = query_ollama(pilot, prompt)
            cached_commands = parse_motor_commands(response)
        end
        
        return cached_commands
    end
    
    return controller
end

# Fallback controller (no LLM needed for basic testing)
function create_naive_controller(course::RaceCourse)
    """
    Simple proportional controller: steer toward next gate.
    No LLM, deterministic, fast.
    """
    gate_idx = 1
    
    function controller(state::ScarabState)
        if gate_idx > length(course.gates)
            return SVector(0.4, 0.0, 0.0, 0.0)  # Hover
        end
        
        gate = course.gates[gate_idx]
        to_gate = gate.position - state.position
        dist = norm(to_gate)
        
        # Move to next gate if close enough
        if dist < 1.0
            gate_idx += 1
        end
        
        # Simple P-control
        direction = to_gate / (dist + 0.1)
        throttle = 0.6
        roll = clamp(direction[2] * 0.5, -0.5, 0.5)
        pitch = clamp(direction[1] * 0.5, -0.5, 0.5)
        yaw = 0.0
        
        m1 = throttle + pitch + roll
        m2 = throttle + pitch - roll
        m3 = throttle - pitch - roll
        m4 = throttle - pitch + roll
        
        return SVector(clamp(m1, 0, 1), clamp(m2, 0, 1), 
                       clamp(m3, 0, 1), clamp(m4, 0, 1))
    end
    
    return controller
end
