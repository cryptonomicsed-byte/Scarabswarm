# LLM-based scarab pilot
# Query Ollama for flight decisions, parse into motor commands

using HTTP
using JSON

struct LLMPilot
    host::String         # Backend base URL/host, meaning depends on backend:
                          # :ollama -> "host:port" (e.g. "localhost:11434")
                          # :openai -> "host:port" (e.g. "localhost:8300", OmniRoute)
                          # :omokoda -> "host:port" of a live omokoda-core kernel (:7777)
                          # :hermes -> docker container name (invoked via `docker exec`,
                          #   not HTTP -- Hermes Agent's web dashboard requires cookie
                          #   auth and its `proxy` OpenAI-compatible adapters need an
                          #   interactive OAuth login neither of which are scriptable
                          #   headlessly; `hermes chat -q ... -Q` is the real scriptable
                          #   entry point and reuses the container's already-authenticated
                          #   Nexos.ai session)
    model::String        # Model name; unused for :omokoda/:hermes (they pick their own)
    system_prompt::String
    backend::Symbol       # :ollama, :openai, :omokoda, or :hermes -- selects wire
                          # format/endpoint. This is the universal-copilot seam: any
                          # agent that can accept a text prompt and return a text
                          # decision can be a pilot -- a raw LLM (:ollama/:openai), a
                          # real sovereign agent (:omokoda, POST /v1/think), or a full
                          # tool-using agent (:hermes, `hermes chat`). Each is one more
                          # query_* function + one more branch in query_llm, not a
                          # rewrite -- proven three times over now.
    api_key::String       # bearer token, only used when backend == :openai
end

function create_llm_pilot(host="localhost:11434", model="llama3.2:3b"; backend::Symbol=:ollama, api_key::String="")
    """
    Initialize LLM pilot. backend=:ollama speaks Ollama's native
    /api/generate; backend=:openai speaks OpenAI-compatible
    /v1/chat/completions (OmniRoute, DeepSeek, any OpenAI-shaped gateway).
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

    return LLMPilot(host, model, system_prompt, backend, api_key)
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

function query_openai_compatible(pilot::LLMPilot, prompt::String)
    """
    Send prompt to an OpenAI-compatible /v1/chat/completions endpoint
    (OmniRoute, DeepSeek, etc.). Returns raw response string.
    """
    url = "http://$(pilot.host)/v1/chat/completions"

    payload = Dict(
        "model" => pilot.model,
        "messages" => [
            Dict("role" => "system", "content" => pilot.system_prompt),
            Dict("role" => "user", "content" => prompt),
        ],
        "stream" => false,
    )

    headers = ["Content-Type" => "application/json"]
    if !isempty(pilot.api_key)
        push!(headers, "Authorization" => "Bearer $(pilot.api_key)")
    end

    try
        response = HTTP.post(url, headers, JSON.json(payload); readtimeout=30)
        body = JSON.parse(String(response.body))
        return body["choices"][1]["message"]["content"]
    catch e
        @warn "OpenAI-compatible query failed: $e"
        return ""  # Fallback to hover
    end
end

function query_omokoda(pilot::LLMPilot, prompt::String)
    """
    Send prompt to a live omokoda-core kernel's /v1/think (the real
    sovereign agent, not a raw model call). Returns raw response string.
    Verified live: real round-trip against the actual kernel (agent
    "Ọmọ Kọ́dà", tier 5) answered a real piloting decision in ~11s
    including SSH overhead.
    """
    url = "http://$(pilot.host)/v1/think"

    payload = Dict(
        "prompt" => "$(pilot.system_prompt)\n\n$prompt",
        "private" => false,
        "agentic" => false,
    )

    try
        response = HTTP.post(url, ["Content-Type" => "application/json"], JSON.json(payload); readtimeout=30)
        body = JSON.parse(String(response.body))
        return get(body, "tool_output", "")
    catch e
        @warn "omokoda /v1/think query failed: $e"
        return ""  # Fallback to hover
    end
end

function query_hermes(pilot::LLMPilot, prompt::String)
    """
    Invoke a live Hermes Agent container (`docker exec ... hermes chat
    -q PROMPT -Q`) for a piloting decision. `pilot.host` is the container
    name. No HTTP: the container's web dashboard needs cookie auth and its
    OpenAI-compatible `proxy` needs an interactive OAuth login neither
    scriptable headlessly; `hermes chat -q ... -Q` is the real scriptable
    surface and reuses the container's already-authenticated session.
    Verified live: real query, real Claude Sonnet 4.6-backed response,
    correctly formatted, ~21s round-trip.
    """
    cmd = `docker exec $(pilot.host) hermes chat -q $prompt -Q`
    try
        return read(cmd, String)
    catch e
        @warn "Hermes query failed: $e"
        return ""  # Fallback to hover
    end
end

"""Dispatch to the right wire format based on pilot.backend -- the
universal-copilot seam: swap backends without touching the controller
logic that calls this."""
function query_llm(pilot::LLMPilot, prompt::String)
    pilot.backend == :openai && return query_openai_compatible(pilot, prompt)
    pilot.backend == :omokoda && return query_omokoda(pilot, prompt)
    pilot.backend == :hermes && return query_hermes(pilot, prompt)
    return query_ollama(pilot, prompt)
end

function parse_motor_commands(response::String)
    """
    Parse a pilot's response into motor commands. Deliberately tolerant of
    formatting, since "universal copilot" means different agent backends
    phrase the same decision differently -- verified live against a real
    omokoda-core /v1/think response that came back as a single line, no
    colons, space-separated ("THROTTLE 0 ROLL -0.2 PITCH 1.0 YAW 0"),
    which the original strict "THROTTLE:" line-prefix parser would have
    silently missed and defaulted to hover. Regex-extracts each keyword
    followed by a number, regardless of colon/newline/spacing.

    Returns SVector{4} motor commands [M1, M2, M3, M4] (0-1).
    """

    throttle = 0.4  # Hover default
    roll = 0.0
    pitch = 0.0
    yaw = 0.0

    num = raw"[-+]?[0-9]*\.?[0-9]+"
    m = match(Regex("THROTTLE\\s*:?\\s*($num)", "i"), response)
    m !== nothing && (throttle = clamp(parse(Float64, m.captures[1]), 0.0, 1.0))
    m = match(Regex("ROLL\\s*:?\\s*($num)", "i"), response)
    m !== nothing && (roll = clamp(parse(Float64, m.captures[1]), -0.5, 0.5))
    m = match(Regex("PITCH\\s*:?\\s*($num)", "i"), response)
    m !== nothing && (pitch = clamp(parse(Float64, m.captures[1]), -0.5, 0.5))
    m = match(Regex("YAW\\s*:?\\s*($num)", "i"), response)
    m !== nothing && (yaw = clamp(parse(Float64, m.captures[1]), -0.5, 0.5))

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
            
            # Query LLM (dispatches to Ollama or OpenAI-compatible wire format)
            response = query_llm(pilot, prompt)
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
        # Altitude control: throttle was hardcoded to 0.6 with no closed
        # loop at all -- direction[3] (the vertical component toward the
        # gate) was computed but never used. With the real hover point at
        # throttle=0.5 (see dynamics.jl's thrust_coeff fix), a fixed 0.6
        # produces MORE thrust than weight, so the drone climbed
        # unboundedly forever regardless of the gates' actual altitude.
        # Proportional correction around the real hover baseline instead.
        throttle = clamp(0.5 + direction[3] * 0.3, 0.0, 1.0)
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
