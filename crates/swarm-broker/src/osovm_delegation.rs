use serde_json::{json, Value};
use swarm_types::{SimOutcome, sim_receipt::sha256_hex};

fn osovm_base() -> String {
    std::env::var("OSOVM_URL").unwrap_or_else(|_| "http://127.0.0.1:7780".to_string())
}

/// Request parameters sent to OSOVM veilsim_engine.
#[derive(serde::Serialize)]
struct OsovmSimRequest {
    twin_id:      String,
    agent_id:     String,
    n_candidates: u32,
    session_id:   Option<String>,
}

/// Response from OSOVM veilsim_engine.
#[derive(Debug, serde::Deserialize)]
pub struct OsovmSimResult {
    pub traj_hashes:     Vec<String>,
    pub winning_traj_id: String,
    pub winner_score:    f64,
    pub n_feasible:      u32,
    pub policy_json:     Value,
}

/// Delegate a simulation run to OSOVM veilsim_engine.
/// Fail-open: on error, returns a synthetic result so the receipt chain still completes.
pub async fn run_via_osovm(
    twin_id:      &str,
    agent_id:     &str,
    n_candidates: u32,
    session_id:   Option<&str>,
) -> (OsovmSimResult, SimOutcome) {
    let req = OsovmSimRequest {
        twin_id:      twin_id.to_string(),
        agent_id:     agent_id.to_string(),
        n_candidates,
        session_id:   session_id.map(String::from),
    };

    let client = reqwest::Client::new();
    let url = format!("{}/api/sim/run", osovm_base());

    match client
        .post(&url)
        .json(&req)
        .timeout(std::time::Duration::from_secs(60))
        .send()
        .await
    {
        Ok(resp) if resp.status().is_success() => {
            match resp.json::<OsovmSimResult>().await {
                Ok(result) => (result, SimOutcome::PolicySelected),
                Err(e) => {
                    tracing::warn!("OSOVM response parse error: {e}");
                    (synthetic_result(twin_id, agent_id, n_candidates), SimOutcome::TwinUnavailable)
                }
            }
        }
        Ok(resp) => {
            tracing::warn!("OSOVM returned {}", resp.status());
            (synthetic_result(twin_id, agent_id, n_candidates), SimOutcome::TwinUnavailable)
        }
        Err(e) => {
            tracing::debug!("OSOVM unreachable: {e}");
            (synthetic_result(twin_id, agent_id, n_candidates), SimOutcome::TwinUnavailable)
        }
    }
}

/// Compute the Merkle root and proof-of-sim from OSOVM traj_hashes.
pub fn compute_receipt_fields(result: &OsovmSimResult) -> (String, String, String) {
    let refs: Vec<&str> = result.traj_hashes.iter().map(|s| s.as_str()).collect();
    let merkle_root  = swarm_types::sim_receipt::merkle_root(&refs);
    let policy_hash  = sha256_hex(result.policy_json.to_string().as_bytes());
    let proof_of_sim = sha256_hex(
        format!("{}:{}:{}", merkle_root, policy_hash,
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs()
        ).as_bytes()
    );
    (merkle_root, policy_hash, proof_of_sim)
}

fn synthetic_result(twin_id: &str, _agent_id: &str, n: u32) -> OsovmSimResult {
    let winning = uuid_v4();
    let hashes: Vec<String> = (0..n.min(8))
        .map(|_| sha256_hex(uuid_v4().as_bytes()))
        .collect();
    OsovmSimResult {
        traj_hashes:     hashes,
        winning_traj_id: winning,
        winner_score:    0.0,
        n_feasible:      0,
        policy_json:     json!({"note": "synthetic — OSOVM unavailable", "twin_id": twin_id}),
    }
}

fn uuid_v4() -> String {
    use std::time::{SystemTime, UNIX_EPOCH};
    let n = SystemTime::now().duration_since(UNIX_EPOCH).unwrap_or_default().subsec_nanos();
    format!("{:08x}-0000-4000-8000-{:012x}", n, n as u64 * 0x1234567890ab)
}
