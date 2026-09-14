//! ScarabSwarm broker ↔ Vantage witness_store wiring.
//!
//! After a simulation run completes, this module opens a Vantage witness
//! round (POST /api/witness/rounds) for the sim_receipt so that peer
//! agents can vote on the Proof-of-Simulation outcome.
//!
//! Fail-open: errors are returned, never panics.

use serde::Deserialize;
use serde_json::json;
use swarm_types::{SimReceipt, sim_receipt::sha256_hex};

fn vantage_base() -> String {
    std::env::var("VANTAGE_URL").unwrap_or_else(|_| "http://127.0.0.1:8000".to_string())
}

fn vantage_key() -> Option<String> {
    std::env::var("VANTAGE_KEY").ok().filter(|s| !s.is_empty())
}

/// Response from POST /api/witness/rounds
#[derive(Debug, Deserialize)]
pub struct WitnessRound {
    pub round_id:     i64,
    pub subject_type: String,
    pub subject_id:   i64,
    pub status:       String,
}

/// Open a Vantage witness round for a completed SimReceipt.
///
/// This makes the Proof-of-Simulation verifiable by the BlockMesh quorum.
pub async fn open_round_for_receipt(receipt: &SimReceipt) -> Result<WitnessRound, String> {
    let base = vantage_base();
    let url = format!("{base}/api/witness/rounds");

    // Derive a stable i64 subject_id from the receipt_id.
    let subject_id = i64::from_str_radix(
        &sha256_hex(receipt.receipt_id.as_bytes())[..12],
        16,
    ).unwrap_or(0).abs();

    let body = json!({
        "subject_type":      format!("sim_receipt:{:?}", receipt.outcome),
        "subject_id":        subject_id,
        "artifact_url":      "",
        "description":       format!(
            "ScarabSwarm sim receipt {} — twin {} agent {} winner_score={:.4}",
            receipt.receipt_id, receipt.twin_id, receipt.agent_id, receipt.winner_score,
        ),
        "sim_receipt_id":    receipt.receipt_id,
        "consensus_output_id": null,
    });

    let client = reqwest::Client::builder()
        .timeout(std::time::Duration::from_secs(10))
        .build()
        .map_err(|e| format!("client build: {e}"))?;

    let mut req = client.post(&url)
        .header("Content-Type", "application/json")
        .json(&body);

    if let Some(key) = vantage_key() {
        req = req.header("Authorization", format!("Bearer {key}"));
    }

    let resp = req.send().await.map_err(|e| format!("POST {url}: {e}"))?;
    if !resp.status().is_success() {
        let status = resp.status();
        let text = resp.text().await.unwrap_or_default();
        return Err(format!("Vantage /api/witness/rounds returned {status}: {text}"));
    }

    resp.json::<WitnessRound>()
        .await
        .map_err(|e| format!("parse WitnessRound: {e}"))
}

/// Store sim receipt metadata in Vantage's proof_of_sim endpoint.
///
/// POST /api/ucx/sim_receipt (or equivalent) so the UCX/OSOVM pipeline
/// can reference this receipt during COMPUTE_PROOF opcode execution.
pub async fn record_sim_receipt(receipt: &SimReceipt) -> Result<(), String> {
    let base = vantage_base();
    let url = format!("{base}/api/ucx/sim_receipt");

    let body = json!({
        "receipt_id":      receipt.receipt_id,
        "twin_id":         receipt.twin_id,
        "agent_id":        receipt.agent_id,
        "n_trajectories":  receipt.n_trajectories,
        "n_feasible":      receipt.n_feasible,
        "winner_score":    receipt.winner_score,
        "proof_of_sim":    receipt.proof_of_sim,
        "merkle_root":     receipt.merkle_root,
        "policy_hash":     receipt.policy_hash,
        "outcome":         receipt.outcome,
        "created_at":      receipt.created_at.to_rfc3339(),
        "canonical_hash":  receipt.canonical_hash(),
    });

    let client = reqwest::Client::builder()
        .timeout(std::time::Duration::from_secs(10))
        .build()
        .map_err(|e| format!("client build: {e}"))?;

    let mut req = client.post(&url)
        .header("Content-Type", "application/json")
        .json(&body);

    if let Some(key) = vantage_key() {
        req = req.header("Authorization", format!("Bearer {key}"));
    }

    req.send().await
        .map_err(|e| format!("POST {url}: {e}"))?
        .error_for_status()
        .map_err(|e| format!("record_sim_receipt: {e}"))?;
    Ok(())
}
