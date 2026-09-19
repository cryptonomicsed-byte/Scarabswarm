use chrono::{DateTime, Utc};
use gix_types::{Gix1, GixKind, GixNamespace, RoutingHints};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Proof-of-Simulation receipt — cryptographic commitment to a simulation run.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SimReceipt {
    pub receipt_id:       String,
    pub twin_id:          String,
    pub agent_id:         String,
    pub session_id:       Option<String>,

    pub n_trajectories:   u32,
    pub n_feasible:       u32,
    pub winning_traj_id:  String,
    pub winner_score:     f64,

    /// SHA-256 Merkle root over all trajectory sim_hashes
    pub merkle_root:      String,
    /// SHA-256 of the selected policy serialized to JSON
    pub policy_hash:      String,
    /// Proof-of-Simulation commitment: SHA-256(merkle_root || policy_hash || timestamp)
    pub proof_of_sim:     String,

    pub outcome:          SimOutcome,
    pub zangbeto_anchor:  Option<String>,
    pub witness_event_id: Option<String>,

    pub created_at:       DateTime<Utc>,
    pub signature:        String,

    /// GIX1 canonical_id (hex) — `Gix1(Simulation, OsovmExecution, receipt_id)`.
    /// Stamped after construction via `stamp_gix1()`.
    #[serde(default)]
    pub gix1_canonical_id: Option<String>,
}

impl SimReceipt {
    /// Stamp a GIX1 Simulation envelope onto this receipt (idempotent).
    pub fn stamp_gix1(&mut self) {
        if self.gix1_canonical_id.is_some() { return; }
        let ts = self.created_at.timestamp_millis() as u64;
        let env = Gix1::new(
            GixKind::Simulation,
            GixNamespace::OsovmExecution,
            self.receipt_id.as_bytes(),
            None,
            ts,
            RoutingHints::default(),
        );
        self.gix1_canonical_id = Some(hex::encode(env.canonical_id));
    }

    pub fn canonical_hash(&self) -> String {
        let data = format!(
            "{}:{}:{}:{}:{}",
            self.receipt_id, self.twin_id, self.merkle_root,
            self.policy_hash, self.created_at.timestamp()
        );
        sha256_hex(data.as_bytes())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum SimOutcome {
    PolicySelected,
    NoFeasiblePolicy,
    TwinUnavailable,
    AgentRevoked,
}

/// Wrapper for the 5-primitive proof chain commitment.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ProofOfSimulation {
    pub proof_id:    String,
    pub sim_receipt: SimReceipt,
    pub vcp_receipt: Option<serde_json::Value>,
    pub arp_receipt: Option<serde_json::Value>,
}

// ── Merkle proof tree ─────────────────────────────────────────────────────────

/// Build a SHA-256 Merkle root from a list of leaf hashes (hex strings).
/// Leaves are sorted for canonical ordering. Pairs reduced pairwise.
pub fn merkle_root(leaf_hashes: &[&str]) -> String {
    if leaf_hashes.is_empty() {
        return sha256_hex(b"empty");
    }

    let mut level: Vec<Vec<u8>> = {
        let mut sorted = leaf_hashes.to_vec();
        sorted.sort_unstable();
        sorted.iter().map(|h| hex::decode(h).unwrap_or_else(|_| sha256_bytes(h.as_bytes()).to_vec())).collect()
    };

    while level.len() > 1 {
        let mut next = Vec::with_capacity((level.len() + 1) / 2);
        let mut i = 0;
        while i < level.len() {
            let left = &level[i];
            let right = if i + 1 < level.len() { &level[i + 1] } else { &level[i] };
            let mut combined = left.clone();
            combined.extend_from_slice(right);
            next.push(sha256_bytes(&combined).to_vec());
            i += 2;
        }
        level = next;
    }

    hex::encode(&level[0])
}

/// Build a Merkle proof (inclusion proof) for a leaf at index `idx`.
/// Returns the sibling hashes from leaf to root.
pub fn merkle_proof(leaf_hashes: &[String], idx: usize) -> Vec<String> {
    if leaf_hashes.is_empty() || idx >= leaf_hashes.len() { return Vec::new(); }

    let mut level: Vec<String> = {
        let mut indexed: Vec<(usize, String)> = leaf_hashes.iter().cloned().enumerate().collect();
        indexed.sort_by_key(|(_, h)| h.clone());
        // Track new position of our target leaf after sort
        indexed.into_iter().map(|(_, h)| h).collect()
    };

    let mut proof = Vec::new();
    let mut target = idx;

    while level.len() > 1 {
        let sibling = if target % 2 == 0 {
            level.get(target + 1).cloned().unwrap_or_else(|| level[target].clone())
        } else {
            level[target - 1].clone()
        };
        proof.push(sibling);

        level = level.chunks(2).map(|pair| {
            let left  = &pair[0];
            let right = pair.get(1).unwrap_or(&pair[0]);
            let mut combined = hex::decode(left).unwrap_or_default();
            combined.extend_from_slice(&hex::decode(right).unwrap_or_default());
            hex::encode(sha256_bytes(&combined))
        }).collect();

        target /= 2;
    }

    proof
}

// ── tests ─────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod gix_tests {
    use super::*;
    use chrono::Utc;

    fn make_receipt() -> SimReceipt {
        SimReceipt {
            receipt_id:       "sim-test-001".into(),
            twin_id:          "twin-abc".into(),
            agent_id:         "agent-xyz".into(),
            session_id:       None,
            n_trajectories:   100,
            n_feasible:       42,
            winning_traj_id:  "traj-07".into(),
            winner_score:     0.91,
            merkle_root:      "a".repeat(64),
            policy_hash:      "b".repeat(64),
            proof_of_sim:     "c".repeat(64),
            outcome:          SimOutcome::PolicySelected,
            zangbeto_anchor:  None,
            witness_event_id: None,
            created_at:       Utc::now(),
            signature:        String::new(),
            gix1_canonical_id: None,
        }
    }

    #[test]
    fn stamp_gix1_sets_canonical_id() {
        let mut r = make_receipt();
        r.stamp_gix1();
        let id = r.gix1_canonical_id.as_ref().expect("gix1_canonical_id must be set");
        assert_eq!(id.len(), 64);
    }

    #[test]
    fn stamp_gix1_is_idempotent() {
        let mut r = make_receipt();
        r.stamp_gix1();
        let first = r.gix1_canonical_id.clone();
        r.stamp_gix1();
        assert_eq!(r.gix1_canonical_id, first);
    }

    #[test]
    fn two_receipts_have_distinct_gix1_ids() {
        let mut r1 = make_receipt();
        let mut r2 = make_receipt();
        r2.receipt_id = "sim-test-002".into();
        r1.stamp_gix1();
        r2.stamp_gix1();
        assert_ne!(r1.gix1_canonical_id, r2.gix1_canonical_id);
    }
}

// ── helpers ───────────────────────────────────────────────────────────────────

pub fn sha256_hex(data: &[u8]) -> String {
    hex::encode(sha256_bytes(data))
}

fn sha256_bytes(data: &[u8]) -> [u8; 32] {
    let mut h = Sha256::new();
    h.update(data);
    h.finalize().into()
}
