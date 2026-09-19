use std::sync::Arc;
use std::collections::HashMap;
use std::sync::RwLock;
use axum::{Router, Json, extract::{State, Path}, routing::{get, post}, http::StatusCode};
use swarm_types::{SimReceipt, SimOutcome};
use swarm_broker::{osovm_delegation, vantage_store};
use uuid::Uuid;
use chrono::Utc;

type AppState = Arc<BrokerState>;

struct BrokerState {
    receipts: RwLock<HashMap<String, SimReceipt>>,
}

impl BrokerState {
    fn new() -> Self { Self { receipts: RwLock::new(HashMap::new()) } }
}

#[tokio::main]
async fn main() {
    tracing_subscriber::fmt::init();

    let state = Arc::new(BrokerState::new());
    let port: u16 = std::env::var("SWARM_PORT")
        .ok().and_then(|p| p.parse().ok())
        .unwrap_or(7793);

    let app = Router::new()
        .route("/health", get(health))
        .route("/api/sim/run",           post(run_simulation))
        .route("/api/sim/receipts",      get(list_receipts))
        .route("/api/sim/receipts/:id",  get(get_receipt))
        .with_state(state);

    let addr = format!("0.0.0.0:{port}");
    tracing::info!("ScarabSwarm broker listening on {addr}");
    let listener = tokio::net::TcpListener::bind(&addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

async fn health() -> Json<serde_json::Value> {
    Json(serde_json::json!({"status": "ok", "service": "swarm-broker"}))
}

#[derive(serde::Deserialize)]
struct SimRequest {
    twin_id:      String,
    agent_id:     String,
    session_id:   Option<String>,
    n_candidates: Option<u32>,
}

/// Run simulation: delegates to OSOVM veilsim_engine (fail-open).
async fn run_simulation(
    State(s): State<AppState>,
    Json(req): Json<SimRequest>,
) -> (StatusCode, Json<serde_json::Value>) {
    let n = req.n_candidates.unwrap_or(100);
    let receipt_id = Uuid::new_v4().to_string();

    let (osovm_result, outcome) = osovm_delegation::run_via_osovm(
        &req.twin_id,
        &req.agent_id,
        n,
        req.session_id.as_deref(),
    ).await;

    let (merkle_root, policy_hash, proof_of_sim) =
        osovm_delegation::compute_receipt_fields(&osovm_result);

    let mut receipt = SimReceipt {
        receipt_id:       receipt_id.clone(),
        twin_id:          req.twin_id.clone(),
        agent_id:         req.agent_id.clone(),
        session_id:       req.session_id.clone(),
        n_trajectories:   n,
        n_feasible:       osovm_result.n_feasible,
        winning_traj_id:  osovm_result.winning_traj_id.clone(),
        winner_score:     osovm_result.winner_score,
        merkle_root:      merkle_root.clone(),
        policy_hash:      policy_hash.clone(),
        proof_of_sim:     proof_of_sim.clone(),
        outcome:          outcome.clone(),
        zangbeto_anchor:  None,
        witness_event_id: None,
        created_at:       Utc::now(),
        signature:        String::new(),
        gix1_canonical_id: None,
    };
    receipt.stamp_gix1();

    let hash = receipt.canonical_hash();
    s.receipts.write().unwrap().insert(receipt_id.clone(), receipt.clone());

    // Fail-open: wire receipt to Vantage witness_store (Gap — ScarabSwarm/Witness ↔ Vantage)
    {
        let r = receipt.clone();
        tokio::spawn(async move {
            if let Err(e) = vantage_store::record_sim_receipt(&r).await {
                tracing::debug!(receipt_id = %r.receipt_id, "record_sim_receipt: {e}");
            }
            if let Err(e) = vantage_store::open_round_for_receipt(&r).await {
                tracing::debug!(receipt_id = %r.receipt_id, "open_round_for_receipt: {e}");
            }
        });
    }

    let status = if outcome == SimOutcome::PolicySelected {
        StatusCode::CREATED
    } else {
        StatusCode::OK
    };

    (status, Json(serde_json::json!({
        "receipt_id":    receipt_id,
        "winning_traj":  osovm_result.winning_traj_id,
        "proof_of_sim":  proof_of_sim,
        "canonical_hash": hash,
        "merkle_root":   merkle_root,
        "n_trajectories": n,
        "outcome": format!("{:?}", receipt.outcome),
    })))
}

async fn list_receipts(State(s): State<AppState>) -> Json<Vec<serde_json::Value>> {
    let receipts = s.receipts.read().unwrap();
    Json(receipts.values().map(|r| serde_json::to_value(r).unwrap_or_default()).collect())
}

async fn get_receipt(
    State(s): State<AppState>,
    Path(id): Path<String>,
) -> (StatusCode, Json<serde_json::Value>) {
    match s.receipts.read().unwrap().get(&id).cloned() {
        Some(r) => (StatusCode::OK, Json(serde_json::to_value(r).unwrap_or_default())),
        None    => (StatusCode::NOT_FOUND, Json(serde_json::json!({"error": "not found"}))),
    }
}
