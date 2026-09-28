FROM rust:1-slim-bookworm AS builder
WORKDIR /build

RUN apt-get update && apt-get install -y --no-install-recommends \
        pkg-config libssl-dev ca-certificates \
    && rm -rf /var/lib/apt/lists/*

COPY . .
RUN cargo build --release --package swarm-broker

FROM debian:bookworm-slim AS runtime
WORKDIR /app

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates libssl3 \
    && rm -rf /var/lib/apt/lists/* \
    && useradd --create-home --uid 10001 scarab

COPY --from=builder /build/target/release/swarm-broker /usr/local/bin/swarm-broker

USER scarab
EXPOSE 7793
ENTRYPOINT ["/usr/local/bin/swarm-broker"]
