FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    bash \
    coreutils \
    && rm -rf /var/lib/apt/lists/*

ENV AGENT_ID=drone_default \
    HEARTBEAT_DIR=/state \
    HEARTBEAT_INTERVAL=2

HEALTHCHECK --interval=10s --timeout=5s --start-period=5s --retries=3 \
    CMD test -f "${HEARTBEAT_DIR}/${AGENT_ID}.heartbeat" || exit 1

CMD ["bash", "-lc", "mkdir -p \"$HEARTBEAT_DIR\"; echo \"[Agent] Starting $AGENT_ID (placeholder)\"; while true; do date +%s > \"$HEARTBEAT_DIR/$AGENT_ID.heartbeat\"; echo \"[$(date -Iseconds)] $AGENT_ID alive\"; sleep \"$HEARTBEAT_INTERVAL\"; done"]