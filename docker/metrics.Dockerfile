FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    bash \
    coreutils \
    && rm -rf /var/lib/apt/lists/*

ENV HEARTBEAT_DIR=/state \
    ALIVE_WINDOW_SEC=5

WORKDIR /metrics

HEALTHCHECK --interval=10s --timeout=5s --start-period=5s --retries=3 \
    CMD test -d "$HEARTBEAT_DIR" || exit 1

CMD ["bash", "-lc", "echo '[Metrics] Monitoring agent heartbeats'; while true; do now=$(date +%s); echo '--- metrics ---'; found=0; for hb in \"$HEARTBEAT_DIR\"/*.heartbeat; do if [ ! -e \"$hb\" ]; then continue; fi; found=1; agent=$(basename \"$hb\" .heartbeat); last=$(cat \"$hb\" 2>/dev/null || echo 0); age=$((now-last)); if [ \"$age\" -le \"$ALIVE_WINDOW_SEC\" ]; then state='ALIVE'; else state='STALE'; fi; echo \"$agent: $state (age=${age}s)\"; done; if [ \"$found\" -eq 0 ]; then echo 'no agents reporting yet'; fi; sleep 2; done"]
