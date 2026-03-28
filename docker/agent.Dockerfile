# STAGE 1: Builder
FROM denddron_base AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake pkg-config nlohmann-json3-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /home/app
COPY src ./src
COPY extern ./extern
COPY CMakeLists.txt .

RUN mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build . --target denddron_agent -j$(nproc) && \
    strip denddron_agent || true

# STAGE 2: Runtime
FROM denddron_base

ENV DEBIAN_FRONTEND=noninteractive

COPY --from=builder /home/app/build/denddron_agent /usr/local/bin/denddron_agent

# Use a shell script to pass env vars to the binary
CMD ["sh", "-c", "exec denddron_agent --agent-id ${AGENT_ID:-drone_default} --zenoh-router ${ZENOH_ROUTER_IP:-zenoh_router} --loop-rate ${LOOP_RATE_HZ:-50.0}"]
