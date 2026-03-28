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
    cmake --build . --target metrics_node -j$(nproc) && \
    strip src/metrics/metrics_node || strip metrics_node || true
# STAGE 2: Runtime
FROM denddron_base
ENV DEBIAN_FRONTEND=noninteractive
COPY --from=builder /home/app/build/src/metrics/metrics_node /usr/local/bin/metrics_node
CMD ["sh", "-c", "exec metrics_node --zenoh-router ${ZENOH_ROUTER_IP:-zenoh_router}"]
