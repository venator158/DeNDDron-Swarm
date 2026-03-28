# ============================================================================
# STAGE 1: Builder
# ============================================================================
FROM denddron_base AS builder

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential cmake git wget pkg-config libssl-dev \
    libcurl4-openssl-dev libjsoncpp-dev libgazebo-dev gazebo \
    nlohmann-json3-dev && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /home/app
COPY . .
RUN mkdir -p build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release .. && \
    cmake --build . --target gazebo_simulator

# ============================================================================
# STAGE 2: Runtime
# ============================================================================
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    gazebo \
    libgazebo-dev \
    libcurl4 \
    libjsoncpp25 \
    libssl3 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/local/lib/libzenohc.* /usr/local/lib/
COPY --from=builder /usr/local/include/zenoh* /usr/local/include/
RUN ldconfig

COPY --from=builder /home/app/build/sim/gazebo_simulator /usr/local/bin/gazebo_simulator
COPY --from=builder /home/app/sim /home/app/sim

WORKDIR /home/app
EXPOSE 11345

CMD echo "[Container] Starting gzserver..." && \
    gzserver --verbose /home/app/sim/ocean.world &> /tmp/gzserver.log & \
    sleep 3 && \
    echo "[Container] Starting gazebo_simulator bridge..." && \
    gazebo_simulator
