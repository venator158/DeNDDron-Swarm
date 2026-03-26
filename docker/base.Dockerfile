# STAGE 1: Builder (Heavy, temporary)
FROM rust:1.75-slim AS builder

RUN apt-get update && apt-get install -y \
    build-essential cmake git pkg-config libssl-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp
RUN git clone --depth 1 --branch 1.0.4 https://github.com/eclipse-zenoh/zenoh-c.git && \
    cd zenoh-c && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    cmake --build . --parallel $(nproc) && \
    cmake --install .

# STAGE 2: Runtime (The "Shared" image)
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Minimal runtime libs
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl3 \
    libstdc++6 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Copy ONLY the shared objects, headers, and CMake config
# Note: Keeping headers + CMake config allows this image to be the 'FROM' for Agent Builders
COPY --from=builder /usr/local/lib/libzenohc.* /usr/local/lib/
COPY --from=builder /usr/local/lib/cmake/zenohc /usr/local/lib/cmake/zenohc
COPY --from=builder /usr/local/include/zenoh* /usr/local/include/

# Refresh linker cache
RUN ldconfig && ldconfig -p | grep zenohc