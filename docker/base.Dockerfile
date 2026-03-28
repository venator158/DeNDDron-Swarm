# STAGE 1: Builder (Heavy, temporary)
FROM rust:1.75-slim AS builder

RUN apt-get update && apt-get install -y \
    build-essential cmake git pkg-config libssl-dev libclang-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /tmp

# Build and install zenoh-c
RUN git clone --depth 1 --branch 1.0.4 https://github.com/eclipse-zenoh/zenoh-c.git && \
    cd zenoh-c && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    cmake --build . --parallel $(nproc) && \
    cmake --install .

# Build and install zenoh-cpp
RUN git clone --depth 1 --branch 1.0.4 https://github.com/eclipse-zenoh/zenoh-cpp.git && \
    cd zenoh-cpp && \
    mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local -DZENOHC_CUSTOM_PREFIX=/usr/local .. && \
    cmake --build . --parallel $(nproc) && \
    cmake --install .

# STAGE 2: Runtime (The "Shared" C++ Base image)
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Minimal runtime libs
RUN apt-get update && apt-get install -y --no-install-recommends \
    libssl3 \
    libstdc++6 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Copy ONLY the shared objects, headers, and CMake config for zenoh-cpp and zenoh-c
COPY --from=builder /usr/local/lib/ /usr/local/lib/
COPY --from=builder /usr/local/include/ /usr/local/include/
COPY --from=builder /usr/local/share/ /usr/local/share/

# Refresh linker cache
RUN ldconfig
