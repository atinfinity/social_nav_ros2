# 動画生成専用(sim_demo.mp4 など)。開発イメージとは別。使い方は docs/sim_results.md
FROM ubuntu:24.04
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ffmpeg python3 fonts-dejavu-core \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /work
