ARG RUNTIME_IMAGE=uav-sim-stack:development
FROM ${RUNTIME_IMAGE}
USER root
RUN apt-get update \
    && apt-get install --no-install-recommends -y ffmpeg \
    && mkdir -p /opt/uav-replay \
    && dpkg-query -W ffmpeg > /opt/uav-replay/ffmpeg-version.txt \
    && rm -rf /var/lib/apt/lists/*
USER app
CMD ["python", "-m", "integrations.blender.worker"]
