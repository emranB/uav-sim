ARG UAV_VISION_STACK=uav-sim-stack:development
FROM ${UAV_VISION_STACK} AS vision_dependencies
USER root
RUN python -m pip install --no-cache-dir numpy==1.26.4 opencv-python-headless==4.10.0.84
CMD ["python", "--version"]

FROM vision_dependencies
COPY src/common /workspace/src/common
COPY src/integrations/vision /workspace/src/integrations/vision
USER app
CMD ["python", "-m", "integrations.vision.stereo_depth", "--help"]
