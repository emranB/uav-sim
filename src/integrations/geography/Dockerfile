ARG TERRAIN_IMAGE=uav-sim-terrain:development
FROM ${TERRAIN_IMAGE}
USER root
RUN python -m pip install --no-cache-dir shapely==2.1.2
USER app
ENTRYPOINT ["python", "-m", "integrations.geography.buildings"]
