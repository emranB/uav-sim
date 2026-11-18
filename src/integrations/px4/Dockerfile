FROM ubuntu:24.04 AS base
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg locales git build-essential cmake ninja-build pkg-config \
    python3 python3-venv python3-dev libeigen3-dev libxml2-dev libssl-dev \
    libgl1-mesa-dri libegl1 libgl1 libxkbcommon-x11-0 libxcb-cursor0 \
    libxcb-icccm4 libxcb-image0 libxcb-keysyms1 libxcb-render-util0 libxcb-xinerama0 \
    libxcb-xkb1 libdbus-1-3 libx11-6 libxext6 libxrender1 libxi6 libasound2t64 \
    libnss3 libxcomposite1 libxdamage1 libxtst6 libxrandr2 libpulse0 libfuse2t64 \
    libgstreamer1.0-0 libgstreamer-plugins-base1.0-0 fonts-dejavu-core \
    xdotool x11-utils mesa-utils unzip && \
    locale-gen en_US.UTF-8 && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable noble main' \
      > /etc/apt/sources.list.d/gazebo.list && \
    curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main' \
      > /etc/apt/sources.list.d/ros2.list && \
    apt-get update && apt-get install -y --no-install-recommends \
      gz-harmonic ros-jazzy-ros-base ros-jazzy-ros-gz-bridge && \
    rm -rf /var/lib/apt/lists/*
ENV LANG=en_US.UTF-8 LC_ALL=en_US.UTF-8
RUN python3 -m venv --system-site-packages /opt/venv && \
    /opt/venv/bin/pip install --no-cache-dir \
      PySide6-Essentials==6.8.3 pymavlink==2.4.49 empy==3.3.4 \
      jinja2 jsonschema kconfiglib packaging pyros-genmsg pyyaml toml \
      numpy==1.26.4 lxml setuptools wheel sympy pyserial psutil pyulog \
      argcomplete cerberus future lark nunavut pkgconfig pycryptodome pygments requests six
ENV PATH=/opt/venv/bin:$PATH PYTHONPATH=/workspace/src
WORKDIR /workspace

FROM base AS firmware
ARG PX4_VERSION=v1.17.0
RUN apt-get update && apt-get install -y --no-install-recommends libopencv-dev && \
    rm -rf /var/lib/apt/lists/*
RUN git clone --depth 1 --branch ${PX4_VERSION} https://github.com/PX4/PX4-Autopilot.git /opt/px4 && \
    cd /opt/px4 && git submodule update --init --recursive --depth 1 \
      src/modules/mavlink/mavlink src/drivers/gps/devices \
      src/lib/events/libevents src/modules/uxrce_dds_client/Micro-XRCE-DDS-Client \
      src/lib/heatshrink/heatshrink Tools/simulation/gz
COPY src/integrations/px4/patches/ /tmp/px4-patches/
RUN git -C /opt/px4 apply --check /tmp/px4-patches/0001-fix-posix-periodic-timer-requeue.patch && \
    git -C /opt/px4 apply /tmp/px4-patches/0001-fix-posix-periodic-timer-requeue.patch
RUN cd /opt/px4 && make px4_sitl_default -j2
RUN test "$(git -C /opt/px4 rev-parse HEAD)" = d6f12ad1c4f70ad3230afd7d86e971421e02fef4 && \
    git -C /opt/px4 rev-parse HEAD > /opt/px4/build/px4_sitl_default/source-revision && \
    git -C /opt/px4/Tools/simulation/gz rev-parse HEAD > /opt/px4/build/px4_sitl_default/models-revision && \
    sha256sum /tmp/px4-patches/*.patch > /opt/px4/build/px4_sitl_default/firmware-patches.sha256

FROM base AS runtime
COPY --from=firmware /opt/px4/build/px4_sitl_default /opt/px4/build/px4_sitl_default
COPY --from=firmware /opt/px4/Tools/simulation/gz /opt/px4/Tools/simulation/gz
ENV GZ_SIM_RESOURCE_PATH=/opt/px4/Tools/simulation/gz/models:/opt/px4/Tools/simulation/gz/worlds \
    GZ_PARTITION=uav-sim GZ_IP=127.0.0.1
RUN useradd --uid 1001 --create-home app && mkdir -p /workspace/outputs /runtime && \
    chown -R app:app /workspace/outputs /runtime /opt/px4/build/px4_sitl_default
RUN apt-get update && apt-get install -y --no-install-recommends libqt5svg5 && \
    rm -rf /var/lib/apt/lists/*
COPY src /workspace/src
USER app
CMD ["python", "-m", "integrations.px4.launcher"]

FROM base AS qgroundcontrol
ARG QGC_VERSION=v5.1.5
ARG QGC_SHA256=9a47e4cf269d9e4f897f582f6b19aa7ebf1e9e67c9d68a532a45b541354bc344
RUN curl -fL --retry 3 https://github.com/mavlink/qgroundcontrol/releases/download/${QGC_VERSION}/QGroundControl-x86_64.AppImage \
      -o /tmp/QGroundControl.AppImage && \
    echo "${QGC_SHA256}  /tmp/QGroundControl.AppImage" | sha256sum -c - && \
    chmod +x /tmp/QGroundControl.AppImage && cd /opt && \
    /tmp/QGroundControl.AppImage --appimage-extract > /dev/null && \
    mv squashfs-root qgroundcontrol && rm /tmp/QGroundControl.AppImage && \
    useradd --uid 1001 --create-home app && mkdir /runtime && chown app:app /runtime
RUN chmod -R a+rX /opt/qgroundcontrol && usermod -aG dialout app
COPY src /workspace/src
USER app
CMD ["python", "-m", "qgroundcontrol.launcher"]

FROM base AS ros2
RUN source /opt/ros/jazzy/setup.bash && \
    git clone --depth 1 --branch v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /opt/agent && \
    cmake -S /opt/agent -B /opt/agent/build \
      -DUAGENT_USE_SYSTEM_FASTDDS=ON -DUAGENT_USE_SYSTEM_FASTCDR=ON \
      -DUAGENT_USE_SYSTEM_LOGGER=OFF -DUAGENT_P2P_PROFILE=OFF && \
    cmake --build /opt/agent/build -j2 && cmake --install /opt/agent/build && ldconfig
RUN apt-get update && apt-get install -y --no-install-recommends python3-colcon-common-extensions && \
    rm -rf /var/lib/apt/lists/* && \
    source /opt/ros/jazzy/setup.bash && mkdir -p /opt/ros-workspace/src && \
    git clone --depth 1 --branch release/1.17 https://github.com/PX4/px4_msgs.git /opt/ros-workspace/src/px4_msgs && \
    git -C /opt/ros-workspace/src/px4_msgs fetch --depth 1 origin 86d8239e962f6939e05c3737784f60c02fa884db && \
    git -C /opt/ros-workspace/src/px4_msgs checkout 86d8239e962f6939e05c3737784f60c02fa884db && \
    cd /opt/ros-workspace && colcon build --cmake-args -DBUILD_TESTING=OFF \
      -DPython3_EXECUTABLE=/usr/bin/python3 -DPYTHON_EXECUTABLE=/usr/bin/python3
COPY src /workspace/src
CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && MicroXRCEAgent udp4 -p 8888"]
