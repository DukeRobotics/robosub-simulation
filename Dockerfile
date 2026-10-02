FROM ros:jazzy

ENV DEBIAN_FRONTEND=noninteractive
ARG BUILD_JOBS=4
# Stonefish and its ROS interface must share the same major/minor version (1.6).
ARG STONEFISH_ROS2_REF=6646e7ac25eed982f37f807abc7b545dbd2d6648

RUN apt-get update && apt-get install -y \
    --no-install-recommends \
    build-essential cmake git patch pkg-config \
    libglm-dev libsdl2-dev libfreetype6-dev libgl1-mesa-dev \
    python3-colcon-common-extensions python3-yaml python3-pytest \
    ros-jazzy-image-transport ros-jazzy-pcl-conversions \
    ros-jazzy-tf2-ros ros-jazzy-ros2launch \
    && rm -rf /var/lib/apt/lists/*

COPY stonefish/ /opt/stonefish/
COPY patches/stonefish-ocean-shading.patch /tmp/stonefish-ocean-shading.patch
RUN patch --batch -p1 -d /opt/stonefish < /tmp/stonefish-ocean-shading.patch \
    && cmake -S /opt/stonefish -B /opt/stonefish/build -DBUILD_TESTS=OFF \
    && cmake --build /opt/stonefish/build --parallel ${BUILD_JOBS} \
    && cmake --install /opt/stonefish/build \
    && ldconfig

WORKDIR /workspace
RUN git init src/stonefish_ros2 \
    && git -C src/stonefish_ros2 remote add origin https://github.com/patrykcieslak/stonefish_ros2.git \
    && git -C src/stonefish_ros2 fetch --depth 1 origin ${STONEFISH_ROS2_REF} \
    && git -C src/stonefish_ros2 checkout --detach FETCH_HEAD
RUN . /opt/ros/jazzy/setup.sh \
    && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --merge-install --packages-select stonefish_ros2 \
       --cmake-args -DBUILD_TESTING=OFF
COPY src/robosub_stonefish/ src/robosub_stonefish/
COPY src/robosub_simulation/ src/robosub_simulation/
RUN . /opt/ros/jazzy/setup.sh \
    && . /workspace/install/setup.sh \
    && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --merge-install --executor sequential \
       --packages-select robosub_stonefish robosub_simulation \
       --cmake-args -DBUILD_TESTING=OFF

# Runtime drivers for the graphical simulator, including software OpenGL.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libgl1-mesa-dri libglx-mesa0 libegl-mesa0 \
    xvfb x11vnc novnc websockify openbox x11-utils xdotool mesa-utils curl \
    && rm -rf /var/lib/apt/lists/*

COPY docker/entrypoint.sh /simulation_entrypoint.sh
COPY patches/novnc-close-status.patch /tmp/novnc-close-status.patch
RUN patch --batch -p1 -d /usr/share/novnc < /tmp/novnc-close-status.patch
COPY docker/browser_viewer.py /opt/simulation/browser_viewer.py
COPY docker/openbox.xml /opt/simulation/openbox.xml
COPY viewer/ /opt/simulation/viewer/
RUN ln -s /usr/share/novnc /opt/simulation/viewer/novnc
EXPOSE 8080
HEALTHCHECK --interval=10s --timeout=3s --start-period=30s \
    CMD curl --fail --silent http://127.0.0.1:8080/ > /dev/null || exit 1
ENTRYPOINT ["bash", "/simulation_entrypoint.sh"]
CMD ["python3", "/opt/simulation/browser_viewer.py"]
