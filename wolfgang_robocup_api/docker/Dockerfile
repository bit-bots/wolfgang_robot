# Use upstream ubuntu images as base
FROM nvidia/cuda:11.8.0-devel-ubuntu22.04
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
ADD sources.list /etc/apt/sources.list

# Setup and prioritize packages.bit-bots.de repository
RUN apt-get update -y \
    && apt-get install -y wget apt-utils auto-apt-proxy \
    && mkdir -p /usr/local/share/keyrings \
    && wget https://packages.bit-bots.de/key.asc -O /usr/local/share/keyrings/bitbots.key \
    && wget https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/local/share/keyrings/ros-archive-keyring.gpg \
    && echo 'deb [arch=amd64 signed-by=/usr/local/share/keyrings/bitbots.key] https://packages.bit-bots.de jammy main' | tee /etc/apt/sources.list.d/bitbots.list \
    && echo "deb [arch=amd64 signed-by=/usr/local/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list \
    && echo 'Package: *' >> /etc/apt/preferences.d/package-bit-bots.pref \
    && echo 'Pin: origin "packages.bit-bots.de"' >> /etc/apt/preferences.d/package-bit-bots.pref \
    && echo 'Pin-Priority: 1000' >> /etc/apt/preferences.d/package-bit-bots.pref \
    && apt-get update -y

# Install a lot of apt packages. They could also be installed with rosdep, but we want them to be cached
RUN apt-get install -y \
      build-essential \
      git \
      libjsoncpp-dev \
      libprotobuf-dev \
      libprotoc-dev \
      locales \
      protobuf-compiler \
      python3-colcon-common-extensions \
      python3-colcon-core \
      python3-construct \
      python3-hypothesis \
      python3-numpy \
      python3-opencv \
      python3-pip \
      python3-protobuf \
      python3-psutil \
      python3-rosdep \
      python3-rospkg \
      python3-sklearn \
      python3-sphinx-rtd-theme \
      python3-transforms3d \
      ros-rolling-ament-cmake-nose \
      ros-rolling-camera-info-manager \
      ros-rolling-control-toolbox \
      ros-rolling-cv-bridge \
      ros-rolling-demo-nodes-cpp \
      ros-rolling-gazebo-msgs \
      ros-rolling-image-geometry \
      ros-rolling-image-transport \
      ros-rolling-imu-complementary-filter \
      ros-rolling-map-msgs \
      # As all ros moveit pkgs are currently not available for rolling
      # anymore we instead install only moveit-msgs, backward-ros and 
      # add the moveit2 repo to our lib folder to build manually
      # see: https://discourse.ros.org/t/new-packages-for-ros-2-rolling-ridley-2023-03-17/30365
      # ros-rolling-moveit-core \
      # ros-rolling-moveit-planners-ompl \
      # ros-rolling-moveit-ros-planning \
      # ros-rolling-moveit-ros-planning-interface \
      # ros-rolling-moveit-ros-robot-interaction \
      # ros-rolling-moveit-simple-controller-manager \
      ros-rolling-navigation2 \
      ros-rolling-rmw-cyclonedds-cpp \
      ros-rolling-robot-state-publisher \
      ros-rolling-ros-base \
      ros-rolling-rot-conv \
      ros-rolling-soccer-vision-2d-msgs \
      ros-rolling-soccer-vision-3d-msgs \
      ros-rolling-test-msgs \
      ros-rolling-tf-transformations \
      ros-rolling-tf2 \
      ros-rolling-tf2-sensor-msgs \
      ros-rolling-topic-tools \
      ros-rolling-urdf \
      ros-rolling-vision-msgs \
      ros-rolling-xacro \
      sudo \
      xvfb \
    && apt-get clean #\
    # && rm -rf /var/lib/apt/lists/*

# Set up locale
RUN echo 'en_US.UTF-8 UTF-8' > /etc/locale.gen \
  && locale-gen \
  && update-locale LANG=en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# Add user
ARG UID=150
RUN useradd -m -d /colcon_ws -s /bin/bash -u $UID robot

# Install sudoers file
ADD sudoers /etc/sudoers

WORKDIR /colcon_ws
USER robot:robot
ENV PATH=$PATH:/colcon_ws/.local/bin

ADD --chown=robot:robot https://raw.githubusercontent.com/bit-bots/bitbots_meta/master/requirements.txt src/requirements.txt

RUN sed -i -e /mycroft-mimic3-tts/d -e /pydot/d -e /pyopencl/d -e /pyttsx3/d src/requirements.txt \
    && pip3 install -U -r src/requirements.txt --no-cache-dir \
    && pip3 uninstall -y numpy

RUN git clone https://github.com/Bit-Bots/bitbots_meta.git src/bitbots_meta \
    && cd src/bitbots_meta \
    && make pull-init

# From here on, we don't want to cache anything. That's achieved by adding the current time.
# ADD https://www.timeapi.io/api/Time/current/zone?timeZone=UTC /tmp/build-time

RUN cd src/bitbots_meta \
    && make pull-all \
    && rm -rf \
      bitbots_lowlevel \
      bitbots_misc/bitbots_basler_camera \
      bitbots_misc/bitbots_live_tool_rqt \
      bitbots_motion/bitbots_recordui_rqt \
      bitbots_navigation/bitbots_visual_compass \
      bitbots_tools/bitbots_jenkins_library \
      bitbots_tools/containers \
      dynamic_stack_decider/dynamic_stack_decider_visualization \
      humanoid_league_visualization \
      udp_bridge \
      wolfgang_robot/wolfgang_pybullet_sim \
      lib/DynamixelSDK \
      lib/dynamixel-workbench \
      lib/events-executor/events_executor_examples \
      lib/events-executor/irobot_lock_free_events_queue \
      lib/pylon-ros-camera \
    && sed -i '/plotjuggler/d' bitbots_motion/bitbots_quintic_walk/package.xml \
    && sed -i '/run_depend/d' wolfgang_robot/wolfgang_moveit_config/package.xml \
    && git -C lib/ros2_numpy switch humble

# Install old moveit pkgs
RUN cd /tmp \
    && wget \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-common_2.6.0-1jammy.20230216.004335_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-configs-utils_2.6.0-1jammy.20230302.162217_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-core_2.6.0-1jammy.20230131.043704_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-kinematics_2.6.0-1jammy.20230131.054605_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-msgs_2.2.1-1jammy.20230224.212651_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-planners-ompl_2.6.0-1jammy.20230131.054928_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-ros_2.6.0-1jammy.20230131.062238_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-ros-benchmarks_2.6.0-1jammy.20230131.054920_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-ros-move-group_2.6.0-1jammy.20230131.055024_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-ros-occupancy-map-monitor_2.6.0-1jammy.20230131.051843_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-ros-planning_2.6.0-1jammy.20230131.052128_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-ros-planning-interface_2.6.0-1jammy.20230131.060026_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-ros-robot-interaction_2.6.0-1jammy.20230131.054309_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-ros-visualization_2.6.0-1jammy.20230131.060718_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-ros-warehouse_2.6.0-1jammy.20230131.054447_amd64.deb \
      https://data.bit-bots.de/moveit_pkgs/ros-rolling-moveit-simple-controller-manager_2.6.0-1jammy.20230131.051314_amd64.deb \
    && sudo apt install -yf ./*.deb \
    && rm ./*.deb

# Install moveit deps with rosdep
# RUN git clone https://github.com/ros-planning/moveit2.git src/bitbots_meta/lib/moveit2 \
    # && sudo rosdep init \
    # && rosdep update \
    # && rosdep install -y --from-paths src/bitbots_meta/lib/moveit2 --ignore-src --rosdistro rolling

RUN . /opt/ros/rolling/setup.sh && colcon build --continue-on-error && exit 1

# TODO execute tests
RUN cp src/bitbots_meta/wolfgang_robot/wolfgang_robocup_api/scripts/start.sh .local/bin/start

# Volume for logs
VOLUME /robocup-logs

# Setup runtime
ENV DEBIAN_FRONTEND=readline
ENV ROS_LOCALHOST_ONLY=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY entrypoint.sh /usr/local/bin/entrypoint
ENTRYPOINT ["/usr/local/bin/entrypoint"]
CMD ["bash"]
