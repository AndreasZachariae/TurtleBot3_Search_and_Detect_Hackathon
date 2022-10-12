##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=foxy
FROM ros:$ROS_DISTRO-ros-base
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN rosdep update --rosdistro $ROS_DISTRO

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
RUN apt-get update && apt-get install --no-install-recommends -y \
    python3-pip \
    ros-$ROS_DISTRO-behaviortree-cpp-v3 \
    ros-$ROS_DISTRO-nav2-msgs \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-gazebo-msgs \
    qtbase5-dev \
    libqt5svg5-dev \
    libzmq3-dev \
    libdw-dev \
    libqt5opengl5-dev \
    qttools5-dev-tools \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install -U pip setuptools

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=docker
ARG PASSWORD=neobotix
ARG UID=1000
ARG GID=1000
ARG DOMAIN_ID=1
ENV UID=$UID
ENV GID=$GID
ENV USER=$USER
ENV ROS_DOMAIN_ID=$DOMAIN_ID
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

COPY dds_profile.xml /home/$USER
RUN chown $USER:$USER /home/$USER/dds_profile.xml
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/home/$USER/dds_profile.xml

USER $USER 
RUN mkdir -p /home/$USER/ros2_ws/src

##############################################################################
##                                 User Dependecies                         ##
##############################################################################
WORKDIR /home/$USER/ros2_ws/src
RUN git config --global advice.detachedHead false
RUN git clone --depth 1 -b v1.1.0 https://project_55_bot:glpat-DjsyN_ixYnq-duDb_Sip@www.w.hs-karlsruhe.de/gitlab/iras/research-projects/petra/petra_interfaces.git
RUN git clone --depth 1 -b foxy https://project_240_bot:glpat-stK1tgiDxr44VV8X95r7@www.w.hs-karlsruhe.de/gitlab/iras/common/behaviortree_ros.git
RUN git clone --depth 1 -b v1.0.0 https://project_109_bot:glpat-ydJpMPNbrfjXxzYoJbvS@www.w.hs-karlsruhe.de/gitlab/iras/core/cpp_core.git
RUN git clone --depth 1 https://project_29_bot:glpat-gsSiSptRUVJHN8oZiffm@www.w.hs-karlsruhe.de/gitlab/iras/research-projects/petra/behavior-tree/petra_dummies.git
RUN git clone --depth 1 https://github.com/BehaviorTree/Groot.git
COPY ./src ./behavior-tree-coordinator

ENV TURTLEBOT3_MODEL=waffle
ENV GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/$ROS_DISTRO/share/turtlebot3_gazebo/models

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
WORKDIR /home/$USER/ros2_ws
RUN rosdep update --rosdistro $ROS_DISTRO
RUN rosdep install --from-paths src --ignore-src
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install
RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc

RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/ros2_ws/install/setup.bash"' \
    /ros_entrypoint.sh

# CMD ["ros2", "launch", "behavior_tree_coordinator", "test.launch.py"]
# CMD ["ros2", "run", "groot", "Groot"]
CMD /bin/bash