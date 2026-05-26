FROM ghcr.io/screamlab/wildbot_base_image:latest
ENV ROS_DISTRO jazzy
COPY . /tmp

WORKDIR /tmp

### list the ros2 packages you want here
#copy the workspaces and install ros packages and dependencies
RUN cp -r ./workspaces/* /workspaces && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
        ros-${ROS_DISTRO}-demo-nodes-py \
        ros-${ROS_DISTRO}-demo-nodes-cpp \
        ros-${ROS_DISTRO}-foxglove-bridge \
        ros-${ROS_DISTRO}-nav2-bringup && \
    apt-get install -y --only-upgrade \
        ros-${ROS_DISTRO}-fastcdr \
        ros-${ROS_DISTRO}-fastrtps \
        ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
        ros-${ROS_DISTRO}-rmw-fastrtps-shared-cpp \
        ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-cpp \
        ros-${ROS_DISTRO}-rosidl-typesupport-fastrtps-c && \
    rosdep update --rosdistro ${ROS_DISTRO} && \
    colcon mixin update && \
    colcon metadata update


#build the workspace
WORKDIR /workspaces
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    rosdep install -q -y -r --from-paths src --ignore-src && \
    colcon build --symlink-install


RUN rm -rf /tmp/* && \
    rm -rf /temp/* && \
    rm -rf /var/lib/apt/lists/*


ENTRYPOINT ["/ros_entrypoint.bash"]
CMD ["bash", "-l"]


