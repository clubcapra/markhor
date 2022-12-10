FROM markhor-base

# Install dependencies
RUN rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN /bin/bash -c '. /opt/ros/melodic/setup.bash; cd  /root/markhor_ws; catkin_make'

COPY scripts/ros_entrypoint.sh /ros_entrypoint.sh

# Source the setup file
RUN sed -i \
    's/ros_env_setup="\/opt\/ros\/$ROS_DISTRO\/setup.bash"/ros_env_setup="${ROS_ROOT}\/devel\/setup.bash"/g' \
    /ros_entrypoint.sh && \
    cat /ros_entrypoint.sh


RUN echo "source /root/markhor_ws/devel/setup.bash" >> ~/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]