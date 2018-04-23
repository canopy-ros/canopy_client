FROM osrf/ros:indigo-desktop

SHELL ["/bin/bash", "-c"]
ENV CATKIN_WS /root/catkin_ws

# Clone Canopy client repository
WORKDIR ${CATKIN_WS}/src
RUN git clone https://github.com/baalexander/rospy_message_converter
RUN git clone https://github.com/canopy-ros/canopy_client

# Install python dependencies
RUN apt-get update && apt-get install -y \
    python-dev \
    python-pip
RUN pip install --upgrade pip
RUN sudo pip install 'tornado>=4.0,<5.0' requests

# Build Canopy client
WORKDIR ${CATKIN_WS}
RUN echo 'source /opt/ros/indigo/setup.bash' >> /.bashrc && \
    source /.bashrc && \
    catkin_make && \
    source devel/setup.bash
WORKDIR ${CATKIN_WS}/src/canopy_client
