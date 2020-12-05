FROM ros:melodic

ENV DEBIAN_FRONTEND noninteractive
ENV WORKSPACE=/workspace

# Install dependencies
RUN apt-get -qq update && \
    apt-get -qq upgrade && \
    apt-get -qq install apt-utils build-essential vim python-catkin-tools

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN apt-get -qq install ros-melodic-mavros ros-melodic-mavros-extras python-pip
#RUN sudo apt-get install -y \
#	ros-melodic-tf \
#	ros-melodic-angles \
#	ros-melodic-gazebo-msgs \
#	ros-melodic-robot-localization \
#	ros-melodic-stereo-image-proc \
#	ros-melodic-gazebo-ros \
#	ros-melodic-geometry2 \
#	python-pip

RUN pip install pulp rx

# Clear apt cache to save on space
RUN rm -rf /var/lib/apt/lists/*

RUN mkdir -p $WORKSPACE

# Install dragonfly controller
COPY . $WORKSPACE

RUN cd $WORKSPACE && rm -rf docker build devel && \
	rm -rf src/dragonfly_dashboard src/dragonfly_messages_java

RUN source /opt/ros/melodic/setup.bash && \
	cd $WORKSPACE && \
    catkin_make && \
    catkin_make install

# Setup Entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]

CMD ["bash"]