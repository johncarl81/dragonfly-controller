FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND noninteractive
ENV WORKSPACE=/workspace

# Install dependencies
RUN apt-get -qq update && \
    apt-get -qq upgrade && \
    apt-get -qq install apt-utils build-essential vim glpk-utils ros-humble-mavros python3-pip

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN pip install pulp rx scikit-learn

# Fix the setuptools version to remove the "SetuptoolsDeprecationWarning: setup.py install is deprecated. \
# Use build and pip and other standards-based tools." Warning print out
RUN pip install setuptools==58.2.0

# Clear apt cache to save on space
RUN rm -rf /var/lib/apt/lists/*

RUN mkdir -p $WORKSPACE

# Create required logging directory
RUN mkdir /root/.ros/log

# Install dragonfly controller
COPY . $WORKSPACE

RUN cd $WORKSPACE && rm -rf docker build devel && \
	rm -rf src/dragonfly_dashboard src/dragonfly_messages_java

RUN source /opt/ros/humble/setup.bash && \
	cd $WORKSPACE && \
    colcon build

# Setup Entrypoint
COPY ./docker/entrypoint.sh /
COPY ./docker/entrypoint.py /

ENTRYPOINT ["/entrypoint.py"]
