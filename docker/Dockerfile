FROM ros:foxy

ENV DEBIAN_FRONTEND noninteractive
ENV WORKSPACE=/workspace

# Install dependencies
RUN apt-get -qq update && \
    apt-get -qq upgrade && \
    apt-get -qq install apt-utils build-essential vim glpk-utils ros-foxy-mavros python3-pip

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

RUN pip install pulp rx scikit-learn

# Clear apt cache to save on space
RUN rm -rf /var/lib/apt/lists/*

RUN mkdir -p $WORKSPACE

# Create required logging directory
RUN mkdir /root/.ros/log

# Install dragonfly controller
COPY . $WORKSPACE

RUN cd $WORKSPACE && rm -rf docker build devel && \
	rm -rf src/dragonfly_dashboard src/dragonfly_messages_java

RUN source /opt/ros/foxy/setup.bash && \
	cd $WORKSPACE && \
    colcon build

# Setup Entrypoint
COPY ./docker/entrypoint.sh /
ENTRYPOINT ["/entrypoint.sh"]


CMD ["bash"]