FROM aiplanning/planutils:latest

# Install solvers and tools
RUN planutils install -y val
RUN planutils install -y ff
RUN planutils install -y metric-ff
RUN planutils install -y enhsp
RUN planutils install -y popf
RUN planutils install -y optic
RUN planutils install -y tfd
RUN planutils install -y downward

# Handle Non Deterministic Case
RUN planutils install -y prp


# Modify the configuration file to enable hostfs, i.e. use the host file system
RUN perl -pi.bak -e "s/mount hostfs = no/mount hostfs = yes/g" /etc/apptainer/apptainer.conf

# Install Java 8 (Required for PANDA Planner) and Configure Locales
RUN apt-get update && \
    apt-get install -y openjdk-8-jre-headless locales && \
    locale-gen en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*

# Set Java 8 as the default Java environment
RUN update-alternatives --set java /usr/lib/jvm/java-8-openjdk-amd64/jre/bin/java || true

# Environment variables for UTF-8 support
ENV LANG=en_US.UTF-8
ENV LANGUAGE=en_US:en
ENV LC_ALL=en_US.UTF-8

CMD /bin/bash


# To build the docker image
# docker build --rm  --tag myplauntils . --file Dockerfile

# To to run the built image
# docker run  -v.:/computer -it --privileged --rm myplanutils bash

# Alternatively, you can use the following command to run the image if the above command does not work
# docker run  -v.:/computer -it --privileged --rm docker.io/library/myplauntils bash

# With -v option, you can mount any folder of the host computer to the container,
# e.g. docker run -v /home/user:/computer -it --privileged --rm myplanutils bash
# This will mount the /home/user folder of the host computer to the /computer folder of the container