FROM ubuntu:18.04

LABEL maintainer="P.A.Tersmette@student.tudelft.nl"
LABEL Description="Dockerised Simulation of Experiment using SUMO"

ENV SUMO_VERSION 1.6.0
ENV SUMO_HOME /opt/sumo
ENV SUMO_USER research_project

# Install system dependencies.
RUN apt-get update && apt-get install -qq -y \
    wget \
    cmake \
    python \
    g++ \
    libxerces-c-dev \
    libfox-1.6-dev \
    libgdal-dev \
    libproj-dev \
    libgl2ps-dev \
    swig

# Download and extract source code.
RUN wget http://downloads.sourceforge.net/project/sumo/sumo/version%20$SUMO_VERSION/sumo-src-$SUMO_VERSION.tar.gz
RUN tar xzf sumo-src-$SUMO_VERSION.tar.gz && \
    mv sumo-$SUMO_VERSION $SUMO_HOME && \
    rm sumo-src-$SUMO_VERSION.tar.gz

# Configure and build from source.
RUN cd $SUMO_HOME && mkdir build/cmake-build && cd build/cmake-build && cmake ../.. && make -j $(nproc)

# Installing the SUMO binaries.
RUN cd $SUMO_HOME && cd build/cmake-build && make install

# Copy experiment files.
COPY \experiment \experiment

# Run experiment.
WORKDIR /experiment
RUN python3 experiment.py