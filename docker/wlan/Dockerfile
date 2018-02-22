FROM jeguzzi/ros:armhf-kinetic-ros-dev
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    iptables \
    net-tools \
    ifupdown \
    && rm -rf /var/lib/apt/lists/*
