FROM jeguzzi/thymioid
MAINTAINER Jerome Guzzi "jerome@idsia.ch"

RUN apt-get update && apt-get install -y \
    # python-pygame \
    automake \
    libtool \
    && rm -rf /var/lib/apt/lists/*


RUN echo  "deb-src http://gb.archive.ubuntu.com/ubuntu/ xenial main restricted universe multiverse"  >> /etc/apt/sources.list
RUN echo  "deb-src http://gb.archive.ubuntu.com/ubuntu/ xenial-updates main restricted universe multiverse" >> /etc/apt/sources.list
RUN echo  "deb-src http://gb.archive.ubuntu.com/ubuntu/ xenial-backports main restricted universe multiverse" >> /etc/apt/sources.list

RUN apt-get update && \
    apt-get -y build-dep python-pygame \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip
RUN pip install pygame

RUN git clone https://github.com/kergoth/tslib.git /tmp/tslib && \
    cd /tmp/tslib && \
    ./autogen.sh && \
    ./configure && \
    make && \
    make install


RUN apt-get update && apt-get install -y \
    python-rsvg \
    python-cairo \
    && rm -rf /var/lib/apt/lists/*


RUN pip install evdev

COPY . src/screen

RUN catkin build
