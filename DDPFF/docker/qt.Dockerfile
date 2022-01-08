FROM ubuntu:focal

MAINTAINER Arindam Roychoudhury <roychoud@cs.uni-bonn.de>

ENV DEBIAN_FRONTEND="noninteractive" 
ENV TZ="Europe/Berlin"

# Build failing without the following:
RUN apt-get update && apt-get full-upgrade -y && apt-get install -y --no-install-recommends tzdata

# Install lots of packages
RUN apt-get update && apt-get install -y libxcb-keysyms1-dev libxcb-image0-dev \
    libxcb-shm0-dev libxcb-icccm4-dev libxcb-sync0-dev libxcb-xfixes0-dev \
    libxcb-shape0-dev libxcb-randr0-dev libxcb-render-util0-dev \
    libfontconfig1-dev libfreetype6-dev libx11-dev libxext-dev libxfixes-dev \
    libxi-dev libxrender-dev libxcb1-dev libx11-xcb-dev libxcb-glx0-dev x11vnc \
    xauth build-essential mesa-common-dev libglu1-mesa-dev libxkbcommon-dev \
    libxcb-xkb-dev libxslt1-dev libgstreamer-plugins-base1.0-dev wget \
    libxkbcommon-x11-0 libxkbcommon-x11-dev libwayland-cursor0 libgssapi-krb5-2 libxcb-xinerama0

# Download script
RUN wget http://download.qt.io/official_releases/online_installers/qt-unified-linux-x64-online.run
RUN chmod +x ./qt-unified-linux-x64-online.run