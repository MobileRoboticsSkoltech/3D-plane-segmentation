FROM arindamrc/qt515:latest

RUN echo "INTEGRATED"

MAINTAINER Arindam Roychoudhury <roychoud@cs.uni-bonn.de>

ENV DEBIAN_FRONTEND="noninteractive" 
ENV TZ="Europe/Berlin"

# Build failing without the following:
RUN apt-get update && apt-get full-upgrade -y && apt-get install -y --no-install-recommends tzdata

# Dependencies for glvnd and X11.
RUN apt-get update \
  && apt-get install -y -qq --no-install-recommends \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
    libclang1 \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    mesa-vulkan-drivers \
    mesa-vdpau-drivers \
    mesa-va-drivers \
    mir-platform-graphics-mesa-x16 \
    mir-client-platform-mesa-dev \
    libglx-mesa0 \
    libegl-mesa0 \
    libglapi-mesa \
    freeglut3-dev \
    mesa-common-dev \
    xdg-utils

# Install basic necessities, OpenGL, OpenMP, OpenNI2, Eigen, GSL, BLAS, LAPACK, Armadillo, PCL.
RUN apt-get update && apt-get full-upgrade -y && apt-get install -y --no-install-recommends \
    git \
    ca-certificates \
    locales \
    sudo \
    cmake \
    build-essential \
    gdb \
    pkg-config \
    libxrender1 \
    libfontconfig1 \
    fonts-ubuntu \
    libxcb-xinerama0-dev \
    libomp-dev \
    libopenni2-dev \
    libeigen3-dev \
    libgsl-dev \
    libopenblas-dev \
    liblapack-dev \
    libarpack2-dev \
    libpcl-dev \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    unzip \
    && apt-get -qq clean

# Install QGLViewer
RUN apt-get -y update && DEBIAN_FRONTEND=noninteractive apt-get -y install \
    libqglviewer-headers \
    libqglviewer-dev-qt5 \
    libqglviewer2-qt5

# Install OpenCV
RUN apt-get update && apt-get full-upgrade -y && apt-get install -y --no-install-recommends libopencv-dev \
    python3-dev \
    python3-numpy \
    python3-opencv \
    libtbb-dev \
    pkg-config \
    && apt-get -qq clean

RUN ln -s /usr/include/opencv4/opencv2 /usr/include/opencv2

# RUN rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/* # Keep apt active for the development environment

# Reconfigure locale
RUN locale-gen en_US.UTF-8 && dpkg-reconfigure locales

# Add group & user + sudo
ENV USERNAME arc
RUN groupadd -r $USERNAME && useradd --create-home --gid $USERNAME $USERNAME 
RUN touch /etc/sudoers.d/$USERNAME && echo $USERNAME' ALL=NOPASSWD: ALL' > /etc/sudoers.d/$USERNAME

USER $USERNAME
ENV HOME /home/$USERNAME

CMD ["bash"]
