FROM gcc:latest as build

RUN apt-get update && \
    apt-get install -y \
      libeigen3-dev \
      cmake


# Install Open3D system dependencies and pip
RUN apt-get update && apt-get install --no-install-recommends -y \
    libgl1 \
    libgomp1 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Open3D from the pypi repositories
RUN python3 -m pip install --no-cache-dir --upgrade open3d

ADD . /app/src

WORKDIR /app/build

RUN cmake ../src && make

ADD scripts/ .

ENTRYPOINT ["bash", "run.sh"]
