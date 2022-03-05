FROM pavelmokeev/pcllib:latest

RUN pip3 install open3d

ADD . /app/build
WORKDIR /app/build

RUN mkdir build && cd build && cmake .. && cmake --build . && cd ..

ENTRYPOINT ["python3", "scripts/run.py"]