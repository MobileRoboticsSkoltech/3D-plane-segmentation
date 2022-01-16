FROM tojiboyevf/perfect:1.0

ADD ./peac /app/src

WORKDIR /app/build

RUN cmake ../src/cpp && make && make install