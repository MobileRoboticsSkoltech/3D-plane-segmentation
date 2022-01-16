FROM tojiboyevf/perfect:1.0

ADD ./cpf_segmentation /app/src

WORKDIR /app/build

RUN cmake ../src && make && make install