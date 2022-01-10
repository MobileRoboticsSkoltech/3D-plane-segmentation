# DDPFFAdoptation
Attempt to adopt original DDPFF algo - https://github.com/arindamrc/DDPFF
To build docker use `sudo docker build -t ddpff .`
To run docker use `sudo docker run --mount src=/home/adminlinux/DDPFFAdoptation/input,target=/app/build/input,type=bind --mount src=/home/adminlinux/DDPFFAdoptation/output,target=/app/build/output,type=bind ddpff`