# docker build --no-cache -f Dockerfile . -t sky360/cyclop:1.0.0

FROM ubuntu:22.04

# install basic requirements 
RUN apt-get update && apt-get install -y \
	software-properties-common \
	udev \
	mdadm \
	wget \
	usbutils

# download QHY SDK
WORKDIR /home
# arm64
RUN wget https://www.qhyccd.com/file/repository/publish/SDK/221014/sdk_Arm64_22.10.14.tgz
RUN tar zxfv sdk_Arm64_22.10.14.tgz
RUN rm sdk_Arm64_22.10.14.tgz
# x64
RUN wget https://www.qhyccd.com/file/repository/publish/SDK/221014/sdk_linux64_22.10.14.tgz
RUN tar zxfv sdk_linux64_22.10.14.tgz
RUN rm sdk_linux64_22.10.14.tgz

RUN chmod +x sdk_Arm64_22.10.14/install.sh
RUN chmod +x sdk_linux64_22.10.14/install.sh

RUN apt-add-repository ppa:mutlaqja/ppa && apt-get update
RUN apt-get install -y indi-full gsc

WORKDIR /home
ADD ./install_qhy_sdk_and_udev_rules_to_host.sh /home 
RUN chmod +x ./install_qhy_sdk_and_udev_rules_to_host.sh

EXPOSE 7624

RUN mkfifo /tmp/indiserver_FIFO 

CMD indiserver -v -f /tmp/indiserver_FIFO
