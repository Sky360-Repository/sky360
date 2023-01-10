FROM ubuntu:22.04

RUN apt-get update && \
    apt-get install -y software-properties-common wget usbutils
        
RUN apt-add-repository ppa:mutlaqja/ppa && \
    apt-get update
RUN apt-get install -y indi-full gsc

WORKDIR /tmp
RUN wget https://www.qhyccd.com/file/repository/publish/SDK/221014/sdk_Arm64_22.10.14.tgz
#RUN wget https://www.qhyccd.com/file/repository/publish/SDK/221014/sdk_linux64_22.10.14.tgz
RUN tar zxfv sdk_Arm64_22.10.14.tgz
#RUN tar zxfc sdk_linux64_22.10.14.tgz
WORKDIR /tmp/sdk_Arm64_22.10.14/
#WORKDIR /tmp/sdk_linux64_22.10.14.tgz

RUN chmod +x ./install.sh
RUN ./install.sh

# need to find the right device via `lsusb` and runt he `fxload` command below
# should automatically work after reboot - need to verify
WORKDIR /tmp
ADD ./fxload_qhy183.sh /tmp
RUN chmod +x ./fxload_qhy183.sh
RUN apt-get install -y vim

CMD /tmp/fxload_qhy183.sh && indiserver -v indi_qhy_ccd
