FROM ubuntu:20.04

ARG DEBIAN_FRONTEND=noninteractive

ENV QT_DEBUG_PLUGINS=0
ENV QT_GRAPHICSSYSTEM="native"
ENV QT_X11_NO_MITSHM=1

RUN apt-get update
RUN apt-get install -y ca-certificates apt-utils build-essential make cmake \
	vim git python3 python3-dev python3-pip ffmpeg libsm6 libxext6
RUN apt-get install -y libeigen3-dev
RUN apt-get install -y libgl1-mesa-dev libqt5x11extras5 libxkbcommon-x11-0 \
	libxcb-randr0-dev libxcb-xtest0-dev libxcb-xinerama0-dev libxcb-shape0-dev \
	libxcb-xkb-dev zlib1g-dev libncurses5-dev libgdbm-dev libnss3-dev libssl-dev \
	libreadline-dev libffi-dev libxml2-dev libxslt1-dev libx11-dev x11proto-gl-dev

RUN pip3 install numpy \
	&& pip3 install vtk \
	&& pip3 install mayavi \
	&& pip3 install PyQt5

WORKDIR /workspace
ADD data /workspace/data 
ADD include /workspace/include
ADD scripts /workspace/scripts
ADD tests /workspace/tests
COPY CMakeLists.txt /workspace

RUN mkdir build \
	&& cd build \
	&& cmake .. \
	&& make

CMD ["bash"]