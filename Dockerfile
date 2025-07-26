FROM ros:humble

RUN apt-get update && apt-get install -y ros-humble-rqt-image-view libqt5gui5 libxcb-util1 qtbase5-dev qt5-qmake libxkbcommon-x11-0

WORKDIR /ws
COPY . .

RUN colcon build

ENTRYPOINT ["bash", "entrypoint.sh"]

