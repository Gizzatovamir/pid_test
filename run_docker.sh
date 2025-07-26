xhost +local:root
docker run -it \
    --privileged \
    --network="host" \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="ROS_IP=$ROS_IP" \
    --env="ROS_MASTER_URI=$ROS_MASTER_URI" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    pid