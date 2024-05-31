xhost local:root
XAUTH=/tmp/.docker.xauth

## For fix issues in ssh, run the following manually in the host before starting the container
# sudo rm /tmp/.docker.xauth
# touch /tmp/.docker.xauth
# sudo xauth -f /tmp/.docker.xauth add $(xauth list $DISPLAY)

docker run --rm -it \
    --name=ros_noetic_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/pkg_grasp_main:/catkin_ws/src/pkg_grasp_main" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/pkg_urdf:/catkin_ws/src/pkg_urdf" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/pkg_virtual_camera:/catkin_ws/src/pkg_virtual_camera" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --volume="/tmp/.docker.xauth:/tmp/.docker.xauth:rw" \
    --volume="/home/$USER/.ssh_docker:/root/.ssh" \
    --net=host \
    --privileged \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --runtime=nvidia \
    --gpus all \
    ros_noetic_image \
    bash

echo "Done."