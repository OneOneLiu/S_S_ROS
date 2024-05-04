xhost local:root
XAUTH=/tmp/.docker.xauth
docker run --rm -it \
    --name=ros_noetic_container \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/pkg_camera:/catkin_ws/src/pkg_camera" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/pkg_robot:/catkin_ws/src/pkg_robot" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/pkg_gripper:/catkin_ws/src/pkg_gripper" \
    --volume="/home/$USER/catkin_ws/src/s_s_ros/src/pkg_grasp_main:/catkin_ws/src/pkg_grasp_main" \
    --volume="/dev/bus/usb:/dev/bus/usb" \
    --volume="/tmp/.docker.xauth:/tmp/.docker.xauth:rw" \
    --net=host \
    --privileged \
    ros_noetic_image \
    bash

echo "Done."