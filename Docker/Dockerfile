FROM osrf/ros:noetic-desktop-full
# Set the environment variable to non-interactive, this is to avoid the tzdata configuration during the installation of the packages
ENV DEBIAN_FRONTEND=noninteractive

# System layer installation
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y wget && \
    apt-get install -y iputils-ping && \
    apt-get -y install build-essential && \
    apt install -y cmake && \
    apt install -y tree && \
    apt install -y gdb && \
    apt-get install -y git && \
    apt-get install -y python3-pip && \
    apt-get install -y xauth x11-apps # This is for GUI testing

# Intel realsense camera
RUN apt-get install -y ros-$ROS_DISTRO-realsense2-camera && \
    apt-get install -y libcanberra-gtk-module libcanberra-gtk3-module && \
    pip3 install pyrealsense2

# Dependencies layer # Ur_driver
RUN mkdir -p /catkin_ws/src && \
    cd /catkin_ws/src && \
    git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git

# Use rosdep to install all dependencies according to the package.xml files in all src folders
RUN cd /catkin_ws && \
    rosdep install --from-paths src --ignore-src -r -y

RUN echo "ALL Done"