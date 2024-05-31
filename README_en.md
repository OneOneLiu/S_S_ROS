# S_S_ROS

[![English](https://img.shields.io/badge/README-English-blue)](README_en.md)
[![中文](https://img.shields.io/badge/README-中文-orange)](README.md)

I would like to use this repository to organize the notes I previously took, and also as a reference for future students.

## 1. Download
```bash{.line-numbers}
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# If you don't clone to the folder above, you need to modify the path of the volume mount in the Docker startup file.
git clone https://github.com/OneOneLiu/s_s_ros.git
```

## 2. Introduction

- This repository is a **practical** guide to using `ROS` for **beginners**. It focuses on tasks such as robot grasping and manipulation, involving hardware such as robotic arms, cameras, and grippers, but not mobile robots or drones.
- Readers should have a basic understanding of `ROS`, including common concepts such as `ROS` software packages ([`package`](https://wiki.ros.org/Packages)), nodes ([`node`](https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)), and common commands such as `catkin_make`.
- This repository is based on `ROS1 noetic` version.

### 2.1. Good ROS Programming Practices
To complete a robotic manipulation task, we need to use `ROS` to control multiple hardware components, including robotic arms, grippers, cameras, force/torque sensors, etc. Therefore, we need to create software `packages` in the `/catkin_ws/src` directory to implement the functions for using these hardware.

A good practice is to [**modularize different functions**](https://answers.ros.org/question/364674/how-to-properly-structure-a-ros-package/). For example, it is recommended to create a separate package for camera functions and another package for robot functions. Then, we can create a main program package that integrates all the functions.

This modular idea helps improve code readability, maintainability, and promotes code and function reuse.

Also, it is important to organize and partition the folders properly within each package. For example, put all launch files in the `launch` directory, source files in the `scripts` directory, and provide a `README.md` file for each folder or function.

### 2.2. Docker
It is recommended to use `Docker` for  developing and testing all `ROS` projects. This greatly reduces issues caused by environment configuration and improves work efficiency. Borrowing a phrase used for `python`:
> `Life is short, you need Docker`!

This repository provides a simple Docker startup command, which is located in the [`./docker`](docker) directory. Read the [`README.md`](docker/README.md) file inside for instructions.

> - Here is a great video tutorial for `Docker` and `ROS`: https://www.youtube.com/watch?v=qWuudNxFGOQ

### 2.3. Folder Descriptions

Following the programming habits mentioned in the previous section, this repository contains multiple `ROS` packages rather than using the entire repository as a single `ROS` package (as I used to do before; you shouldn't do this as it's a bad habit).

I have created separate git repository for each individual functionality package.

During actual testing with `Docker`, I clone these packages' repositories into the `src/` directory, so they exist in the `/catkin_ws/src/` directory when the container environment is started.

Thus, the folder structure of this repository would roughly be as follows:

```bash{.line-numbers}
s_s_ros/
├── Docker
│   ├── build.bash
│   ├── Dockerfile
│   ├── noetic.bash
│   └── README.md
├── docs
│   └── Hardware
│       ├── 1.Realsense_camera.md
│       ├── 2.UR_robot.md
│       └── 3.Robotiq_gripper.md
├── README_en.md
├── README.md
└── src
    ├── pkg_camera
    ├── pkg_grasp_main
    ├── pkg_gripper
    └── pkg_robot
```

- The `docker` directory contains files for building and starting `Docker` environments as well as related explanations.
- The `docs` directory contains various documentation.
- The `src` directory is empty or contains only a few `pkg_` prefixed packages that are still under development; I will create separate repositories for them and remove them after completion.

After the `Docker` container is started, the directory seen in the virtual environment will roughly be as follows:

```bash{.line-numbers}
catkin_ws/
└── src
  ├── Universal_Robots_ROS_Driver
  ├── pkg_grasp_main
  ├── pkg_urdf
  ├── pkg_virtual_camera
  ├── realsense2_description
  ├── realsense_camera
  ├── realsense_gazebo_plugin
  ├── roboticsgroup_gazebo_plugins
  ├── robotiq
  ├── robotiq_description
  ├── robotiq_gripper
  ├── universal_robot
  ├── ur5_gripper_moveit
  ├── ur5_robot
  └── ur5_robot_gripper
```
More repositories will show up than what you saw outside the container, as they contain repositories we clone during the building of the docker image.

> :memo:
> This includes not only my custom repositories but also some official libraries like those for robots, grippers, and cameras. You can find which are my custom repositories in the comments of the dockerfile as follows:
```bash{.line-numbers}
# Clone my customized repositories
RUN cd /catkin_ws/src && \
    git clone https://github.com/OneOneLiu/realsense_camera.git && \
    git clone https://github.com/OneOneLiu/robotiq_gripper.git && \
    git clone https://github.com/OneOneLiu/ur5_robot.git && \
    git clone https://github.com/OneOneLiu/ur5_gripper_moveit.git && \
    git clone https://github.com/OneOneLiu/ur5_robot_gripper.git
```
> These custom repositories are automatically synchronized when starting the container using the [`entrypoint.sh`](docker/entrypoint.sh) file.

### 2.4. Usage Instruction
This repository is a modular tutorial for beginners, each custom repository clone in the `dockerfile` is an individual functional package, such as using cameras, using grippers, etc. Each package contains its own source code and test files, which can be used separately for reference.

## 3. Common Hardware Usage

- For using the `Intel Realsense` camera, please refer to [`docs/Hardware/1.Realsense_camera.md`](docs/Hardware/1.Realsense_camera.md).
- For using the `UR robot`, please refer to [`docs/Hardware/2.UR_robot.md`](docs/Hardware/2.UR_robot.md).
- For using the `Robotiq` gripper, please refer to [`docs/Hardware/3.Robotiq_gripper.md`](docs/Hardware/3.Robotiq_gripper.md).

The above are all very commonly used hardware, and their official sites also provide corresponding ROS packages, such as:

- [ROS package for Intel RealSense cameras](https://github.com/IntelRealSense/realsense-ros)
- [ROS driver package for UR robots](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

Generally, there is no need to create packages for these hardware by ourselves. You can download and use the official packages directly. Even when customization is needed, it is usually done within their packages or added to the main package of your own project, rather than creating a new package for each hardware. 

I still create separate packages for the camera, robot, and gripper here only for demonstration purposes.

> :memo: Note
> 
>There are several ways to download and use official or third-party ROS packages, which you can find out more about [in this guide](docs/ROS_basics/install_ros_packages.md).