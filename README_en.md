# S_S_ROS

[![English](https://img.shields.io/badge/README-English-blue)](README_en.md)
[![中文](https://img.shields.io/badge/README-中文-orange)](README.md)

I would like to use this repository to organize the notes I previously took, and also as a reference for future students.

## 1. Introduction

- This repository is a **practical** guide to using `ROS` for **beginners**. It focuses on tasks such as robot grasping and manipulation, involving hardware such as robotic arms, cameras, and grippers, but not mobile robots or drones.
- Readers should have a basic understanding of `ROS`, including common concepts such as `ROS` software packages ([`package`](https://wiki.ros.org/Packages)), nodes ([`node`](https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)), and common commands such as `catkin_make`.
- This repository is based on `ROS1 noetic` version.

### 1.1. Good ROS Programming Practices
To complete a robotic manipulation task, we need to use `ROS` to control multiple hardware components, including robotic arms, grippers, cameras, force/torque sensors, etc. Therefore, we need to create software `packages` in the `/catkin_ws/src` directory to implement the functions for using these hardware.

A good practice is to modularize different functions. For example, it is recommended to create a separate package for camera functions and another package for robot functions. Then, we can create a main program package that integrates all the functions.

This modular idea helps improve code readability, maintainability, and promotes code and function reuse.

Also, it is important to organize and partition the folders properly within each package. For example, put all launch files in the `launch` directory, source files in the `scripts` directory, and provide a `README.md` file for each folder or function.

### 1.2. Docker
It is recommended to use `Docker` for  developing and testing all `ROS` projects. This greatly reduces issues caused by environment configuration and improves work efficiency. Borrowing a phrase used for `python`:
> `Life is short, you need Docker`!

This repository provides a simple Docker startup command, which is located in the [`./Docker`](Docker) directory. Read the [`README.md`](Docker/README.md) file inside for instructions.

> - Here is a great video tutorial for `Docker` and `ROS`: https://www.youtube.com/watch?v=qWuudNxFGOQ

### 1.3. Folder Descriptions

Following the programming practices mentioned earlier, this repository contains multiple `ROS packages`, rather than treating the entire repository as a single `ROS` package (which was a bad practice that I used to follow, but you should avoid). I will use the `Docker` [`volume`](https://docs.docker.com/storage/volumes/) feature to map each subfolder to the `/catkin_ws/src/` directory in the container environment. You can check the command in this [Docker startup file](Docker/noetic.bash).

Therefore, the folder structure of this repository will look like this:
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
├── pkg_camera
├── pkg_grasp
├── pkg_gripper
├── pkg_robot
├── README_en.md
├── README.md
```

- The `Docker` directory contains files for building and starting the Docker environment, along with related instructions.
- The `docs` directory contains various documentation.
- Those starting with `pkg_` are packages added for various features; using pkg_ as a prefix might look strange, but it groups them together, so let's stick with that.

After the Docker container starts, before the first catkin_make, the directory structure seen in the virtual environment will roughly be as follows:
```bash{.line-numbers}
catkin_ws/
└── src
    ├── pkg_camera
    ├── pkg_grasp_main
    ├── pkg_gripper
    └── pkg_robot
```

## 2. Common Hardware Usage

- For using the `Intel Realsense` camera, please refer to `docs/Hardware/1.Realsense_camera.md`.
- For using the `UR robot`, please refer to `docs/Hardware/2.UR_robot.md`.
- For using the `Robotiq` gripper, please refer to `docs/Hardware/3.Robotiq_gripper.md`.

The above are all very commonly used hardware, and their official sites also provide corresponding ROS packages, such as:

- [ROS package for Intel RealSense cameras](https://github.com/IntelRealSense/realsense-ros)
- [ROS driver package for UR robots](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

Generally, there is no need to create packages for these hardware by ourselves. You can download and use the official packages directly. Even when customization is needed, it is usually done within their packages or added to the main package of your own project, rather than creating a new package for each hardware. 

I still create separate packages for the camera, robot, and gripper here only for demonstration purposes.

> :memo: Note
> 
>There are several ways to download and use official or third-party ROS packages, which you can find out more about [in this guide](docs/ROS_basics/install_ros_packages.md).