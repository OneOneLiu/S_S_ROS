# S_S_ROS

[![English](https://img.shields.io/badge/README-English-blue)](README_en.md)
[![中文](https://img.shields.io/badge/README-中文-orange)](README.md)

使用这个仓库整理一下之前记的笔记, 也可以用来给之后的同学做参考

## 1. 一些说明

- 本仓库面向是一个面向**初学者**的**实操导向**`ROS`使用笔记. 针对的任务机器人抓取操纵等任务, 涉及到的硬件主要是机械臂, 相机, 以及电动夹爪, 不涉及移动机器人, 无人机等技术.
- 读的同学需要对`ROS`有一些基本的了解, 知道常用的概念, 如`ROS`的软件包([`package`](https://wiki.ros.org/Packages)), 节点([`node`](https://wiki.ros.org/ROS/Tutorials/UnderstandingNodes)), 和常用命令, 如`catkin_make`.
- 本仓库基于`ROS1 noetic`版本.

### 1.1. 好ROS 编程习惯
要完成一个机器人操作任务, 我们需要使用`ROS`来控制多个硬件, 包括机械臂, 夹爪, 相机, 力/力矩传感器等. 那么在开发的时候就需要在 `/catkin_ws/src` 目录下创建软件包来实现这些功能, 也就是`package`. 

一个比较好的习惯是尽量把不同的功能做好隔离, 也就是**模块化**. 比如, 相机的功能就创建一个相机的`package`, 机器人就创建机器人的`package`. 最后整体控制所有的主程序, 再创建一个整体功能的`package`.

这种模块化的思想可以帮助我们提高代码的可读性和可维护性，促进代码和功能的重复使用.

与此同时, 在每个`package`下面, 我们也应当做好文件夹的整理和分区, 比如把所有的`launch` file都放到`launch`目录下, 源文件都放到 `scripts` 目录下, 同时给每个文件夹或者功能都添加说明文档 `README.md`. 

### 1.2. 使用Docker
建议使用`Docker`进行所有`ROS`项目的测试和开发, 这会极大地减少环境配置所带来的问题, 提高工作效率. 借用一下形容 `python` 的话:
> `Life is short, you need Docker`!

本仓库提供了傻瓜式的docker启动命令, 就放在[`./Docker`](Docker)目录下, 去读一下里面的[`README.md`](Docker/README.md)就知道怎么用了.

> - 这是一个很不错的视频教程: https://www.youtube.com/watch?v=qWuudNxFGOQ

### 1.3. 文件夹说明

依据前一节说的编程习惯, 本仓库是包含多个`ROS packages`的, 而不是把这整个仓库只作为一个`ROS`的`package`, (我以前是这么搞的, 你们不要这么做, 这是个很坏的习惯). 在实际测试的时候, 我们使用`Docker`, 我会通过`Docker` 的[`volume`](https://docs.docker.com/storage/volumes/)功能把每个子文件夹都映射到`container`容器环境中的`/catkin_ws/src/`目录下. 你可以在这个[`Docker`启动文件](Docker/noetic.bash)里面找到相关命令.

所以这个仓库的文件夹结构大致会是下面的样子:
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
其中:
- `Docker` 目录下放的是`Docker`环境编译,启动的文件以及相关说明
- `docs`目录下放的就是各种说明文档
- 以`pkg_`开头的就是给各个功能添加的`package`, 把`pkg_`作为前缀看着很奇怪, 但是可以让他们看起来在一起, 就这样吧.

当`Docker container`启动之后, 首次`catkin_make`之前, 在虚拟环境中看到的目录大致会是这样:
```bash{.line-numbers}
catkin_ws/
└── src
    ├── pkg_camera
    ├── pkg_grasp_main
    ├── pkg_gripper
    └── pkg_robot
```

## 2. 常用硬件的使用

- `Intel Realsense` 相机的使用, 请参照 [docs/Hardware/1.Realsense_camera.md](docs/Hardware/1.Realsense_camera.md).

- `UR` 机器人的使用, 请参照 [docs/Hardware/2.UR_robot.md](docs/Hardware/2.UR_robot.md).

- `Robotiq` 夹爪的使用, 请参照 [docs/Hardware/3.Robotiq_gripper.md](docs/Hardware/3.Robotiq_gripper.md).

> 以上几个都是非常常用的硬件, 他们的官方也都提供对应的ROS功能包, 比如:
> - [Intel RealSense相机的ROS软件包](https://github.com/IntelRealSense/realsense-ros)
> - [UR 机器人的ROS驱动软件包](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver)

一般要使用这些硬件, 直接下载使用他们官方的软件包就可以了, 无需自己再创建一个package. 如果需要自定义功能, 一般也是在他们的packages里面添加, 或者在自己项目的main package里添加, 无需再创建一个新的package. 我这里仍旧创建相机, 机器人以及夹爪的三个packages是为了展示这一过程. 

> :memo: **Note**
>
> 下载使用官方或者第三方提供的ROS package有多种方式, 具体可以参考[这个说明](docs/ROS_basics/install_ros_packages.md). 