# 安装使用现有的 ROS Packages

很多情况下, 我们使用的硬件都是很常用的, 存在官方或者第三方提供的`ROS`软件包可以直接使用. 使用这些包可以避免重复造轮子, 加速我们的开发过程.

要使用这些包一般有以下几种方式

## 1. 通过ROS软件仓库安装

最简单且方便的方法是通过`ROS`的软件仓库安装包。这些包已经被预编译，可以通过apt-get直接安装。这需要你的系统已经安装了`ROS`，并配置了相应的软件源。安装命令模板如下：

```bash{.line-numbers}
sudo apt-get install ros-<ros_version>-<package_name>
```

比如我们可以通过下面的命令来安装 `RealSense` 相机的软件包:
```bash{.line-numbers}
apt-get install -y ros-$ROS_DISTRO-realsense2-camera
```
这一行命令已经被添加到了我们的[`Dockerfile`](../../Docker/Dockerfile)里, 会被自动安装. 

通过这种方式安装的软件包会被安装到系统的ROS的库目录里, 比如`/opt/ros/<ros_version>/lib/<package_name>`, 而不会出现在我们的工作空间源码目录 `~/catkin_ws/src`

## 2. 从源代码编译

有很多软件是没有经过预编译的, 无法通过上面的方式快捷安装. 这时候可以下载对应包的源代码到我们的工作空间的 src 目录. 或者我们想要修改某个包的源代码, 也可以通过这种方式. 具体步骤如下:

1. 下载软件包源码
```bash{.line-numbers}
cd ~/catkin_ws/src
git clone <repository_url>
```

2. 编译软件包并更新ROS编译好的环境
```bash{.line-numbers}
catkin_make
source devel/setup.bash
```

3. 有时编译会出错, 提示缺少依赖, 安装对应软件包的依赖即可.

> 使用 `rosdep install --from-paths src --ignore-src -r -y` 命令可以根据当前选定的源文件目录自动安装所需要的 ROS 依赖, 本仓库的 [`dockerfile`](../../docker/Dockerfile) 中也使用了.

## 3. 使用
通过上面的方式安装好包, 并编译更新环境之后, 就可以通过对应提供的`launch file`, 或者`rosrun`命令启动需要的节点, 使用功能.