# ROS中的服务和参数

所有的相关资料均可在下面链接的 ROS wiki上面查到：
- https://wiki.ros.org/ROS/Tutorials/UnderstandingServicesParams

## 一个问题：

1. 为什么要用`ros params server`， 直接`launch`文件启动节点不也是可以设置节点参数吗？

   - 在启动某个节点时在launch文件中设置的参数是固定的，在启动之后无法再改变，而ROS1的参数服务器允许不同的节点把**节点参数，节点配置信息**存储到一个公共的中央服务器上，允许动态的修改。
   - 参数服务器帮助实现更加灵活的行为，我们不需要在编译节点代码的时候考虑参数的配置问题，可以在使用时通过参数服务器给节点设置不同参数来实现不同的功能

## 一个例子
参数服务器可以存放各种数据类型的数据，包括整型，浮点型，布尔型，字典，列表等等，乃至整个`xml`文件作为参数服务器的参数都可以。在每次启动ROS核心时终端都会输出现在的参数。我们下面放一个`UR`机器人描述文件的参数来看一下。

进入我们的docker container，执行
```bash{.line-numbers}
roslaunch ur_description load_ur5.launch
```

可以看到下面的输出：
```bash{.line-numbers}
started roslaunch server http://localhost:40855/

SUMMARY
========

PARAMETERS
 * /robot_description: <?xml version="1....
 * /rosdistro: noetic
 * /rosversion: 1.16.0

NODES
```

可以从`PARAMETERS`栏看到当前参数服务器中一共有三个参数，其中后两个是ROS的版本信息，只要启动`roscore`就会自动添加的。

第一个`/robot_description`参数是我们启动的launch文件添加的参数，如果你打开它所使用的[load_ur.launch](https://github.com/ros-industrial/universal_robot/blob/noetic-devel/ur_description/launch/load_ur.launch)文件，就会发现这个launch文件里面用了一行命令做了两件事：
- 通过一个[`xacro`](urdf.md)模板文件生成机器人的[`urdf`](urdf.md)描述文件
- 向参数服务器添加这个描述文件

它没有添加任何节点，所以我们看到节点（`NODES`）的输出为空

你可以使用`rosparam get /robot_description`命令查看这个参数的内容，会发现它就是一个URDF的完整内容。

通过这个`launch`文件，我们就将`UR5`机器人完整描述信息上传到了`ROS`的参数服务器，后面如果有要使用相关信息的节点，比如`moveit`节点，就可以来这里读取。

所以，如果想要启动`moveit`节点对机器人进行运动规划，应当同时或者先启动添加描述文件的这个launch file.