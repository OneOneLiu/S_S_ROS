# Robot state publisher

所有的相关资料均可在下面链接的 ROS wiki 上面查到：
> https://wiki.ros.org/robot_state_publisher

## 一个问题
**这个节点是是干什么的？**
- `robot_state_publisher` uses the `URDF` specified by the parameter `robot_description` and the joint positions from the topic `joint_states` to calculate the forward kinematics of the robot and publish the results via `tf`. 

即，这个节点主要做三件事：
- 从参数服务器中的 `robot_description` 参数中读取当前机器人的 URDF 信息 （`机器人硬件结构长什么样`），同时从 `joint_states` 这个话题订阅机器人的关节角度信息（`机器人每个关节处在什么角度`）。
- 使用这两种信息进行前向运动学计算，计算出机器人上所有的坐标系（`比如UR机器人基坐标有个坐标系，末端也有坐标系，中间每个关节都有自己的坐标系`）位置。
- 将这些关节信息发布到ROS的坐标系管理系统 [`tf`](../concepts/tf.md) 中去。这样所有的ROS组成部分都可以去 [`tf`](../concepts/tf.md)  中了解当前机器人各个关节的信息。

总的来说，他就是把机器人的关节角度角度，速度等间接信息，计算得到机器人上坐标系的信息，所以这里的state我觉得应该翻译成机器人坐标系状态。