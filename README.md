# S_S_ROS

[![English](https://img.shields.io/badge/README-English-blue)](README_en.md)
[![中文](https://img.shields.io/badge/README-中文-orange)](README.md)

使用这个仓库整理一下之前记的笔记, 也可以用来给之后的同学做参考

## 0. 预备工作
> 建议使用`Docker`进行所有`ROS`项目的测试和开发, 这会极大地减少环境配置所带来的问题, 提高工作效率. 借用一下形容 `python` 的话:
> `Life is short, you need Docker`!

这个仓库提供了傻瓜式的docker启动命令, 就放在[`./Docker`](Docker)目录下, 去读一下里面的[`README.md`](Docker/README.md)就知道怎么用了.

## 1. 常用硬件的连接和使用

- `Intel Realsense` 相机的使用, 请参照 [docs/Hardware/1.Realsense_camera.md](docs/Hardware/1.Realsense_camera.md).

- `UR` 机器人的使用, 请参照 [docs/Hardware/2.UR_robot.md](docs/Hardware/2.UR_robot.md).

- `Robotiq` 夹爪的使用, 请参照 [docs/Hardware/3.Robotiq_gripper.md](docs/Hardware/3.Robotiq_gripper.md).