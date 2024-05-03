# 向Package添加可执行文件作为ROS节点

当我们在ROS包中添加新的可执行程序（无论是Python还是C++）时，需要按照一定的步骤来确保这些程序可以正确编译和执行。本文档介绍如何处理新添加的Python和C++源程序, 使其能够作为节点被启动。

- 无论添加Python源文件还是C++源文件，关键步骤都是将文件放置在适当的目录、更新 `CMakeLists.txt` 文件以及确保程序可执行。

- 下列命令均在某个软件包的根目录下执行: `~/catkin_ws/src/<package_name>/`

## 1. Python源程序
### 1.1. 将程序放置在正确的位置
通常，`Python`脚本放在包的 `src` 文件夹中。如果没有这个文件夹，先创建它：
```bash{.line-numbers}
mkdir src
```
然后，将写好的Python脚本放入此文件夹。

### 1.2. 更新 CMakeLists.txt 文件
添加源程序到`src`目录后, 我们必须告诉ROS我们会使用这个文件作为ROS的节点. 这需要修改此软件包的`CMakeLists.txt` 文件，在该文件中取消注释 `catkin_install_python()` 函数，并添加我们需要使用的Python源文件的路径：
```cmake{.line-numbers}
catkin_install_python(PROGRAMS src/your_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```

### 1.3. 确保这个文件有可执行权限
有时候可能无法执行，提示该文件没有可执行权限， 可以执行下面的命令来赋予该文件可执行权限。
```bash{.line-numbers}
chmod +x src/your_script.py
```

### 1.4. 编译源代码
```bash{.line-numbers}
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 1.5. 运行测试
首先启动ROS core，然后执行
```bash{.line-numbers}
rosrun <package_name> your_script.py
```

## 2. 添加C++程序
### 2.1. 添加源文件和头文件
将你的 `.cpp` 源文件 和 `.h` 头文件分别放在包的 `src` 和 `include` 文件夹中。

### 2.2. 更新 CMakeLists.txt 文件
为了编译`C++`代码，也需要修改 `CMakeLists.txt` 文件来添加相关的编译指令：

- **添加可执行文件：**

```cmake{.line-numbers}
add_executable(executable_name src/your_source_file.cpp)
```

- **添加依赖库：**
```cmake{.line-numbers}
target_link_libraries(executable_name ${catkin_LIBRARIES})
```
- **安装可执行文件：**
```cmake{.line-numbers}
install(TARGETS executable_name
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
### 2.3. 编译项目
```bash{.line-numbers}
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2.4. 运行测试
```bash{.line-numbers}
rosrun <package_name> executable_name
```