# Docker图形界面的加速

直接使用docker的图形界面, 可能会卡到只有几帧. 比如Rviz或者gazebo, 这个时候我们可以在启动docker时把显卡也加进来.

只需要两步. 

第一根据[这个页面](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html#installing-with-yum-or-dnf)的教程安装 nvidia-docker-toolkit:

```bash{.line-numbers}
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update

sudo apt-get install -y nvidia-container-toolkit
```

第二在[启动docker的命令中](../../Docker/noetic.bash)添加相关配置

```bash{.line-numbers}
...
--runtime=nvidia \
--gpus all
...
```

**References:**
> http://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration