<h1 align="center">Quadruped ROS2 Control</h1>

<p align="center">
  <img src="https://www.unitree.com/images/382c851410194e098c12d816caf8b90d_3840x2146.jpg" alt="Unitree Go2 中文宣传图" width="88%">
</p>

<p align="center">
  <em>面向 Unitree Go2 的仿真验证、控制器问题排查与 ROS 2 集成记录。</em>
</p>

<p align="center">
  <strong>简体中文</strong> | <a href="./readme_EN.md">English</a>
</p>

> 这是一份基于原始 `Quadruped ROS2 Control` 项目整理的中文实践说明。当前阶段的修改主要从 `Unitree_guide` 相关部分展开，并结合 `Unitree Go2` 的仿真调试经验持续整理，具体修改内容可参考[修改记录](./readme_fixed.md)；后续也会逐步扩展到其他控制器的修改与优化。

---

## 分支版本介绍
在继续扩展当前项目之前，这里也想先介绍一下一个很重要的分支版本：[quadruped_ros2_control_unitree_guide `debug` 分支](https://github.com/yanyuze1/quadruped_ros2_control_unitree_guide/tree/debug?tab=readme-ov-file)。这个分支版本对 `unitree_guide` 控制器相关内容做了单独提取，方便更聚焦地查看、调试和分析该部分逻辑；同时也补充了一些可用于 `debug` 的内容，便于在排查问题、观察控制状态和后续优化时更高效地定位关键环节。

## 前言
如果你也在折腾四足机器人控制、仿真和调参，希望这份记录能帮你更快进入状态，也少走一些弯路。感谢项目[Quadruped ROS2 Control](https://github.com/legubiao/quadruped_ros2_control/tree/humble)提供了扎实的基础，本项目正是在它之上继续修改与整理而来，当前使用的是 `Unitree_guide` 分支。后面的内容会围绕真实调试过程中遇到的问题与解决思路展开，希望能让你带着兴趣一路看下去，也能在需要时真正派上用场。
## 项目结构图
![alt text](images/Unitree_guide.png)

## 快速开始
### 软件准备
- 安装unitree_sdk2
- 安装Unitree_mujoco
- 安装Unitree_guide

#### unitree_sdk2
```bash
git clone https://github.com/unitreerobotics/unitree_sdk2.git
cd unitree_sdk2/
mkdir build
cd build
cmake .. 
sudo make install
```
![alt text](images/image2.png)

#### Unitree_mujoco
```bash
sudo apt install libyaml-cpp-dev libspdlog-dev libboost-all-dev libglfw3-dev
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```
![alt text](images/image3.png)

在网页mujoco_releases中寻找到mujoco-3.3.6版本进行下载,下载完成后解压到路径unitree_mujoco/simulate下,注意记得要修改名字为mujoco，官方使用的是软连接方式进行和我们的方式不同。

![alt text](images/image4.png)
![alt text](images/image5.png)

```bash
cd unitree_mujoco/simulate
mkdir build && cd build
cmake ..
make -j4
```
![alt text](images/image6.png)

若上述使用非软链接方式进行编译因找不到mujoco而编译失败，可以尝试使用unitree_mujoco中的方式进行或如下直接安装mujoco到系统中

```bash
git clone https://github.com/google-deepmind/mujoco.git
cd mujoco
git checkout 3.3.6
mkdir build && cd build
cmake ..
make -j
sudo make install
```

使用下述指令启动仿真器进行测试,请注意Cyclonedds在lo网卡下容易导致仿真的启动失败，这是因为Cyclonedds的本地回环网卡不支持组播导致，这里推荐指定使用Fastdds或使用Cyclonedds的其他网卡进行。同时后续的使用中Cyclonedds也会带来不便，在此项目中推荐将其删除。

```bash
./unitree_mujoco -r go2
```
![alt text](images/image7.png)

#### Unitree_guide
```bash
git clone https://github.com/yanyuze1/quadruped_ros2_control_unitree_guide.git
cd quadruped_ros2_control_unitree_guide
rosdep install --from-paths src --ignore-src -r -y
```
![alt text](images/image8.png)

仿真部分使用下方的编译方式即可
```bash
colcon build --packages-up-to unitree_guide_controller go2_description keyboard_input hardware_unitree_mujoco --symlink-install
```
![alt text](images/image9.png)

### 使用
- 仿真使用
- 真实机器人使用
- docker使用
#### 仿真使用
- 启动仿真器Unitree_mujoco
```bash
cd unitree_mujoco/simulate/build/
./unitree_mujoco -r go2
```
![alt text](images/image10.png)

- 启动Unitree_guide控制器

修改路径quadruped_ros2_control_unitree_guide/src/quadruped_ros2_control/descriptions/unitree/go2_description/xacro下的文件ros2_control.xacro，注意domain需要对齐ros2_control.xacro和unitree_mujoco的config.yaml
```bash
<hardware>
    <plugin>hardware_unitree_mujoco/HardwareUnitree</plugin>
    <param name="domain">0</param>
    <param name="network_interface">enp109s0</param>
    <param name="show_foot_force">true</param>
</hardware>
# 修改为
<hardware>
    <plugin>hardware_unitree_mujoco/HardwareUnitree</plugin>
    <param name="domain">1</param>
    <param name="network_interface">lo</param>
    <param name="show_foot_force">true</param>
</hardware>
```
![alt text](images/image11.png)

![alt text](images/image12.png)

修改完成后编译功能包go2_description
```bash
cd quadruped_ros2_control_unitree_guide/
colcon build --packages-up-to go2_description --symlink-install
```
![alt text](images/image13.png)

在编译完成后进行控制器启动,成功启动会出现rviz2界面
```bash
source install/setup.bash
ros2 launch unitree_guide_controller mujoco.launch.py
```
![alt text](images/image14.png)

启动遥控器，仿真支持的控制器有键盘和游戏手柄，当前ros2版本的Unitree_guide不支持cmd_vel模式，在后续会进行添加。

![alt text](images/image15.png)

```bash
cd quadruped_ros2_control_unitree_guide/
source install/setup.bash
ros2 run keyboard_input keyboard_input    # 键盘遥控器
```
![alt text](images/image16.png)

![alt text](<images/2026-04-11 14-50-38.gif>)

- 游戏手柄控制器
```bash
cd quadruped_ros2_control_unitree_guide/
colcon build --packages-up-to joystick_input --symlink-install
source install/setup.bash
ros2 launch joystick_input joystick.launch.py
```
#### 真机使用
在真机部分依照彪哥的真机部署，将接口修改为对应的接口，在启动控制器时记得使用手机将官方控制器关闭，未关闭官方控制器对时两个控制器争抢机器狗控制权对电机关节损伤较大。
![alt text](images/image17.png)

```bash
ros2 launch unitree_guide_controller mujoco.launch.py
```
![alt text](images/bb601e9e3a84df7402d91245d37eb7bd.gif)

#### Docker镜像
当前已经将配置完成的docker镜像上传至docker镜像仓库，在需要进行独立使用时可以直接进行配置使用。注意:这份镜像只简化了环境的配置，但软件包的构建依然由自己安成。
```bash
docker pull huahuajiang/unitree_guide:ros2-humble
```
![alt text](images/image18.png)

在获取到docker镜像后使用下方指令完成环境配置
```bash
docker run -it \
  --env DISPLAY=${DISPLAY} \
  --env NVIDIA_DRIVER_CAPABILITIES=all \
  --env QT_X11_NO_MITSHM=1 \
  --env USER_ID=${USER_ID:-1000} \
  --env GROUP_ID=${GROUP_ID:-1000} \
  --device /dev/snd:/dev/snd \
  --device /dev/dri:/dev/dri \
  --device /dev/input:/dev/input \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --volume /home/$(whoami):/home/ROS2 \
  --gpus all \
  huahuajiang/unitree_guide:ros2-humble
```
在docker容器中完成前面的软件构建后即可直接开启仿真和真机控制了。

## 鸣谢
感谢[Quadruped ROS2 Control](https://github.com/legubiao/quadruped_ros2_control/tree/humble)提供了扎实的工程基础，本项目正是在它之上继续修改与整理而来；同时感谢[Unitree_guide](https://support.unitree.com/home/zh/Algorithm_Practice/about_unitreeguide)提供了完整的理论基础，本项目的理论基础全来源于此。