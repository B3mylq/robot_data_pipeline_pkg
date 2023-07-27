# Communication Interface Package for Littledog

## 1、配置和编译
### 1.1 编译工作包
打开终端，创建或进入ROS工作空间，执行以下命令克隆项目并编译
```bash
cd src
git clone https://github.com/B3mylq/robot_data_pipeline_pkg
cd ..
catkin_make
source devel/setup.bash
```

### 1.2 安装支持库(应对1.1报错缺少对应依赖的情况，如果1.1编译成功则不需要进行这一步) 
a. 编译安装libzmq：\
下载源码: 
```bash 
git clone https://gitcode.net/mirrors/zeromq/libzmq.git
```
编译安装: 
```bash
cd libzmq && mkdir build && cd build
cmake ..
sudo make install
```
b. 编译安装cppzmq \
下载源码: 
```bash 
git clone https://gitcode.net/mirrors/zeromq/cppzmq.git
```
编译安装: 
```bash
cd cppzmq && mkdir build && cd build
cmake ..
sudo make install
```
c. 编译安装RapidJSON\
下载源码: 
```bash 
git clone https://github.com/Tencent/rapidjson.git
```
编译安装: 
```bash
cd rapidjson && mkdir build && cd build
cmake ..
sudo make install
```
d. 编译安装spdlog\
下载源码: 
```bash 
https://github.com/gabime/spdlog.git
```
编译安装: 
```bash
cd spdlog && mkdir build && cd build
cmake ..
sudo make install
```

---
## 2、启动下位机
---
## 3、通信结构与数据格式说明
&emsp;&emsp;上下位机间的通信结构如图所示，下位机、上位机中继发布节点(src/robot_state_pub_node.cpp)和上位机中继接收节点(src/robot_state_sub_node.cpp)间基于zmq进行通信，中继节点通过rostopic与上位机的其他客户端进行交互。
![](https://github.com/B3mylq/robot_data_pipeline_pkg/blob/main/img/communication_structure.png)

&emsp;&emsp;用户通过中继发布节点提供的RobotState消息获取下位机数据，格式定义于msg/RobotState.msg,各属性解释如下：

__RobotState数据结构__
| 属性        | 含义   |  备注  |
| :--------:   | :-----:  | :----:  |
| RobotState.q[18]     | 各关节电机位置 |        |
| RobotState.qd[18]     |   各关节电机角速度   |      | 
| RobotState.trq[18]  |    各关节电机扭矩    |    | 
|  RobotState.imu_data[9] |    Base惯性测量单元数据    |  [0~2]表示基体的roll,pitch,yaw角；[3~5]表示x,y,z轴角速度；[6~8]表示x,y,z轴加速度  |
| RobotState.body_pos[3]  |    Base的x,y,z轴位置    |  x,y轴位置为无效值，无法使用；z轴位置由足端运动学估计  |
| RobotState.body_vel[3]  |    Base的x,y,z轴速度    |  由足端逆运动学估计  |

&emsp;&emsp;用户通过中继接收节点提供的RobotCmd消息向下位机发布控制指令，格式定义于msg/RobotCmd.msg,各属性解释如下：

__RobotCmd数据结构__
| 属性        | 含义   |  备注  |
| :--------:   | :-----:  | :----:  |
| RobotCmd.q[18]     | 下发各关节目标位置指令 |        |
| RobotCmd.qd[18]     |   下发各关节目标角速度指令   |   该指令暂时无效，可直接赋0，下位机采用位置环PD控制器   | 
| RobotCmd.trq[18]  |    下发各关节目标力矩指令    |  该指令暂时无效，可直接赋0，下位机采用位置环PD控制器  | 
| RobotCmd.q_first_flag  |   表示该指令是否为第一帧，赋值为1表示第一帧，其他实数表示非第一帧   |  通常用在运行离线轨迹的情况下  | 

---
## 4、中继节点的使用和编辑
上位机中继发布节点的示例源码位于src/robot_state_pub_node.cpp；上位机中继接收节点的示例源码位于src/robot_state_sub_node.cpp。

上下位机间的通信频率由中继发布节点决定，上限为500Hz，频率和rostopic节点名称的定义方式详见两份示例源码

新开一个终端启动中继发布节点示例：
```bash 
rosrun robot_data_pipeline_pkg robot_state_pub
```
新开一个终端启动中继接收节点示例：
```bash 
rosrun robot_data_pipeline_pkg robot_state_sub
```

此时客户端可以通过rostopic向中继节点订阅状态和发布指令。

---
## 5、小狗实机操作指南--23/7/26(针对仓库更新时间的Jetson本地目录和实验需求，不一定长期有效)
### 5.1 启动ROS核心
新建一个终端，启动ROS核心：
```bash 
roscore
```
### 5.2 小狗进入上位机控制模式
按照第2章的要求启动小狗，在网页中执行recover指令使小狗恢复正常站立姿态；
在网页中点击RlContorl按键进入上位机控制模式。
### 5.3 启动中继发布节点
新建终端，进入rl_dog工作空间，启动中继发布节点。如果5.2操作正确，此时能看到终端中滚动发布下位机上发的数据。
```bash 
cd ~/ros_ws/rl_dog
source devel/setup.bash
rosrun robot_data_pipeline_pkg robot_state_pub
```
### 5.4 启动中继接收节点
新建终端，进入rl_dog工作空间，启动中继接收节点。
```bash 
cd ~/ros_ws/rl_dog
source devel/setup.bash
rosrun robot_data_pipeline_pkg robot_state_sub
```
### 5.5 用户启动自己编写的程序
以isaac gym网络测试为例，新建终端，进入rl_dog工作空间，可以打开pytorch虚拟环境。
```bash 
conda activate pytorch #根据环境需求选择是否执行
cd ~/ros_ws/rl_dog
source devel/setup.bash
rosrun rl_dog_test xxx.py #运行用户文件(纯CPG控制文件为little_dog_core_pureCPG.py)
```
### 5.6 注意事项
每次测试完成后，请将中继接收节点、中继发布节点和用户程序都中止。