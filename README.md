# Communication Interface Package for Littledog

## 1、配置和编译
### 1.1 安装支持库 
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

### 1.2 编译工作包
打开终端，创建或进入ROS工作空间，执行以下命令克隆项目并编译
```bash
cd src
git clone https://github.com/B3mylq/robot_data_pipeline_pkg
cd ..
catkin_make
source devel/setup.bash
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
