# ROS自学笔记

标签（空格分隔）： ROS 笔记

---

## **ROS系统架构**

 主要被设计了三个部分，每个部分代表一个层级的概念
- 文件系统级
- 计算图级
- 开源社区级

### **文件系统级**
```graphTD
    A(文件系统级) --> B[综合功能包]
    B --> C[功能包]
    C --> D[功能包清单]
    C --> E[消息]
    C --> F[服务]
    C --> G[代码]
    C --> H[其他]
```

- **功能包** 是最小的组成单元
- **功能包清单** 由package.xml文件管理，提供功能包、许可信息、依赖关系、编译标志等信息
- **消息类型** 其说明存储在对应功能包的msg文件夹下（my_package/msg/MyMessageType.msg 中）
- **服务类型** 其说明存储在对应功能包的srv文件夹下（my_package/srv/MyServiceType.srv 中），定义了在ROS中每个进程提供的关于服务请求和相应的数据结构
- **综合功能包** 将几个具有某些功能的功能包组织在一起，就可以得到一个综合功能包。如导航功能包集
- **综合功能包清单** 类似普通包，但有一个XML格式的导出标记，在结构上也有一定的限制

#### **工作空间**
工作空间是一个包含功能包、可编辑源文件或编译包的文件夹。可同时便宜不同的功能包，并且可以用来保存本地开发包

一般用catkin_ws文件夹作为工作空间，其中一般包含/build、/devel以及/src文件夹

- **源文件空间** 
一般使用/src文件夹作为源文件空间，其中放置了功能包、项目、克隆包等。其中CMakeList.txt很重要，当在工作空间配置功能包时，src文件夹CMakeList.txt调用CMake。该文件通过
```
catkin_init_workspace
```
命令创建。

- **编译空间**
一般使用/build文件夹作为编译空间，在该文件夹中，CMake和catkin为功能包和项目保存缓存信息、配置和其他中间文件
- **开发空间**
一般使用/devel文件夹作为开发空间，用来保存编译后的程序，这些是无需安装就能用来测试的程序

catkin编译包由两个选项。

- **第一个**是使用标准的CMake工作流程，通过此方式可以一次编译一个包
```
cmake packageToBuild/
make
```
- **第二个** 是编译所有的包，可以使用catkin_make命令行
```
cd workspace
catkin_make
```
#### **功能包**

- include/package_name/:此目录包含了需要的库的头文件
- msg/：如果需要开发非标准消息，需要将文件放在这里
- scripts/：其中包括Bash、Python或任何其他脚本的可执行脚本文件
- src/：这是存储程序源文件的地方
- srv/：表示服务的类型
- CMakeLists.txt CMake的生成文件
- package.xml 功能包清单文件

**常用功能包使用指令与工具**
- rospack 获取信息或在系统中查找工作空间
- catkin_create_pkg 创建性的功能包
- cakin_make 编译工作空间
- rosdep 安装功能包的系统依赖项
- rqt_dep 查看包的依赖关系，可在rqt发现一个包图（package graph）插件，选择一个包并查看依赖关系

**常用功能包文件操作指令与工具**
- roscd 对应Linux中cd
- rosed 编辑文件
- roscp 复制文件
- rosd 列出功能包的目录
- roscd 对应Linux中ls

## ROS节点编程

### 一、简单的发布节点
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
```
第一行包含了使用ROS节点所有必要的文件
第二行包含了需要使用的消息类型（根据所需使用的消息类型不同而包含不同的头文件）

```cpp
ros::init(argc,argv,"example1_a");
```
启动节点并设置**节点**名称，**节点**名称必须是唯一的
```cpp
ros::NodeHandle n;
```
设置节点进程的句柄,第一个创建的节点句柄将会初始化这个节点，最后一个销毁的节点将会释放节点所使用的所有资源。
```cpp
ros::Publisher chatter_pub = n.advertise<std_msgs::String>("message",1000);
```
将节点设置成发布者，
告诉主机所发布的主题和类型的名称(我们将会在一个名字为message的**话题（Topic）**上发布一个std_msgs/String类型的消息)，并将告知节点管理器,这就使得主机告诉了所有订阅了message话题的节点，我们将在这个话题上发布数据。第二个参数是发布队列的大小，它的作用是缓冲。当我们发布消息很快的时候，它将能缓冲1000条信息。如果慢了的话就会覆盖前面的信息。

NodeHandle::advertise()将会返回ros::Publisher对象，该对象有两个作用，首先是它包括一个publish()方法可以在制定的话题上发布消息，其次，当超出范围之外的时候就会自动的处理。

```cpp
ros::Rate loop_rate(10);  
```
一个ros::Rate对象允许你制定循环的频率。它将会记录从上次调用Rate::sleep()到现在为止的时间，并且休眠正确的时间。在这个例子中，设置的频率为10Hz(设置发送数据的频率为10Hz)。

```cpp
while (ros::ok())
{
```
默认情况下，roscpp将会安装一个SIGINT监听，ros::ok()在以下几种情况下也会返回false：（1）按下Ctrl-C时（2）我们被一个同名同姓的节点从网络中踢出（3）ros::shutdown()被应用程序的另一部分调用（4）所有的ros::NodeHandles都被销毁了。一旦ros::ok()返回false，所有的ROS调用都会失败。

```cpp
std_msgs::String msg;  
std::stringstream ss;  
ss << "hello world " << count;  
msg.data = ss.str();  
```
我们使用message-adapted类在ROS中广播信息，这个类一般是从msg文件中产生的。我们现在使用的是标准的字符串消息，它只有一个data数据成员，当然更复杂的消息也是可以的。

```cpp
chatter_pub.publish(msg);  
```
现在我们向话题message发布消息。
```cpp
ROS_INFO("%s", msg.data.c_str());  
```
ROS_INFO等同于cout和printf。

```cpp
ros::spinOnce();
```
在这个简单的程序中调用ros::spinOnce();是不必要的，因为我们没有收到任何的回调信息。然而如果你为这个应用程序添加一个订阅者，并且在这里没有调用ros::spinOnce()，你的回调函数将不会被调用。所以这是一个良好的风格。

```cpp
loop_rate.sleep();  
```

休眠一下，使程序满足前面所设置的10Hz的要求。

**完整程序如下**
```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc,argv,"example1_a");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("message",1000);
  ros::Rate loop_rate(10); 
  while (ros::ok())
    {
      std_msgs::String msg;  
      std::stringstream ss;  
      ss << "hello world " << count;  
      msg.data = ss.str();  
      chatter_pub.publish(msg);  
      ROS_INFO("%s", msg.data.c_str());  
      ros::spinOnce();
      loop_rate.sleep();  
     }
  return 0;
}
```

### 二、简单的订阅节点

```cpp
#include "ros/ros.h"
#include "std_msgs/String.h"
```
第一行包含了使用ROS节点所有必要的文件
第二行包含了需要使用的消息类型（根据所需使用的消息类型不同而包含不同的头文件）

