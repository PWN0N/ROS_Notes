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

