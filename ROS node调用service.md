# 简介
在一个node里调用现有的rosservice list查询到的service,通过talker publish需要调用服务的数据，listener接收到数据后调用服务进行三个整形数据相加，并返回相加和。

# 创建msg文件夹，srv文件夹。
msg文件夹下创建Num.msg，内容是:
```
int32 A
int32 B
int32 C
```
srv文件夹下AddTwoInts.srv，内容是
```
int32 A
int32 B
int32 C
---
int32 sum
```
该文件表面输入是ABC，返回是 sum，两者通过“---”分割。
# 然后更改CMakeList.txt文件，修改为如下内容，主要设计msg，srv的其他依赖关系及编译运行的相关关系。
```
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)
add_message_files(
  FILES
  Num.msg
)
add_service_files(
  FILES
  AddTwoInts.srv
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES beginner_tutorials
  #  CATKIN_DEPENDS roscpp rospy std_msgs
  #  DEPENDS system_lib
  CATKIN_DEPENDS message_runtime
)
```

