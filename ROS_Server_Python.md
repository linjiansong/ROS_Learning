# service简介
service方式在通信模型上与topic做了区别。Service通信是双向的，它不仅可以发送消息，同时还会有反馈。所以service包括两部分，一部分是请求方（Clinet），另一部分是应答方/服务提供方（Server）。这时请求方（Client）就会发送一个request，要等待server处理，反馈回一个reply，这样通过类似“请求-应答”的机制完成整个服务通信。
这种通信方式的示意图如下：
Node B是server（应答方），提供了一个服务的接口，叫做/Service，我们一般都会用string类型来指定service的名称，类似于topic。Node A向Node B发起了请求，经过处理后得到了反馈。

# 工作空间与功能包的创建
```
# 创建工作空间
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace

# 编译工作空间
$ cd ~/catkin_ws/
$ catkin_make

# 设置环境变量
$ source ~/catkin_ws/devel/setup.bash

# 创建功能包
$ cd ~/catkin_ws/src
$ catkin_create_pkg learning_service rospy std_msgs geometry_msgs turtlesim

# 编译功能包
$ cd ~/catkin_ws
$ catkin_make
```
#话题消息.srv的定义
## Person.srv
```
（以"—"来隔开请求和响应）

string name
uint8  age
uint8  sex

uint8 unknown = 0
uint8 male    = 1
uint8 female  = 2

---
string result
```
## 编译

    在package.xml中添加功能包依赖：
```
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```
    在CMakeLists.txt添加编译选项：
```
find_package( ...... message_generation)
add_service_files(FILES Person.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs rospy std_msgs turtlesim message_runtime)
```
  编译：
```
$ cd ~/catkin_ws
$ catkin_make
```

# 客户端Client的编写
```
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将请求/show_person服务，服务数据类型learning_service::Person

import sys
import rospy
from learning_service.srv import Person, PersonRequest

def person_client():
	# ROS节点初始化
    rospy.init_node('person_client')

	# 发现/spawn服务后，创建一个服务客户端，连接名为/spawn的service
    rospy.wait_for_service('/show_person')
    try:
        person_client = rospy.ServiceProxy('/show_person', Person)

		# 请求服务调用，输入请求数据
        response = person_client("Tom", 20, PersonRequest.male)
        return response.result
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
	#服务调用并显示调用结果
    print "Show person result : %s" %(person_client())
```
# 服务端Server的编写
```
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# 该例程将执行/show_person服务，服务数据类型learning_service::Person

import rospy
from learning_service.srv import Person, PersonResponse

def personCallback(req):
	# 显示请求数据
    rospy.loginfo("Person: name:%s  age:%d  sex:%d", req.name, req.age, req.sex)

	# 反馈数据
    return PersonResponse("OK")

def person_server():
	# ROS节点初始化
    rospy.init_node('person_server')

	# 创建一个名为/show_person的server，注册回调函数personCallback
    s = rospy.Service('/show_person', Person, personCallback)

	# 循环等待回调函数
    print "Ready to show person informtion."
    rospy.spin()

if __name__ == "__main__":
    person_server()
```
