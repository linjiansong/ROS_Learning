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
# 建立几个cpp的源文件，位于src目录下。
## src/example_srv_request.cpp
作用：主要实现服务的定义，初始化。
```
    #include "ros/ros.h"
    #include "beginner_tutorials/AddTwoInts.h"
     
    bool add(beginner_tutorials::AddTwoInts::Request &req,
             beginner_tutorials::AddTwoInts::Response &res)
    {
        res.sum=req.A+req.B+req.C;
        ROS_INFO("request: A=%ld,B=%ld,C=%ld",(int)req.A,(int)req.B,(int)req.C);
        ROS_INFO("sending back response:[%ld]",(int)res.sum);
     
        return true;
    }

    int main(int argc,char **argv)
    {
       ros::init(argc,argv,"add_3_ints_server");
       ros::NodeHandle n;
       //init service:
       //Name: add_3_ints,advertise the service. add: call back function
       ros::ServiceServer service=n.advertiseService("add_3_ints",add);
       ROS_INFO("Ready to add 3 ints.");
       ros::spin();// start to deal with the call back event
     
       return 0;
    }
```
## src/example_srv_response.cpp
该文件并非必要，是典型的client调用服务的形式，并非此文目的，不添加最终目的也可实现。
```
    #include "ros/ros.h"
    #include "beginner_tutorials/AddTwoInts.h"
    #include <cstdlib>
     
     
    int main(int argc,char **argv)
    {
        ros::init(argc,argv,"add_3_ints_client");
        if(argc != 4)
        {
            ROS_INFO("usage: add_3_ints_client A B C");
            return 1;
        }
        ros::NodeHandle n;
        ros::ServiceClient client=n.serviceClient<beginner_tutorials::AddTwoInts>("add_3_ints");
     
        beginner_tutorials::AddTwoInts srv;
     
        //atoll:convert a string to long long int variable
        srv.request.A=atoll(argv[1]);//把字符串转换成长长整型数
        srv.request.B=atoll(argv[2]);
        srv.request.C=atoll(argv[3]);
     
     
        if(client.call(srv))
        {
            ROS_INFO("Sum:%ld",(long int)srv.response.sum);
        }
        else
        {
            ROS_ERROR("Failed to call service add_three_ints");
            return 1;
     
        }
        return 0;
    }
```
## src/example_talker_msg.cpp
该文件主要实现3个参数的发布，此处发布了A=1,B=2,C=3；
```
    #include "ros/ros.h"
    #include "beginner_tutorials/Num.h"
    #include <sstream>
    //Usage:publish a msg with topic named message
 
    int main(int argc, char **argv)
    {
      ros::init(argc, argv, "example_talker_msg");
      ros::NodeHandle n;
      //advertise a topic named "message"
      ros::Publisher pub=n.advertise<beginner_tutorials::Num>("message",1000);
      ros::Rate loop_rate(10);
      while (ros::ok())
      {
        beginner_tutorials::Num msg;
        msg.A=1;
        msg.B=2;
        msg.C=3;
        pub.publish(msg);
     
        ros::spinOnce();
        loop_rate.sleep();
      }
      return 0;
    }
```

## src/example_listener_msg.cpp

该文件主要实现对前文talker发布的数据的监听，监听到Num类型定义数据后，通过service: ros::service::call / client.call()两种方式进行add_3_ints服务的调用。
```
    #include "ros/ros.h"
    #include "beginner_tutorials/Num.h"
    #include "beginner_tutorials/AddTwoInts.h"
     
    //declaration of the Example class:construct function, member and so on.
    class Example{
    public:
        Example();
    private:
        ros::NodeHandle nh;
        ros::ServiceClient client;
        ros::Subscriber listener;
        void messageCallback(const beginner_tutorials::Num::ConstPtr& msg);
     
    };
     
    //implementation of construct function.
    Example::Example()
    {
        client=nh.serviceClient<beginner_tutorials::AddTwoInts>("add_3_ints");
        //monitor a topic named "message",this -> current object,pointer
        listener=nh.subscribe("message",1000,&Example::messageCallback,this);
    }
    //implementation of one of the Example class memeber.
    void Example::messageCallback( const beginner_tutorials::Num::ConstPtr& msg)
    {
    //    ROS_INFO("I heard:[%d] [%d] [%d]",msg->A,msg->B,msg->C);
        beginner_tutorials::AddTwoInts srv;
        srv.request.A=msg->A;
        srv.request.B=msg->B;
        srv.request.C=msg->C;
        if(ros::service::call("add_3_ints",srv))
            ROS_INFO("SUM: %ld",(long int)srv.response.sum);
        else
            ROS_ERROR("Failed to call service add_three_ints");
        /*
        if(client.call(srv))
        {
            ROS_INFO("SUM: %ld",(long int)srv.response.sum);
        }
        else
        {
            ROS_ERROR("Failed to call service add_three_ints");
        }
        */
    }

    int main(int argc, char **argv)
    {
      ros::init(argc, argv, "example_listener_msg");
      Example exp;
      ros::spin();
      return 0;
    }
    ```
# 再次修改CMakeList.txt文件
添加编译链接的相关控制语句：
```
    add_executable(example_srv_request src/example_srv_request.cpp)
    target_link_libraries(example_srv_request ${catkin_LIBRARIES})
     
    add_executable(example_srv_response src/example_srv_response.cpp)
    target_link_libraries(example_srv_response ${catkin_LIBRARIES})
     
    add_executable(example_talker_msg src/example_talker_msg.cpp)
    target_link_libraries(example_talker_msg ${catkin_LIBRARIES})
     
    add_executable(example_listener_msg src/example_listener_msg.cpp)
    target_link_libraries(example_listener_msg ${catkin_LIBRARIES})
```
