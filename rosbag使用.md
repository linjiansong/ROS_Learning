# C++
## Example usage for write: 
```
#include <rosbag/bag.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

rosbag::Bag bag;
bag.open("test.bag", rosbag::bagmode::Write);

std_msgs::String str;
str.data = std::string("foo");

std_msgs::Int32 i;
i.data = 42;

bag.write("chatter", ros::Time::now(), str);
bag.write("numbers", ros::Time::now(), i);

bag.close();
```

## Example usage for read:
```

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

rosbag::Bag bag;
bag.open("test.bag", rosbag::bagmode::Read);//打开一个bag文件

std::vector<std::string> topics;//设置需要遍历的topic
topics.push_back(std::string("chatter"));
topics.push_back(std::string("numbers"));

rosbag::View view(bag, rosbag::TopicQuery(topics));// 读指定的topic，如果全读，第二个参数不写，这样：rosbag::View view_all(view);

foreach(rosbag::MessageInstance const m, view)//按时间顺遍历每一个topic
{
   std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();// 将当前topic取为String类型，如果不是，则得到NULL
   if (s != NULL)
     std::cout << s->data << std::endl;
     
   std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();/// 将当前topic取为Int32类型，如果不是，则得到NULL
   if (i != NULL)
     std::cout << i->data << std::endl;
}

bag.close();
```
