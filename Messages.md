# 1、sensor_msgs
## （1）sensor_msgs/LaserScan————激光数据
```
std_msgs/Header header //timestamp in the header is the acquisition time of the first ray in the scan.  
  uint32 seq  
  time stamp  
  string frame_id //固定坐标系  
float32 angle_min //最小的扫描角度值  
float32 angle_max //最大的扫描角度值  
float32 angle_increment //每次扫描增加的角度  
float32 time_increment //测量的时间间隔  
float32 scan_time //扫描的时间间隔  
float32 range_min //最小测距范围  
float32 range_max //最大测距范围  
float32[] ranges //一周的测量数据,一共360个(NaN,Not a Number--非数字;INF,即,Infinite--无穷大)  
float32[] intensities //强度的意思就是返回光的强度。代表了反射面的情况
```
## （2）std_msgs/Header.msg————文件头
```
uint32 seq //数据队列对应的ID，sequence ID: consecutively increasing ID 
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
  stamp.sec:
  stamp.nsec：
#Frame this data is associated with
string frame_id
```

