---
layout: post
title: ROS Navigation之fake_localization
date: 2018-10-23
categories: 
- Ros Navigation
tags: ros
status: publish
type: post
published: true
meta:
  _edit_last: '1'
  views: '2'
author:
  login: laujinseoi
  email: laujinseoi@yeah.com
  display_name: 
comments: true
---

#### 简介
>本包提供一个node，fake_localization，用来替代定位系统，提供可被amcl使用的接口，常在仿真中
使用，计算量更少。它将odometry数据转换成位姿信息、粒子云、tf数据，数据格式与amcl发布的数据相同

#### 源码分析
 ```c++
void update(const nav_msgs::OdometryConstPtr& message)
{
  tf::Pose txi;
  // 这里将ros odom的格式转换成tf的格式，得到txi
  tf::poseMsgToTF(message->pose.pose, txi);
      
  // 偏差矩阵×上odom的数值，目的是为了得到一个基于地图的机器人位置值
  tf::Pose txi_new = m_offsetTf * txi;
  tf::Stamped<tf::Pose> odom_to_map;
  try
  {
    // 从base_frame_id到odom，就是在base_footrpint上表示odom的坐标值，而对于odom来说，这个点即是odom
    m_tfListener->transformPose(odom_frame_id_, tf::Stamped<tf::Pose>(txi_new.inverse(), message->header.stamp, base_frame_id_), odom_to_map);
  }
  catch(tf::TransformException &e)
  {
    ROS_ERROR("Failed to transform to %s from %s: %s\n", odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
    return;
  }
```

上面将txi.inverse转换到odom上，就可以得到map相对于odom的坐标关系了。这是为什么呢？请看下图：
![frame.png](https://raw.githubusercontent.com/laujinseoi/laujinseoi.github.io/master/images/post/fake_localization1.png)

首先，一开始我们得到机器人相对于odom的位置值，其实就是base_link相对于odom的相对关系txi

然后通过m_offsetTF就得到了base_link相对于map的相对坐标关系txi_new = m_offsetTF×txi

接着通过求txi的逆矩阵即txi_new.inverse得到map相对于base_link的相对关系

因为一开始odom和base_link的相对关系是已知的，并且已经存在tf中的，则通过tf的转换函数，就可以得到map相对于odom的相对关系了。
事实上，如果是通过rviz的
![2d_pose.png](https://raw.githubusercontent.com/laujinseoi/laujinseoi.github.io/master/images/post/2d_pose_estimate.png)改变机器人的位置，odom相对于map的关系就是m_offsetTF。

```c++
  // 发布tf，从map到odom
  m_tfServer->sendTransform(tf::StampedTransform(odom_to_map.inverse(),
                                                 message->header.stamp + ros::Duration(transform_tolerance_),
                                                 global_frame_id_, message->header.frame_id));
  // 模拟amcl发布机器人估计后的位置
  tf::Pose current;
  tf::poseMsgToTF(message->pose.pose, current);
  //also apply the offset to the pose
  current = m_offsetTf * current;
  geometry_msgs::Pose current_msg;
  tf::poseTFToMsg(current, current_msg);
  // Publish localized pose
  m_currentPos.header = message->header;
  m_currentPos.header.frame_id = global_frame_id_;
  m_currentPos.pose.pose = current_msg;
  m_posePub.publish(m_currentPos);

  // The particle cloud is the current position. Quite convenient.
  m_particleCloud.header = m_currentPos.header;
  m_particleCloud.poses[0] = m_currentPos.pose.pose;
  m_particlecloudPub.publish(m_particleCloud);
}
```
对fake_localization的分析就到此为止，总体而言还是非常简单的。

#### 参考资料
[fake_localization](http://wiki.ros.org/fake_localization)

[fake_localization说明](https://sychaichangkun.gitbooks.io/ros-tutorial-icourse163/content/chapter10/navigation%E5%B7%A5%E5%85%B7%E5%8C%85%E8%AF%B4%E6%98%8E/fake_localization%E8%AF%B4%E6%98%8E.html)
