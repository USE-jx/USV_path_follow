# USV_motion_control
usv path follow, trajectory tracking,and so on 
编译报错自己解决不了就别看了，就一个LOS没啥有用的

整个算法的文件夹叫motion_control，其中包括control，guidance，control_manage三个包。

control这个文件夹以后专门实现路径跟随和轨迹跟踪的控制器，

guidance这个文件夹以后实现各种导引和规划器，

control_manage这个包实现ros节点调用上面两个包的规划控制算法，起名叫控制管理是因为我觉得导引和规划也是广义控制的一部分。
