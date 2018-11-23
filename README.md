# trackball_v2
基于HSV颜色过滤、KCF跟踪的红蓝球识别
##（1）基于verison_v1更新：
1.加入蓝色的HSV颜色空间，并设置识别优先级：红 > 蓝，大>小;
2.trackball.launch文件加入respawn = "true",可以在节点崩溃后自动重启;
3.取消了空ROI类型消息的发布，只发布成功跟踪60次以上的跟踪框ROI;
##（2）实际运行要修改：
1.图片订阅话题 Line 60，调试时修改成了“camera/bgr/image_raw”,注意改回来;
2.红色的HSV颜色空间中的H范围 Line49 & Line50,调试时修改成0～10,注意修改到实际范围;
##（3）重要参数调整：
### 1.HSV颜色空间，即红蓝两色的H范围，各自窗口可调；
### 2.self.track_count_thread = 90（Line 47）,发布ROI前必须成功跟踪90次以上，考虑/roi发布速率较慢的话调至60，考虑检测到的框不符合要求（红色，最大）可调高该值，保证足够长的时间检测到符合要求的球；
### 3.self.red_area_percent = rospy.get_param("~red_area_percent", 75)
  self.blue_area_percent = rospy.get_param("~blue_area_percent", 85),各自窗口可调，
  红蓝两色，轮廓面积与轮廓外接圆面积的比，调低可放松要求，接受更多的不完整轮廓并判定为圆；
### 4.Line 312:判断新检测到圆是否比当前追踪的圆更大，外接矩形的长宽差均大于2，矩形中心的两轴坐标差值均大于5，自己调整


# 说明：在实验室做这种需要现场调试的活，难受啊，调参大法好啊
