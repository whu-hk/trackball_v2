# trackball_v2
基于HSV颜色过滤、KCF跟踪的红蓝球识别
基于verison_v1更新：
1.加入蓝色的HSV颜色空间，并设置识别优先级：红 > 蓝，大>小
2.trackball.launch文件加入respawn = "true",可以在节点崩溃后自动重启
3.取消了空ROI类型消息的发布，只发布跟踪两帧以上的跟踪框ROI
实际运行要修改：
4.图片订阅话题 Line 60，调试时修改成了“camera/bgr/image_raw”,注意改回来
5.红色的HSV颜色空间中的H范围 Line49 & Line50,调试时修改成0～10,注意修改到实际范围
