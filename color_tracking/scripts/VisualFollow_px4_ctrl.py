#!/usr/bin/env python 
# coding=utf-8

import rospy 
import message_filters
import cv2, cv_bridge 
import numpy as np
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist , Vector3
from dynamic_reconfigure.server import Server
from color_tracking.cfg import HSV_ParamentConfig
from std_msgs.msg import Int8
from color_tracking.msg import position#自定义的追踪消息类型
#定义视觉跟踪类
class VisualFollower:
    def __init__(self):#类初始化
        #从参数服务器获取相关参数，这些参数在launch文件中定义
        self.vertAngle=rospy.get_param('~visual_angle/vertical') #定义摄像头垂直可视角度大小
        self.horizontalAngle=rospy.get_param('~visual_angle/horizontal') #定义摄像头水平可视角度大小
        self.centerRaw=np.array([0,0])

        self.position = position() #创建位置发布变量
        self.bridge = cv_bridge.CvBridge() #OpenCV与ROS的消息转换类
        #message_filters的作用是把订阅的数据同步后再在message_filters的回调函数中使用
        im_sub = message_filters.Subscriber('/camera/color/image_raw', Image)
        dep_sub = message_filters.Subscriber('/camera/depth/image_rect_raw', Image)
        self.timeSynchronizer = message_filters.ApproximateTimeSynchronizer([im_sub, dep_sub], 10, 0.5)
        self.timeSynchronizer.registerCallback(self.VisualFollow)

        self.position_pub = rospy.Publisher("landmark_position", position, queue_size=1) #目标位置发布者
        self.srv = Server(HSV_ParamentConfig, self.config_callback) 

    def publish_flag(self):
        visual_follow_flag=Int8()
        visual_follow_flag.data=1
        rospy.sleep(1.)
        visualfwflagPublisher.publish(visual_follow_flag)
        rospy.loginfo('a=%d',visual_follow_flag.data)

    # 动态参数配置
    def config_callback(self, config, level):

        self.color = config.color

        if self.color== 0:  #
            HSV_H_MIN =config.HSV_H_MIN
            HSV_S_MIN =config.HSV_S_MIN
            HSV_V_MIN =config.HSV_V_MIN
            HSV_H_MAX =config.HSV_H_MAX
            HSV_S_MAX =config.HSV_S_MAX
            HSV_V_MAX =config.HSV_V_MAX
            self.targetUpper=np.array([HSV_H_MIN,HSV_S_MIN,HSV_V_MIN])
            self.targetLower=np.array([HSV_H_MAX,HSV_S_MAX,HSV_V_MAX])
        elif self.color == 1:  #
            self.targetUpper=self.col_red_U
            self.targetLower=self.col_red_L

        elif self.color== 2:  #
            self.targetUpper=self.col_blue_U
            self.targetLower=self.col_blue_L

        elif self.color== 3:  #
            self.targetUpper=self.col_green_U
            self.targetLower=self.col_green_L

        elif self.color== 4:  #
            self.targetUpper=self.col_yellow_U
            self.targetLower=self.col_yellow_L

        return config
        

    #视觉跟踪实现函数
    def VisualFollow(self, image_data, depth_data):
        #图像转换与预处理            
        frame = self.bridge.imgmsg_to_cv2(image_data, desired_encoding='bgr8')            #ROS图像转OpenCV图像
        depthFrame = self.bridge.imgmsg_to_cv2(depth_data, desired_encoding='passthrough')#ROS图像转OpenCV图像
        frame = cv2.resize(frame, (320,240), interpolation=cv2.INTER_AREA)            #降低图像分辨率，以提高程序运行速度        
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        depthFrame = cv2.resize(depthFrame, (320,240), interpolation=cv2.INTER_AREA)  #降低图像分辨率，以提高程序运行速度        
        frame_height, frame_width, frame_channels = frame.shape #获得图像尺寸高度、宽度、通道数
        frame_threshold = cv2.inRange(frame, self.targetUpper, self.targetLower)

        #膨胀腐蚀
        kernel_width=5;  
        kernel_height=5; 
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (kernel_width, kernel_height))
        frame_threshold = cv2.erode(frame_threshold, kernel)
        frame_threshold = cv2.dilate(frame_threshold, kernel)
        frame_threshold = cv2.dilate(frame_threshold, kernel)
        frame_threshold = cv2.erode(frame_threshold, kernel)

        #寻找轮廓API，功能：输入图片，返回原图、轮廓对象、轮廓父子结构
        contours= cv2.findContours(frame_threshold.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2] #cv2.RETR_EXTERNAL：只检测外轮廓，cv2.CHAIN_APPROX_SIMPLE)：只保留轮廓的交点

        try:
            contours_sorted=sorted(contours, key=cv2.contourArea, reverse=True) #按照轮廓面积进行排序
            cv2.drawContours(frame, contours_sorted, 0, (0,0,255), 3)           #画出第一个轮廓，因为已排序，所以即为最大的轮廓

            (x, y),radius = cv2.minEnclosingCircle(contours_sorted[0]) #对最大轮廓求最小外接圆，并返回中心坐标(浮点型)，半径(浮点型)
            x=x-12
            x_float=x #保留浮点型，用于后面计算角度
            y_float=y #保留浮点型，用于后面计算角度
            x=int(x) #转为整型才能用于图像处理
            y=int(y) #转为整型才能用于图像处理
            radius=int(radius) #转为整型才能用于图像处理
            cv2.circle(frame, (x, y), radius, (0, 255, 255), 3) #画出最小外接圆
            target_radius=radius/3 #用于简单求轮廓内接矩形
            depthObject = depthFrame[(y-target_radius):(y+target_radius), (x-target_radius):(x+target_radius)]#截取目标区域的深度图， 用于取区域内深度信息, frame[(height_start:height_end), (width_start:height_end)]
            cv2.rectangle(frame,(x-target_radius,y-target_radius),(x+target_radius, y+target_radius), (0,255,0), 2)#画出轮廓内接矩形
            depthArray = depthObject[~np.isnan(depthObject)]#获取区域内非空的深度信息
            averageDistance = np.mean(depthArray)           #求平均得到与目标的距离，单位:毫米

            angleX=self.horizontalAngle*(0.5-x_float/frame_width)
            angleY=self.vertAngle*(0.5-y_float/frame_height)

            #打印偏角、距离信息
            rospy.loginfo("angleX:"+str(angleX))
            rospy.loginfo("angleY:"+str(angleY))
            rospy.loginfo("distance:"+str(averageDistance))
      
	    self.position.angleX = angleX
	    self.position.angleY = angleY
	    self.position.distance = averageDistance
	    self.position.iffind = True 
            self.position_pub.publish(self.position)
        #没有找到轮廓，速度控制命令置零
        except IndexError:
            rospy.logwarn("contours not found")
	    self.position.iffind = False
	    self.position_pub.publish(self.position)

        #cv2.imshow("frame2", frame) #显示识别效果
        #cv2.waitKey(1)

if __name__ == '__main__': 
    rospy.init_node("opencv") 
    visualfwflagPublisher = rospy.Publisher('/visual_follow_flag', Int8, queue_size =1)
    rospy.loginfo("OpenCV VisualFollow node started") 
    Visual_follower=VisualFollower() 
    rospy.spin() 

