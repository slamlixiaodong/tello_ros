#!/usr/bin/env python
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import Image 
from sensor_msgs.msg import Imu

import cv2
import numpy as np
import time
from tello import Tello
import re
import math
import threading


IMAGE_WIDTH=960
IMAGE_HEIGHT=720


rospy.init_node("listener", anonymous=True)
image_pubulish=rospy.Publisher('/camera/image_raw',Image,queue_size=1)
IMU_pubulish=rospy.Publisher('/Imu/imu_raw',Imu,queue_size=20)


drone=Tello()
def eularToQuaternion( yaw, roll, pitch):
    X = yaw/180*math.pi
    Y = roll/180*math.pi
    Z = pitch/180*math.pi
    x = math.cos(Y/2)*math.sin(X/2)*math.cos(Z/2)+math.sin(Y/2)*math.cos(X/2)*math.sin(Z/2)
    y = math.sin(Y/2)*math.cos(X/2)*math.cos(Z/2)-math.cos(Y/2)*math.sin(X/2)*math.sin(Z/2)
    z = math.cos(Y/2)*math.cos(X/2)*math.sin(Z/2)-math.sin(Y/2)*math.sin(X/2)*math.cos(Z/2)
    w = math.cos(Y/2)*math.cos(X/2)*math.cos(Z/2)+math.sin(Y/2)*math.sin(X/2)*math.sin(Z/2)
    quat = [x,y,z,w]
    return quat 

def publish_image():
    if drone.is_new_frame_ready():
        frame = drone.get_last_frame()
        #gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        #edged = cv2.Canny(gray, 30, 150)
        #cv2.imshow("image",frame)
        image_temp=Image()
        header = Header(stamp=rospy.Time.now())
        header.frame_id = 'map'
        image_temp.height=IMAGE_HEIGHT
        image_temp.width=IMAGE_WIDTH
        image_temp.encoding='rgb8'
        image_temp.data=np.array(frame).tostring()
    #print(imgdata)
    #image_temp.is_bigendian=True
        image_temp.header=header
        image_temp.step=960*3
        time.sleep(0.1)
        image_pubulish.publish(image_temp)
        

def imu_acc():
    imu_acc = str(drone.get_acceleration())
    imu_acc = re.findall(r"\d+\.?\d*",imu_acc)
    return imu_acc

def imu_att():
    imu_att = str(drone.get_attitude())
    imu_att = re.findall(r"\d+\.?\d*",imu_att)
    return imu_att

def publish_imu():
    imu_ac = imu_acc()
    imu_at = imu_att()
    p=float(imu_ac[0])
    r=float(imu_ac[1])
    y=float(imu_ac[2])
    quat=eularToQuaternion(y,r,p)
    imu_temp=Imu()
    header= Header(stamp=rospy.Time.now())
    header.frame_id = 'map'
    imu_temp.orientation.x =float(quat[0])
    imu_temp.orientation.y =float(quat[1])
    imu_temp.orientation.z =float(quat[2])
    imu_temp.orientation.w =float(quat[3])
    imu_temp.linear_acceleration.x = float(imu_ac[0])
    imu_temp.linear_acceleration.y = float(imu_ac[1])
    imu_temp.linear_acceleration.z = float(imu_ac[2])
    imu_temp.angular_velocity.x = math.sqrt(float(imu_ac[0])/0.01)
    imu_temp.angular_velocity.y = math.sqrt(float(imu_ac[1])/0.01)
    imu_temp.angular_velocity.z = math.sqrt(float(imu_ac[2])/0.01)
    imu_temp.header=header
    IMU_pubulish.publish(imu_temp) 


#def thread_imu():
    #while True:
#        publish_image()
#def thread_image():
#    while True:
#        publish_imu()

while True:
    publish_image()

#threads = []
#t_imu = threading.Thread(target=thread_imu)
#t_image = threading.Thread(target=thread_image)
#threads.append(t_imu)
#threads.append(t_image)


#if __name__=='__main__':
#    for t in threads:
#        t.start()
#    for t in threads:
#        t.join()
