#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
import cv2
from geometry_msgs.msg import Quaternion
import threading
from aptdaemon import lock
import time
lock = threading.Lock()


def getYaw(orientation):
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
    return yaw


# 直線のピクセル値を取得
def LineIterator(p1, p2):
    p1x = p1[0]
    p1y = p1[1]
    p2x = p2[0]
    p2y = p2[1]

    dx = p2x - p1x
    dy = p2y - p1y

    xnum = np.abs(dx) + 1
    ynum = np.abs(dy) + 1

    if(xnum < ynum):
        slope = dx.astype(np.float32) / dy.astype(np.float32)
        steep = True
    else:
        slope = dy.astype(np.float32) / dx.astype(np.float32)
        steep = False

    # x,yのうち、成分が多い方を基準にする
    line_pixnum = np.maximum(xnum, ynum)
    line = np.empty(shape=(line_pixnum, 2), dtype=np.int)

    # ブレゼンハムのアルゴリズム
    if steep:
        # yをベースに、xを求める
        if p1y > p2y:
            line[:, 1] = np.arange(p1y, p2y - 1, -1)
        else:
            line[:, 1] = np.arange(p1y, p2y + 1, 1)
        line[:, 0] = (slope * (line[:, 1] - p1y)).astype(np.int) + p1x
    else:
        # xをベースに、yを求める
        if p1x > p2x:
            line[:, 0] = np.arange(p1x, p2x - 1, -1)
        else:
            line[:, 0] = np.arange(p1x, p2x + 1, 1)
        line[:, 1] = (slope * (line[:, 0] - p1x)).astype(np.int) + p1y

    return line.astype(int)


class DummyScan:

    def __init__(self, map_topic, scan_topic, sensor_frame, scan_base_frame, scan_range_min, scan_range_max, angle_min, angle_max, resolution):
        self.map_topic = map_topic

        self.tf_listener = tf.TransformListener()

        self.scan_pub = rospy.Publisher(scan_topic, LaserScan, queue_size=1)
        self.map_sub = rospy.Subscriber(map_topic, OccupancyGrid, self.on_map_received)

        self.scan = LaserScan()
        self.scan.header.frame_id = sensor_frame
        self.scan.angle_min = angle_min
        self.scan.angle_max = angle_max
        self.scan.angle_increment = resolution
        self.scan.time_increment = 0
        self.scan.scan_time = 0
        self.scan.range_min = scan_range_min  #
        self.scan.range_max = scan_range_max
        
        self.map_base_frame = ""
        self.resolution = 0
        self.sensor_frame = sensor_frame
        self.origin_x_pix = 0
        self.origin_y_pix = 0
        self.prev_time = rospy.get_time()
        self.scan_base_frame = scan_base_frame
        
    def on_map_received(self, msg):
        global lock
        lock.acquire()
        self.map_base_frame = msg.header.frame_id
        info = msg.info
        self.resolution = info.resolution  # [m/cell]
        width = info.width  # cells
        height = info.height  # cells
        origin_x = info.origin.position.x  # マップ原点から見た、画像左下位置[m]
        origin_y = info.origin.position.y
        origin_z = info.origin.position.z
        origin_yaw = getYaw(info.origin.orientation)
        map_tmp = np.array(msg.data).reshape(height, width).astype(np.uint8)
        map_tmp = np.where(map_tmp == 255, 0, map_tmp)
        self.origin_x_pix = -int(origin_x / self.resolution)
        self.origin_y_pix = height + int(origin_y / self.resolution)
        self.origin_yaw_rad = origin_yaw

        # マップの回転は未対応
        angle = math.degrees(origin_yaw)
        scale = 1.0
        center = (int(width / 2), int(height / 2))
        trans = cv2.getRotationMatrix2D(center, angle, scale)
        self.map = cv2.warpAffine(map_tmp, trans, (width, height))
        
        print(self.map_base_frame, self.resolution, width, height)
        print(origin_x, origin_y, origin_yaw)

        lock.release()

    def make_scan_data(self):
        global lock
        lock.acquire()
        if(self.resolution == 0):
            lock.release()
            return
        try:
            # センサ位置を取得
            t = rospy.Time(0)
            sensor_pos, sensor_rot = self.tf_listener.lookupTransform(self.scan_base_frame, self.sensor_frame, t)
        except (Exception):
            print("tf transform could not be solved. from:", self.scan_base_frame, " to:", self.sensor_frame)
            lock.release()
            return
        (sensor_roll, sensor_pitch, sensor_yaw) = tf.transformations.euler_from_quaternion(sensor_rot)
        sensor_range = self.scan.range_max  # [m]
        ang_min = self.scan.angle_min + sensor_yaw
        ang_max = self.scan.angle_max + sensor_yaw
        ang_step = self.scan.angle_increment
        
        cur_x_pix = int(sensor_pos[0] / self.resolution)
        cur_y_pix = int(sensor_pos[1] / self.resolution)
        self.scan.ranges = []
        self.scan.intensities = []
        # lidarを取得する角度列を作成
        angles = np.arange(ang_min, ang_max, ang_step)
        xmaxs = (cur_x_pix + (self.scan.range_max * np.cos(angles)) / self.resolution).astype(int)
        ymaxs = (cur_y_pix + (self.scan.range_max * np.sin(angles)) / self.resolution).astype(int)
        xmins = (cur_x_pix + (self.scan.range_min * np.cos(angles)) / self.resolution).astype(int)
        ymins = (cur_y_pix + (self.scan.range_min * np.sin(angles)) / self.resolution).astype(int)
        
        # time1 = time.perf_counter()
        for i in range(angles.size):
            # lidarの走査直線上のピクセル座標を取得
            points = LineIterator((xmins[i], ymins[i]), (xmaxs[i], ymaxs[i]))
            points = points + (self.origin_x_pix, self.origin_y_pix)

            # 座標が画像の範囲を超えないようにする
            points = np.where(points < 0, 0, points)
            points[:, 0] = np.where(points[:, 0] >= self.map.shape[1], self.map.shape[1] - 1, points[:, 0])
            points[:, 1] = np.where(points[:, 1] >= self.map.shape[0], self.map.shape[0] - 1, points[:, 1])

            # lidarの値を入れる
            pixval = self.map[points[:, 1], points[:, 0]]
            idxs = np.where(pixval > 10)[0]
            if(idxs.size == 0):
                self.scan.ranges.append(sensor_range + 1)
            else:
                p = points[idxs[0]]
                dist = math.hypot((p[0] - self.origin_x_pix) - cur_x_pix, (p[1] - self.origin_y_pix) - cur_y_pix) * self.resolution
                self.scan.ranges.append(dist)

        cur_time = rospy.get_time()
        self.scan.time_increment = 0  # 各角度ごとのスキャン時間
        self.scan.scan_time = cur_time - self.prev_time  # 全角度分のスキャン時間
        self.scan.header.stamp = rospy.get_rostime()
        self.scan_pub.publish(self.scan)
        self.prev_time = cur_time
        lock.release()


if __name__ == '__main__':
    rospy.init_node('dummy_scan2', anonymous=True)

    scan_range_min = rospy.get_param("~scan_range_min", 0.01)  # [m]
    scan_range_max = rospy.get_param("~scan_range_max", 10)  # [m]
    scan_angle_min = rospy.get_param("~scan_angle_min", -math.pi / 2)
    scan_angle_max = rospy.get_param("~scan_angle_max", math.pi / 2)
    scan_resolution = rospy.get_param("~scan_resolution", 0.01)
    
    map_topic = rospy.get_param("~map_topic", "scan_map")
    sensor_frame = rospy.get_param("~sensor_frame", "wheels_base_laser_link")
    publish_rate = rospy.get_param("~publish_rate", 20)
    scan_topic = rospy.get_param("~scan_topic", "scan")
    scan_base_frame = rospy.get_param("~scan_base_frame", "odom")
    
    print("publish rate: ", publish_rate)
    dummy_scan = DummyScan(map_topic, scan_topic, sensor_frame, scan_base_frame, scan_range_min, scan_range_max, scan_angle_min, scan_angle_max, scan_resolution)
    
    rate = rospy.Rate(publish_rate)
    while not rospy.is_shutdown():
        dummy_scan.make_scan_data()
        rate.sleep()
