#!/usr/bin/env python
 
import setup_path 
import airsim
import rosbag
import rospy
import sensor_msgs.msg
import std_msgs.msg
import sensor_msgs.point_cloud2 as point_cloud2
from sensor_msgs.msg import CompressedImage, Imu, NavSatFix, NavSatStatus
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import sys
import math
import time
import argparse
import pprint
import numpy
import os
import errno


class LidarTest:

    def __init__(self, bagfile, lidar_out, frames):

        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        
        # not enabled so user can drive with RC
        #self.client.enableApiControl(True)

        self.bag = rosbag.Bag(bagfile, 'w')
        self.lidar_out = lidar_out
        self.frames = frames 

        if (lidar_out != ''):
            try:
                os.makedirs(lidar_out)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise

    def execute(self):
        airsim.wait_key('Press any key to start capture')

        #self.client.armDisarm(True)

        #self.client.takeoffAsync().join()

        #z = -5
        #print("hovering at 5 meters...")
        #self.client.moveToZAsync(z, 1).join()
        
        #print("flying on path...")
        #result = self.client.moveOnPathAsync([airsim.Vector3r(125,0,z),
        #                        airsim.Vector3r(125,-130,z),
        #                        airsim.Vector3r(0,-130,z),
        #                        airsim.Vector3r(0,0,z)],
        #                3, 120,
        #                airsim.DrivetrainType.ForwardOnly, airsim.YawMode(False,0), 20, 1).join()
        i = 0
        print("capture start")
        while(i < self.frames):
            
            self.capture_frame(i)
            i += 1
    
    def capture_frame(self, i):
        print("capturing frame " + str(i))

        self.capture_tf()
        self.capture_lidar(i)
        self.capture_image()
        self.capture_imu()
        self.capture_gps()

    def capture_image(self):
        responses = self.client.simGetImages([
            airsim.ImageRequest(0, airsim.ImageType.Scene)
        ])
        self.write_image_to_bag(responses[0].image_data_uint8, responses[0].time_stamp, responses[0].height, responses[0].width)


    def capture_tf(self):
        state = self.client.getMultirotorState()

        tf_msg = TFMessage()

        t = TransformStamped()

        t.header.frame_id = "drone"
        t.child_frame_id = "world"
        t.header.stamp = rospy.Time.from_sec(float(state.timestamp)/1000000000)

        t.transform.translation.x = state.kinematics_estimated.position.x_val
        t.transform.translation.y = state.kinematics_estimated.position.y_val
        t.transform.translation.z = state.kinematics_estimated.position.z_val

        t.transform.rotation.x = state.kinematics_estimated.orientation.x_val
        t.transform.rotation.y = state.kinematics_estimated.orientation.y_val
        t.transform.rotation.z = state.kinematics_estimated.orientation.z_val
        t.transform.rotation.w = state.kinematics_estimated.orientation.w_val

        tf_msg.transforms.append(t)

        self.bag.write("/tf", tf_msg, t.header.stamp)

    def capture_gps(self):
        gps_data = self.client.getGpsData()
        
        print("gps_data: " + str(gps_data.gnss.geo_point.latitude) + ", " + str(gps_data.gnss.geo_point.longitude) + ", " + str(gps_data.gnss.geo_point.altitude))

        gps_msg = NavSatFix()
        status = NavSatStatus()
        status.status = NavSatStatus.STATUS_FIX
        status.service = NavSatStatus.SERVICE_GPS
        gps_msg.status = status
         
        gps_msg.altitude = gps_data.gnss.geo_point.altitude
        gps_msg.latitude = gps_data.gnss.geo_point.latitude
        gps_msg.longitude = gps_data.gnss.geo_point.longitude
        gps_msg.header.frame_id = "drone"
        gps_msg.header.stamp = rospy.Time.from_sec(float(gps_data.time_stamp)/1000000000)
        self.bag.write("/fix", gps_msg, gps_msg.header.stamp)


    def capture_imu(self):
        imu_data = self.client.getImuData()
        
        imu_msg = Imu()
        imu_msg.header.frame_id = "drone"
        imu_msg.header.stamp = rospy.Time.from_sec(float(imu_data.time_stamp)/1000000000)
        imu_msg.orientation.x = imu_data.orientation.x_val
        imu_msg.orientation.y = imu_data.orientation.y_val
        imu_msg.orientation.z = imu_data.orientation.z_val
        imu_msg.orientation.w = imu_data.orientation.w_val
        imu_msg.linear_acceleration.x = imu_data.linear_acceleration.x_val
        imu_msg.linear_acceleration.y = imu_data.linear_acceleration.y_val
        imu_msg.linear_acceleration.z = imu_data.linear_acceleration.z_val
        imu_msg.angular_velocity.x = imu_data.angular_velocity.x_val
        imu_msg.angular_velocity.y = imu_data.angular_velocity.y_val
        imu_msg.angular_velocity.z = imu_data.angular_velocity.z_val
        self.bag.write("/imu/data", imu_msg, imu_msg.header.stamp)

    def capture_lidar(self, frame):
        self.client.hoverAsync().join()

        for i in range(1,2):
            lidarData = self.client.getLidarData()
            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")
            else:
                points = self.parse_lidarData(lidarData)
                print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                #print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                #print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))
                if (self.lidar_out != ''):
                    numpy.savetxt(self.lidar_out + '/' + str(frame) + '.xyz', points, fmt='%10.4f', delimiter=' ')
            time.sleep(.2)

    def parse_lidarData(self, data):
       # print("\t time_stamp: %d" % (data.time_stamp))
                
        # reshape array of floats to array of [X,Y,Z]
        
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))

        # change direction of Z      
        for i in range(points.size / 3):
            points[i, 2] = -1 * points[i, 2]
       
        self.write_lidarData_to_bag(points, data.time_stamp)
        
        return points

    def write_lidarData_to_bag(self, points, timestamp):
        header = std_msgs.msg.Header()
        header.frame_id = "drone"
        header.stamp = rospy.Time.from_sec(float(timestamp)/1000000000)
        pc2 = point_cloud2.create_cloud_xyz32(header, points)
        self.bag.write('/rslidar_points', pc2, header.stamp)

    def write_image_to_bag(self, image_data, timestamp, h, w):
        image = CompressedImage()
        image.header.stamp = rospy.Time.from_sec(float(timestamp)/1000000000)
        image.format = "png"
        image.header.frame_id = "drone"
        image.data = image_data
        self.bag.write('/camera', image, image.header.stamp)

    def stop(self):
        self.bag.close()
        airsim.wait_key('Press any key to reset to original state')

        #self.client.armDisarm(False)
        self.client.reset()

        #self.client.enableApiControl(False)
        print("Done!\n")

# main
if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Scan1 test")

    arg_parser.add_argument('-bag', type=str, default="output.bag")

    arg_parser.add_argument('-f', type=int, default=50)

    arg_parser.add_argument('-lidar', type=str, default="")

    args = arg_parser.parse_args(args)    

    lidarTest = LidarTest(args.bag, args.lidar, args.f)
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()
