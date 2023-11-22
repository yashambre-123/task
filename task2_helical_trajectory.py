#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

def helical_trajectory(radius, height, loops, velocity, local_pos_pub, path_pub):
    count = 0
    rate = rospy.Rate(10.0)  # 20 Hz
    MARKERS_MAX = 123
    message_to_be_published = MarkerArray()
    # point_message = Point()
    for t in range(int(20 * math.pi * loops)):
        print(count)
        # count+=1
        x = radius * math.cos(t * velocity / radius)
        y = radius * math.sin(t * velocity / radius)
        z = height * t / (20 * math.pi * loops)
        setpoint_msg = PoseStamped()
        print([x,y,z])
        setpoint_msg.pose.position.x = x
        setpoint_msg.pose.position.y = y
        setpoint_msg.pose.position.z = 30 + z
        local_pos_pub.publish(setpoint_msg)

        message = Marker()
        message.header.frame_id = "base_link"
        message.type = message.SPHERE_LIST
        message.action = message.ADD

        message.scale.x = 3.0
        message.scale.y = 3.0
        message.scale.z = 3.0

        message.color.r = 1.0
        message.color.g = 0.0
        message.color.b = 0.0
        message.color.a = 1.0

        message.pose.orientation.x = 0.0
        message.pose.orientation.y = 0.0
        message.pose.orientation.z = 0.0
        message.pose.orientation.w = 1.0

        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.position.z = 30 + z

        message.points.append(setpoint_msg.pose.position)

        if (count > MARKERS_MAX):
            message_to_be_published.markers.pop(0)

        message_to_be_published.markers.append(message)

        id = 0
        for m in message_to_be_published.markers:
            m.id = id
            id = id + 1

        path_pub.publish(message_to_be_published)

        count = count + 1

        rate.sleep()

# def local_pos_callback(data):
#     print(data.pose.position)

def main():
    rospy.init_node('helical_trajectory_node', anonymous=True)
    local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    path_pub = rospy.Publisher('/visualization_marker_array', MarkerArray)


    radius = 5  # meters
    height = 10  # meters
    loops = 2
    velocity = 1  # m/s

    try:
        # Fly in helical trajectory
        helical_trajectory(radius, height, loops, velocity, local_pos_pub, path_pub)
        # Return in reverse helical trajectory
        helical_trajectory(radius, -height, loops, -velocity, local_pos_pub, path_pub)
    except rospy.ROSInterruptException:
        pass

if __name__=="__main__":
    main()