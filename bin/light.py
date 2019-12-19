#!/usr/bin/python
import rospy
import math
import copy
import sys
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def deltadistance(a, b):
    ap = a.pose.pose.position
    bp = b.pose.pose.position
    return math.sqrt((bp.x - ap.x) ** 2 + (bp.y - ap.y) ** 2)

def dumpPoints(poseA, poseB):
    pa = poseA.pose.pose.position
    pb = poseB.pose.pose.position
    print("A({}): x:{} y:{} z:{}".format(id(poseA), pa.x, pa.y, pa.z))
    print("B({}): x:{} y:{} z:{}".format(id(poseB), pb.x, pb.y, pb.z))

# GLOBAL OBJECTS
sideLength = 0.03
anglePoint = None
updateAnglePoint = False
count = 0
# /GLOBAL OBJECTS

pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

def calcSquareCallback(msg):
    global anglePoint
    global sideLength
    global updateAnglePoint
    global count

    print("++++++++++++++++++++++++++++++++++CALLBACK++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    o = msg.pose.pose.orientation
    p = msg.pose.pose.position
    (roll, pitch, yaw) = euler_from_quaternion([o.x, o.y, o.z, o.w])
    print("x:{} y:{} z:{} angle:{}".format(p.x, p.y, p.z, yaw))

    if anglePoint is None:
        print("update angle point!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        anglePoint = msg
    else:
        print("dumping points before")
        dumpPoints(msg, anglePoint)

        # cmd_vel command for stopping robot
        stop = Twist()

        if updateAnglePoint:
            count = count + 1
            anglePoint = msg
            updateAnglePoint = False

        if count >= 4:
            print("Done")
            pub.publish(stop)
            sys.exit()

        print("dumping points after")
        dumpPoints(msg, anglePoint)

        # sleep for one second
        sleep = rospy.Rate(1)
        # sleep for 5 sec
        sleep5 = rospy.Rate(0.2)

        ddist = deltadistance(msg, anglePoint)
        
        print("delta distance:{}".format(ddist))
        # if we're at the vertex of square
        if(ddist >= sideLength):
            # update angle point
            updateAnglePoint = True
            p = msg.pose.pose.position
            print("coords x:{} y:{} z:{}".format(p.x, p.y, p.z))
            print("------------------------------")

            # turn by 90 deg
            #ddist = deltadistance(msg, anglePoint)
            print("\nrotate by 90 deg, ddist:{}".format(ddist))
            pub.publish(stop)
            rotate = Twist()

            rotate.angular.z = math.pi/10
            pub.publish(rotate)
            sleep5.sleep()
            pub.publish(stop)
            # otherwise move towards the point
        else:
            print("make a step")
            #pub.publish(stop)
            step = Twist()
            step.linear.x = 0.07
            print("making a step: {}".format(step.linear.x))
            pub.publish(step)
            sleep.sleep()
            #pub.publish(stop)

    print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

rospy.init_node("square27_2")

sub = rospy.Subscriber("/odom", Odometry, calcSquareCallback)

rospy.spin()
