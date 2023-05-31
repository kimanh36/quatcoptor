#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

current_state = State()
def state_cb(state):
    global current_state
    current_state = state


if __name__ == "__main__":
    rospy.init_node('offb_node')
    rate = rospy.Rate(20.0)

    #Subscribe to MAVROS state
    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    # Wait for FCU connection
    while not rospy.is_shutdown() and not current_state.connected:
        rate.sleep()

    pose = PoseStamped()
    pose.pose.position.x = 1.8
    pose.pose.position.y = 2.0
    pose.pose.position.z = 3.5

    # Send a few setpoints before starting offboard mode
    for i in range(100):
        local_pos_pub.publish(pose)
        rate.sleep()

    # Request offboard and arm
    offb_set_mode = SetMode()
    offb_set_mode.custom_mode = "OFFBOARD"

    armCmd = CommandBool()
    armCmd.value = True

    last_request = rospy.get_rostime()

    while not rospy.is_shutdown():
        if not current_state.armed and (rospy.get_rostime() - last_request > rospy.Duration(5.)):
            armCmd.response = arming_client(True)
            if armCmd.response.success:
                rospy.loginfo("Armed")
            last_request = rospy.get_rostime()
        elif current_state.mode != "OFFBOARD":
            offb_set_mode_response = set_mode_client(custom_mode="OFFBOARD")
            if offb_set_mode_response.mode_sent:
                rospy.loginfo("Offboard enabled")
            last_request = rospy.get_rostime()

        local_pos_pub.publish(pose)

        rate.sleep()

