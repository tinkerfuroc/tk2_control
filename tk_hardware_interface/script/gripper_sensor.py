#!/usr/bin/python
import rospy
from std_msgs.msg import Bool


def main():
    rospy.init_node('gripper_laser_sensor', anonymous=False)
    publisher = rospy.Publisher('gripper_laser_sensor', Bool)
    sensor_pin = rospy.get_param('~gripper_laser_pin', 58)
    rate = rospy.Rate(100)
    try:
        gpiopin = "gpio%s" % (str(sensor_pin),)
        pin = open("/sys/class/gpio/" + gpiopin + "/value", "r")
        rospy.loginfo('Starting gripper laser sensor ...')
    except:
        rospy.logerr('Cannot open sensor pin')
        return

    old_value = int(pin.read().strip())
    publisher.publish(old_value)
    while not rospy.is_shutdown():
        pin.seek(0)
        value = int(pin.read().strip())
        if value != old_value:
            publisher.publish(value)
            rospy.loginfo('gripper laser sensor state has changed to %d', value)
        old_value = value
        rate.sleep()

    pin.close()
    rospy.spin()


if __name__ == '__main__':
    main()
