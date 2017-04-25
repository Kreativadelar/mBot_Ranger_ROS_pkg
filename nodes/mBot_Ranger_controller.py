#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy
import time
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from meauriga import *
from mBot_Ranger_ROS_pkg.srv import *

bot = None
lastTime = time.time()
useTankSteering = True;
online = True;
blueLight = False;
m1 = 0;
m2 = 0;

# Defining functions
# -------------------------

# Callback for readings from ultrasonic sensor 
def onUltrasonicSensorRead(v):
    #print("and here!")
    #rospy.loginfo('Distance: ' + str(v))
    pub.publish(v)


# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
def joyCallback(data):
    global blueLight
    #twist = Twist()
    #twist.linear.x = 4*data.axes[1]
    #twist.angular.z = 4*data.axes[0]
    #pub.publish(twist)
    #if(useTankSteering)
    m2 = (numpy.interp(data.axes[1],[-1,1],[-255,255]))
    m1 = (numpy.interp(data.axes[4],[-1,1],[-255,255]) * -1)
    
    bot_control_motors(m1, m2)
    
    #Set led ring to blue 
    if data.buttons[2] == 1:
        blueLight = True
    else:
        blueLight = False

    #rospy.loginfo(m1)
    #rospy.loginfo(m2)
    #rospy.loginfo(data)


# Service method add velocity to motors 
def handle_makeblock_motors(req):
    rospy.loginfo(req)
    bot_control_motors(req.s1, req.s2)
    return 1


def bot_control_motors(_m1, _m2):
    global bot
    global online
    global lastTime
    global m1
    global m2
    if online:
        m1 = _m1
        m2 = _m2
        #bot.encoderMotorRun(1, _m1)
        #bot.encoderMotorRun(2, _m2)


# Defining variables
# ----------------------
pub = rospy.Publisher('mBot_Ranger_controller_ultrasensor', Float32, queue_size=1)
s = rospy.Service('mBot_Ranger_controller_move_motors', MakeBlockMover,
                  handle_makeblock_motors)


def main():
    global bot
    global online
    global m1
    global m2
    if online:
        bot = MeAuriga()
        bot.start("/dev/ttyUSB0")

    # Subscribe for teleoperations
    rospy.Subscriber("joy", Joy, joyCallback)

    rospy.init_node('mBot_Ranger_controller', anonymous=False)

    rate = rospy.Rate(10)  # 10hz
    count = 0
    while not rospy.is_shutdown():
        sleep(0.15)
        #print("been here!")
        if online:
            bot.ultrasonicSensorRead(3, onUltrasonicSensorRead)
            # Motor
            if count >= 0:
                bot.encoderMotorRun(1, m1)
                bot.encoderMotorRun(2, m2)
                count = 0;
            #RGB Led ring
            for i in range(16):
                if blueLight:
                    red = 0;
                    green = 0;
                    blue = 255;
                else:
                    red = 0;
                    green = 0;
                    blue = 0;
                bot.rgbledDisplay(0,1,i,red,green,blue);
            count = count +1;
            


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
