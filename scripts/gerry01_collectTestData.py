import gerry00_armcontroller
from gerry00_armcontroller import init, relax_all, ik, go_to_q
import rospy
import numpy as np
import math

index = 0

def main():
    global setpoint, index
    rate = rospy.Rate(1) # 10hz

    focus_y = 0.25
    allTXY = []
    is_up = True
    for theta in np.arange(-math.pi*5/6, math.pi*5/6, math.pi/12):
        for j in (range(6) if is_up else range(5,-1,-1)):
            x = 0.075
            y = focus_y - 0.10 + 0.10 * (j / 5.0)
            # r = 0.05
            # x = r * math.cos(index / 10.0 - math.pi/2)
            # y = focus_y - r * math.sin(index / 10.0 - math.pi/2)
            allTXY.append([theta, x, y])
        is_up = not is_up

    while not rospy.is_shutdown() and index < len(allTXY):
        txy = allTXY[index]
        setpoint = ik(txy[0], txy[1], txy[2], focus_y=focus_y)
        hello_str = "going to position %f %f %f %f" % tuple(setpoint)
        rospy.loginfo(hello_str)
        go_to_q(setpoint)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('arm_controller_test')
    init()
    main()
    relax_all()