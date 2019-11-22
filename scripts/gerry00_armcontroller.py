from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from arbotix_msgs.srv import Relax
import rospy
import math
import actionlib
import numpy as np
import rpi_ssh_controller

joint_names = ['base_rot_joint',
               'arm_shoulder_joint',
               'arm_elbow_joint',
               'arm_wrist_joint']
#               height  link1  link2  cam-mount
link_lengths = [0.0803, 0.107, 0.107, 0.0265]

publishers = []
relaxers = []

index = 0
setpoint = [0]*len(joint_names)

ssh = 0

def init():
    global publishers, relaxers, ssh
    ssh = rpi_ssh_controller.init()
    for name in joint_names:
        publishers.append(
            rospy.Publisher(name+'/command', Float64, queue_size=5)
        )
        relaxers.append(
            rospy.ServiceProxy(name+'/relax', Relax)
        )
    if not relaxers:
        raise Exception("relax services couldn't be read")
    rospy.Subscriber('joint_states', JointState, callback=check_joint_state)
    rospy.on_shutdown(relax_all)

def check_joint_state(data):
    if not setpoint:
        return
    curpoint = data.position
    for i in range(len(curpoint)):
        try:
            curpoint_index = data.name.index(joint_names[i])
        except ValueError:
            print("%s %s" % (data.name, joint_names[i]))
            return
        if abs(curpoint[curpoint_index] - setpoint[i]) > 0.05:
            print('%s has error %f' %
                (joint_names[i], abs(curpoint[curpoint_index] - setpoint[i])))
            return
    global index, setpoint
    rpi_ssh_controller.takeImage(ssh, 'Desktop/data2/test_%d.jpg'%index)
    index = index + 1
    setpoint = []

def ik(theta, x, y, focus_y = .25):
    '''Calculate inverse kinematics for joint angles given target location
    Arguments:
        theta: base rotation
        x: x location of camera (relative to base)
        y: y location of camera (relative to base)
        focus_y: height above base the camera is focused
    Returns:
        angles: 4-list of joint angles, base first wrist last
    '''
    q = [theta,0,0,0]
    if (x<0):
        q[0] = q[0] + math.pi
        x = -x
    angle_to_focus = math.atan2(focus_y - y, -x)
    x_lastj = x - math.cos(angle_to_focus)*link_lengths[3]
    y_lastj = y - math.sin(angle_to_focus)*link_lengths[3] - link_lengths[0]
    
    c = math.sqrt(x_lastj**2 + y_lastj**2)
    a = link_lengths[1]
    b = link_lengths[2]
    q[2] = math.acos((a**2 + b**2 - c**2) / (2*a*b)) # law of cosines
    first_angle = math.asin( math.sin(q[2]) / c * b ) # law of sines
    q[1] = math.atan(abs(y_lastj) / abs(x_lastj)) - first_angle
    q[3] = angle_to_focus + (q[2]-q[1])
    # fix angle reference frames
    q[1] = q[1] - math.pi/2
    q[2] = math.pi - q[2]
    q[3] = q[3] - math.pi
    return q

def go_to_q(q):
    for i, angle in enumerate(q):
        publishers[i].publish(angle)

def relax_all():
    global relaxers
    rospy.loginfo('shutdown routine')
    rpi_ssh_controller.exit(ssh)
    rospy.loginfo('closed ssh')
    for relaxer in relaxers:
        relaxer.call()
    rospy.loginfo('relaxed all servos')

def main():
    global setpoint
    rate = rospy.Rate(1) # 10hz

    focus_y = 0.26
    allTXY = []
    is_up = True
    for theta in np.arange(-math.pi*5/6, math.pi*5/6, math.pi/12):
        for j in (range(6) if is_up else range(5,-1,-1)):
            y = focus_y - 0.10 + 0.07 * (j / 5.0)
            fo_y = focus_y - 0.07 + 0.07 * (j / 5.0) + 0.025
            # x = 0.14 - 0.03*(j/5.0)
            x = 0.125
            # r = 0.05
            # x = r * math.cos(index / 10.0 - math.pi/2)
            # y = focus_y - r * math.sin(index / 10.0 - math.pi/2)
            allTXY.append([theta, x, y, fo_y])
        is_up = not is_up

    while not rospy.is_shutdown() and index < len(allTXY):
        txy = allTXY[index]
        setpoint = ik(txy[0], txy[1], txy[2], focus_y=txy[3])
        hello_str = "going to position %f %f %f %f" % tuple(setpoint)
        rospy.loginfo(hello_str)
        go_to_q(setpoint)
        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('arm_controller_test')
    init()
    main()
    relax_all()