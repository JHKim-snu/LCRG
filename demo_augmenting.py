import sys
import rospy
import socket
import cv2
import numpy as np
import base64

import time
from agent import Agent
from util import random_orientation_sample

K = 2

def main():

    try:
        idx = int(sys.argv[1])
    except Exception:
        idx = 0
    rospy.init_node('visual_grounding_client', anonymous=False, disable_signals=True)
    agent = Agent()
    while not rospy.is_shutdown():
        try:
            rospy.loginfo('Visual Grounding Trial {}'.format(idx+1))
            agent.arm.to_ready_pose()
            rospy.sleep(2.0)
            raw_input('Place Things on gripper and press any key')
            print('Wait 10 secs')
            rospy.sleep(10.)
            pose = agent.arm.get_pose()
            current_ori = [pose.orientation.x,
                           pose.orientation.y,
                           pose.orientation.z,
                           pose.orientation.w]
            sampled_ori_array = random_orientation_sample(current_ori, K, scale=8e-1)
            for i, ori in enumerate(sampled_ori_array):
                agent.arm.eef_orientation(*ori)
                cv_img = agent.camera.get_rgb()
                # Send Captured Image
                cv2.imwrite('demo_{}_{}.png'.format(idx, i), cv_img)
                rospy.loginfo('Iter {}'.format(i))
            idx += 1
            raw_input('Press any key to release object')
            agent.arm.finger_open()
            value = raw_input('Continue? [y/n]')
            if value != 'y': break
        except IOError and ValueError as e:
            print(e)
            break
    return

if __name__ == '__main__':

    main()






