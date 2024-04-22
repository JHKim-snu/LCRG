import sys
import rospy
import socket
import cv2
import numpy as np
import base64

import time
from agent import Agent
from util import random_orientation_sample

HOST = '147.47.200.155'
PORT = 9998
K = 2

def main():

    idx = int(sys.argv[1])
    rospy.init_node('visual_grounding_client', anonymous=False, disable_signals=True)
    cli_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    cli_sock.connect((HOST, PORT))
    agent = Agent()
    while not rospy.is_shutdown():
        try:
            rospy.loginfo('Visual Grounding Trial {}'.format(idx+1))
            #rospy.loginfo('To Camera Pose')
            #agent.arm.to_camera_pose()
            agent.arm.to_ready_pose()
            rospy.sleep(2.0)
            raw_input('Place Things and Get Visual Info (Press any key)')
            # Take visual info and send image to the server
            cv_img = agent.camera.get_rgb()
            cloud = agent.camera.get_cloud()
            # Send Captured Image
            retval, buf = cv2.imencode('.png', cv_img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT])
            data = np.array(buf)
            b64_data = base64.b64encode(data)
            length = str(len(b64_data))
            cli_sock.sendall(length.encode('utf-8').ljust(64))
            cli_sock.sendall(b64_data)
            rospy.loginfo('Wait for the server')
            # Receive bounding box Info
            data = cli_sock.recv(1024)
            str_data = data.decode()
            bbox = str_data.strip().split(';')
            print(bbox)
            TL_X = int(float(bbox[0]))
            TL_Y = int(float(bbox[1]))
            BR_X = int(float(bbox[2]))
            BR_Y = int(float(bbox[3]))
            print(TL_X, TL_Y, BR_X, BR_Y)
            if TL_X * TL_Y * BR_X * BR_Y < 0:
                rospy.logwarn('Negative BBOX')
                continue 
            # Visualize received bbox on input image
            cv_img = cv2.rectangle(cv_img, (TL_X, TL_Y), (BR_X, BR_Y), (0, 0, 255))
            cv2.imwrite('grasp_result/demo_ori_{}.png'.format(idx), cv_img)
            rospy.loginfo('Pick bbox: {} {} {} {}'.format(TL_X, TL_Y, BR_X, BR_Y))
            agent.arm.to_ready_pose()
            agent.grasp(TL_X, TL_Y, BR_X, BR_Y, cloud)
            # Manual Input to Continue or Not (whether it succeeded to grasp)
            val = raw_input('Input \'y\' to continue. Otherwise it retry: ')
            if val == 'y':
                pass
            else:
                continue
            # augementation with random pose
            pose = agent.arm.get_pose()
            current_ori = [pose.orientation.x,
                           pose.orientation.y,
                           pose.orientation.z,
                           pose.orientation.w]
            sampled_ori_array = random_orientation_sample(current_ori, K, scale=8e-1)
            for ori in sampled_ori_array:
                agent.arm.eef_orientation(*ori)
                cv_img = agent.camera.get_rgb()
                # Send Captured Image
                retval, buf = cv2.imencode('.png', cv_img, [cv2.IMWRITE_PNG_STRATEGY_DEFAULT])
                data = np.array(buf)
                b64_data = base64.b64encode(data)
                length = str(len(b64_data))
                cli_sock.sendall(length.encode('utf-8').ljust(64))
                cli_sock.sendall(b64_data)
                rospy.loginfo('Loop sent') 
                recv_data = cli_sock.recv(1024)
                print(recv_data.decode())
            idx += 1
            raw_input('Press any key to release object')
            agent.arm.finger_open()
            value = raw_input('Continue? [y/n]')
            if value != 'y': break
        except KeyboardInterrupt:
            print('\nClient Ctrl-c')
            break
        except IOError and ValueError as e:
            print(e)
            break

    cli_sock.close() 
    return

if __name__ == '__main__':

    main()






