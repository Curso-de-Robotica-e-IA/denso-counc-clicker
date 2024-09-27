import os
os.environ['USING_ROBOT_TEST'] = 'True'

from robot.denso_robot import DensoRobot
from robot.denso_test import DensoTest
DENSO_IP = "Server=192.168.160.228"
CLICKS = 5


def main(denso_ip, robot_type='test'):
    if robot_type == 'test':
        os.environ['USING_ROBOT_TEST'] = 'True'
        robot = DensoTest()
    else:
        robot = DensoRobot('test_pen_holder', 'test_pen_holder', denso_ip)

    robot.connect()
    robot.motor_on()
    robot.move_preset_joints('home')
    robot.move_preset_joints('pre_tap')
    for i in range(CLICKS):
        robot.move_preset_cartesian('tap')
        robot.move_preset_cartesian('pre_tap')
    robot.move_preset_joints('home')
    robot.disconnect()


if __name__ == '__main__':
    main(DENSO_IP, robot_type='test')
