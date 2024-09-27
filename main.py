import os
from time import sleep
os.environ['USING_ROBOT_TEST'] = 'True'

from robot.denso_robot import DensoRobot
from robot.denso_test import DensoTest
from rria_api_denso import CAOVariableType
DENSO_IP = "Server=192.168.160.228"
CLICKS = 5


def main(denso_ip, robot_type='test'):
    if robot_type == 'test':
        os.environ['USING_ROBOT_TEST'] = 'True'
        robot = DensoTest()
    else:
        robot = DensoRobot('test_pen_holder', 'test_pen_holder', denso_ip)

    robot.connect()
    robot.set_arm_speed(50, 50, 50)
    robot.motor_on()
    robot.move_by_variable(CAOVariableType.J, 9)  # home
    robot.move_by_variable(CAOVariableType.J, 15)
    successful_feedback = 0
    for _ in range(CLICKS):
        robot.move_by_variable(CAOVariableType.J, 15)  # pre
        robot.move_by_variable(CAOVariableType.J, 16)  # tap
        feedback = robot.get_finger_feedback()
        successful_feedback += feedback
    robot.move_by_variable(CAOVariableType.J, 9)  # home
    robot.motor_off()
    robot.disconnect()
    print(f'EXPECTED = {CLICKS}: ACTUAL = {successful_feedback}')


if __name__ == '__main__':
    main(DENSO_IP, robot_type='real')
