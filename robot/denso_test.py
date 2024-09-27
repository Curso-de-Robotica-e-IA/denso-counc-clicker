from time import sleep

# Import commands for the gripper and the robot
from rria_api_denso import (RobotCartesianCommand,
                            RobotJointCommand)
# Import abstract robot class
from robot.denso_abstract import AbstractDenso
from robot.bank_movements import get_pose


# Implement test Denso robot from abstract robot
class DensoTest(AbstractDenso):
    def __init__(self):
        self.__motor_enabled = False
        self.__connected = False
        self.__gripper_is_connect = False

    def is_connected(self) -> bool:
        sleep(1)
        return self.__connected

    def motor_enabled(self) -> bool:
        sleep(1)
        return self.__motor_enabled

    def connect(self) -> bool:
        sleep(1)
        if not self.__connected:
            self.__connected = True
            print('\nRobot successfully connected')
        else:
            print('\nRobot already connected')
        return True

    def disconnect(self) -> bool:
        sleep(2)
        if self.__connected:
            self.__connected = False
            print('\nRobot successfully disconnected')
        else:
            print('\nRobot already disconnected')
        return True

    def motor_on(self) -> bool:
        sleep(2)
        if self.__connected:
            if not self.__motor_enabled:
                self.__motor_enabled = True
                print('\nRobot motor was successfully turned on')
            else:
                print('\nRobot motor is already on')
            return True
        else:
            print('\nRobot not connected')
            return False

    def motor_off(self) -> bool:
        sleep(2)
        if self.__connected:
            if self.__motor_enabled:
                self.__motor_enabled = False
                print('\nRobot motor was successfully turned off')
            else:
                print('\nRobot motor is already off')
            return True
        else:
            print('\nRobot not connected')
            return False

    def move_joints(self, command) -> bool:
        sleep(2)
        if self.__connected and self.__motor_enabled:
            print(
                "moving joints to:",
                [command.joint_1, command.joint_2, command.joint_3, command.joint_4, command.joint_5, command.joint_6],
            )
            return True
        elif not self.__motor_enabled:
            print('\nRobot motor is off')
            return False
        elif not self.__connected:
            print('\nRobot not connected')
            return False

    def move_cartesian(self, command) -> bool:
        sleep(2)
        if self.__connected and self.__motor_enabled:
            print(
                "moving cartesian to:",
                [command.x, command.y, command.z, command.rx, command.ry, command.rz, command.fig]
            )
            return True
        elif not self.__motor_enabled:
            print('\nRobot motor is off')
            return False
        elif not self.__connected:
            print('\nRobot not connected')
            return False

    def move_preset_joints(self, pose: str) -> bool:
        if self.__connected and self.__motor_enabled:
            _, msg = get_pose(pose, 0)
            sleep(1)
            print('\n' + msg + 'by joints move')
            return True
        elif not self.motor_enabled():
            print('\nRobot motor is off')
            return False
        elif not self.is_connected():
            print('\nRobot not connected')
            return False

    def move_preset_cartesian(self, pose: str) -> bool:
        if self.__connected and self.__motor_enabled:
            _, msg = get_pose(pose, 1)
            sleep(1)
            print('\n' + msg + 'by cartesian move')
        elif not self.motor_enabled():
            print('\nRobot motor is off')
            return False
        elif not self.is_connected():
            print('\nRobot not connected')
            return False

    def get_joints_pose(self):
        sleep(1)
        if self.__connected:
            return RobotJointCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            print('\nRobot not connected')
            return None

    def get_cartesian_pose(self):
        sleep(1)
        if self.__connected:
            return RobotCartesianCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
        else:
            print('\nRobot not connected')
            return None

    def set_arm_speed(self, speed, accel, decel) -> bool:
        sleep(1)
        if self.__connected:
            print("set arm speed to:", [speed, accel, decel])
            return True
        else:
            print('\nRobot not connected')
            return False

    def get_current_arm_speed(self) -> tuple | None:
        sleep(1)
        if self.__connected:
            return (10, 10, 10)
        else:
            print('\nRobot not connected')
            return None

    def get_current_error(self):
        if self.__connected:
            return (1, "", "")
        else:
            print('\nRobot not connected')
            return None

    def get_current_fig(self) -> int | None:
        if self.__connected:
            return 1
        else:
            print('\nRobot not connected')
            return None

    def is_out_of_range(self, robot_command) -> bool | None:
        sleep(1)
        if not self.__connected:
            print('\nRobot not connected')
        return False

    def joint_to_cartesian(self, command):
        if self.__connected:
            return RobotCartesianCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
        else:
            print('\nRobot not connected')
            return None

    def cartesian_to_joint(self, command):
        if self.__connected:
            return RobotJointCommand(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        else:
            print('\nRobot not connected')
            return None

    def move_tool_z(self, z_step: float) -> bool:
        sleep(2)
        if self.__connected and self.__motor_enabled:
            print(f"move z: {z_step}")
            return True
        elif not self.__connected:
            print('\nRobot not connected')
            return False
        elif not self.__motor_enabled:
            print('\nRobot motor is off')
            return False

    def calculate_dist_to(self, position):
        if self.__connected:
            return 1
        else:
            print('\nRobot not connected')
            return None
