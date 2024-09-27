# Import control over the gripper
from rria_api_denso import (DensoRobotAPI, RobotJointCommand,
                            RobotCartesianCommand)
from rria_api_denso.denso_robot_api import CAOVariableType
# Import abstract robot class
from robot.denso_abstract import AbstractDenso
from robot.bank_movements import get_pose

IO_PORT = 48


# Implement real Denso robot from abstract robot
class DensoRobot(AbstractDenso):
    def __init__(self, workspace_name, control_name, options):
        self.__denso_api = DensoRobotAPI(workspace_name, control_name, options)  # Instantiate the robot

    def is_connected(self) -> bool:
        return self.__denso_api.is_connected()

    def motor_enabled(self) -> bool:
        return self.__denso_api.motor_enabled()

    def connect(self) -> bool:
        try:
            return self.__denso_api.connect()
        except Exception as e:
            print(e)
            return False

    def disconnect(self) -> bool:
        try:
            return self.__denso_api.disconnect()
        except Exception as e:
            print(e)
            return False

    def motor_on(self) -> bool:
        try:
            return self.__denso_api.motor_on()
        except Exception as e:
            print(e)
            return False

    def motor_off(self) -> bool:
        try:
            return self.__denso_api.motor_off()
        except Exception as e:
            print(e)
            return False

    def move_joints(self, command: RobotJointCommand) -> bool:
        try:
            return self.__denso_api.move_joints(command)
        except Exception as e:
            print(e)
            return False

    def move_cartesian(self, command: RobotCartesianCommand) -> bool:
        try:
            return self.__denso_api.move_cartesian(command)
        except Exception as e:
            print(e)
            return False

    def move_preset_joints(self, pose: str) -> bool:
        if self.is_connected() and self.motor_enabled():
            joints, _ = get_pose(pose, 0)
            if joints:
                command = RobotJointCommand.from_list(joints)
                self.move_joints(command)
                return True
        elif not self.motor_enabled():
            print('\nRobot motor is off')
            return False
        elif not self.is_connected():
            print('\nRobot not connected')
            return False

    def move_preset_cartesian(self, pose: str) -> bool:
        if self.is_connected() and self.motor_enabled():
            coordinates, _ = get_pose(pose, 1)
            if coordinates:
                command = RobotCartesianCommand.from_list(coordinates)
                self.move_cartesian(command)
                return True
        elif not self.motor_enabled():
            print('\nRobot motor is off')
            return False
        elif not self.is_connected():
            print('\nRobot not connected')
            return False

    def move_tool_z(self, z_step: float) -> bool:
        try:
            return self.__denso_api.move_tool_z(z_step)
        except Exception as e:
            print(e)
            return False

    def get_joints_pose(self):
        try:
            return self.__denso_api.get_joints_pose()
        except Exception as e:
            print(e)

    def get_cartesian_pose(self):
        try:
            return self.__denso_api.get_cartesian_pose()
        except Exception as e:
            print(e)

    def set_arm_speed(self, speed, accel, decel) -> bool:
        try:
            return self.__denso_api.set_arm_speed(speed, accel, decel)
        except Exception as e:
            print(e)
            return False

    def get_current_arm_speed(self) -> tuple | None:
        try:
            return self.__denso_api.get_current_arm_speed()
        except Exception as e:
            print(e)

    def get_current_error(self):
        try:
            return self.__denso_api.get_current_error()
        except Exception as e:
            print(e)
            return False

    def get_current_fig(self) -> int | None:
        try:
            return self.__denso_api.get_current_fig()
        except Exception as e:
            print(e)

    def is_out_of_range(self, robot_command) -> bool | None:
        try:
            return self.__denso_api.is_out_of_range(robot_command)
        except Exception as e:
            print(e)

    def joint_to_cartesian(self, command):
        try:
            return self.__denso_api.joint_to_cartesian(command)
        except Exception as e:
            print(e)

    def cartesian_to_joint(self, command):
        try:
            return self.__denso_api.cartesian_to_joint(command)
        except Exception as e:
            print(e)

    def calculate_dist_to(self, position):
        return self.__denso_api.dist_to(position)

    def move_by_variable(self, type_var, number):
        try:
            pos = self.__denso_api.get_variable_content(type_var, number)

            robot_pos = RobotJointCommand(pos[0], pos[1], pos[2], pos[3], pos[4], pos[5])

            return self.__denso_api.move_joints(robot_pos)
        except Exception as e:
            print(e)

    def get_finger_feedback(self) -> int:
        """Acquires the value from the Denso I/O port
        which teh finger is connected to.

        Returns:
            int: 0 or 1, where 0 is a negative feedback.
        """
        caotype = CAOVariableType.IO
        feedback = self.__denso_api.get_variable_content(
            cao_type=caotype,
            number=IO_PORT,
        )
        return feedback
