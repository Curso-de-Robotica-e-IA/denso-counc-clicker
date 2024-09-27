from abc import ABC, abstractmethod

from rria_api_denso import RobotCartesianCommand, RobotJointCommand


class AbstractDenso(ABC):
    """
    Abstract robot class
    """

    @abstractmethod
    def is_connected(self) -> bool:
        """
        Check connection
        """
        ...

    @abstractmethod
    def motor_enabled(self) -> bool:
        """
        Check whether the motor is turned on
        """
        ...

    @abstractmethod
    def connect(self) -> bool:
        """
        Connect the robot
        """
        ...

    @abstractmethod
    def disconnect(self) -> bool:
        """
        Disconnect the robot
        """
        ...

    @abstractmethod
    def motor_on(self) -> bool:
        """
        Turn on the motor
        """
        ...

    @abstractmethod
    def motor_off(self) -> bool:
        """
        Turn off the motor
        """
        ...

    @abstractmethod
    def move_joints(self, command: RobotJointCommand) -> bool:
        """
        Move robot joints according to command
        """
        ...

    @abstractmethod
    def move_cartesian(self, command: RobotCartesianCommand) -> bool:
        """
        Move robot in cartesian manner according to command
        """
        ...

    @abstractmethod
    def move_preset_joints(self, pose: str) -> bool:
        """
        Move robot to a preset joints pose
        """
        ...

    @abstractmethod
    def move_preset_cartesian(self, pose: str) -> bool:
        """
        Move robot to a preset cartesian pose
        """
        ...

    @abstractmethod
    def get_joints_pose(self) -> RobotJointCommand | None:
        """
        Return current joints configuration
        """
        ...

    @abstractmethod
    def get_cartesian_pose(self) -> RobotCartesianCommand | None:
        """
        Return current cartesian configuration
        """
        ...

    @abstractmethod
    def set_arm_speed(self, speed: float, accel: float, decel: float) -> bool:
        """
        Setup speed, acceleration and deceleration for the robotic arm
        """
        ...

    @abstractmethod
    def get_current_arm_speed(self) -> tuple | None:
        """
        Return current speed, acceleration and deceleration of the robotic arm
        """
        ...

    @abstractmethod
    def get_current_error(self):
        """
        Return current raised error
        """
        ...

    @abstractmethod
    def get_current_fig(self) -> int | None:
        """
        Return current cartesian figure
        """
        ...

    @abstractmethod
    def is_out_of_range(self, robot_command) -> bool | None:
        """
        Check whether the arm is whithin workspace
        """
        ...

    @abstractmethod
    def joint_to_cartesian(self, command: RobotJointCommand) -> RobotCartesianCommand | None:
        """
        Convert joint to cartesian pose
        """
        ...

    @abstractmethod
    def cartesian_to_joint(self, command: RobotCartesianCommand) -> RobotJointCommand | None:
        """
        Convert cartesian to joint pose
        """
        ...

    @abstractmethod
    def move_tool_z(self, z_step: float) -> bool:
        """
        Move end effector forward or backward according to step size
        """
        ...

    @abstractmethod
    def calculate_dist_to(self, position):
        """
        Return distance between current pose and target pose
        """
        ...
