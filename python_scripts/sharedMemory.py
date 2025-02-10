import numpy as np
from canine_msgs.msg import CANINECommand
from sensor_msgs.msg import LaserScan

np.set_printoptions(precision=3)

class SharedMemoryManager:
    def __init__(self):
        # Canine states information
        self.local_time = 0.0
        self.fsm_state = -1
        self.arm_fsm_state = -1
        self.gait_table = np.zeros(20)
        self.contact_state = np.zeros(4)
        self.motor_status = -1
        self.motor_error_status = np.zeros(12)
        self.motor_temp = np.zeros(12)
        self.motor_voltage = np.zeros(12)
        self.motor_position = np.zeros(12)
        self.motor_velocity = np.zeros(12)
        self.motor_torque = np.zeros(12)
        self.motor_desired_position = np.zeros(12)
        self.motor_desired_velocity = np.zeros(12)
        self.motor_desired_torque = np.zeros(12)
        self.arm_motor_error_status = np.zeros(7)
        self.arm_motor_temp = np.zeros(7)
        self.arm_motor_voltage = np.zeros(7)
        self.arm_motor_position = np.zeros(7)
        self.arm_motor_velocity = np.zeros(7)
        self.arm_motor_torque = np.zeros(7)
        self.arm_motor_desired_position = np.zeros(7)
        self.arm_motor_desired_velocity = np.zeros(7)
        self.arm_motor_desired_torque = np.zeros(7)
        self.arm_end_effector_position = np.zeros(3)
        self.arm_end_effector_euler_angle = np.zeros(3)
        self.arm_end_effector_velocity = np.zeros(3)
        self.arm_end_effector_angular_velocity = np.zeros(3)
        self.global_base_position = np.zeros(3)
        self.global_base_velocity = np.zeros(3)
        self.global_base_euler_angle = np.zeros(3)
        self.global_base_quaternion = np.zeros(4)
        self.global_base_angular_velocity = np.zeros(3)
        self.global_base_desired_position = np.zeros(3)
        self.global_base_desired_velocity = np.zeros(3)
        self.global_base_desired_euler_angle = np.zeros(3)
        self.global_base_desired_quaternion = np.zeros(4)
        self.global_base_desired_angular_velocity = np.zeros(3)
        self.body_base_velocity = np.zeros(3)
        self.body_base_desired_velocity = np.zeros(3)
        self.body_base_angular_velocity = np.zeros(3)
        self.body_base_desired_angular_velocity = np.zeros(3)
        self.middleware_connected = False

        self.cmd = CANINECommand()
        self.cmd.command = 0
        self.cmd.reference_base_velocity = [0.0, 0.0]
        self.cmd.reference_base_yaw_velocity = 0.0
        self.cmd.reference_arm_position = [0.0, 0.0, 0.0]
        self.cmd.reference_arm_euler_angle = [0.0, 0.0, 0.0]
        self.cmd.reference_arm_tele_operation_linear_vel = [0.0, 0.0, 0.0]
        self.cmd.reference_arm_tele_operation_angular_vel = [0.0, 0.0, 0.0]

        # Lidar information
        self.lidar_msg = LaserScan()