import rclpy
import numpy as np
from rclpy.node import Node
from canine_msgs.msg import CANINECommand,CANINEState
from enum import Enum
from typing import Literal

class Command(Enum):
    NO_INPUT = 0
    START = 1
    EMERGENCY_STOP = 2
    STAND_UP = 3
    SIT_DOWN = 4
    TROT_STOP = 5
    TROT_SLOW = 6
    TROT_FAST = 7
    TROT_OVERLAP = 8
    RESTART = 9
    ARM_HOME = 10
    ARM_MOVE = 11
    ARM_TELE_ON = 12
    ARM_TELE_OFF = 13
    ARM_GRP_OPEN = 14
    ARM_GRP_CLOSE = 15

class ROSCommunicatoin(Node):
    def __init__(self):
        super().__init__('ros_python_communication')
        self.subscription_canine_states = self.create_subscription(
            CANINEState,
            'canine_states',
            self.topic_callback_canine_states,
            10
        )
        self.publisher_canine_command = self.create_publisher(
            CANINECommand,
            'canine_command'
            ,10
        )
        self.timer_canine_command = self.create_timer(0.02, self.timer_callback)

        self.subscription_canine_states
        self.publisher_canine_command
        self.timer_canine_command
        self.command = Command.NO_INPUT

        ### canine states (from middleware)
        self.local_time = 0.0
        self.fsm_state = -1
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
        

    def topic_callback_canine_states(self, msg):
        self.local_time = msg.local_time
        self.fsm_state = msg.fsm_state
        self.motor_status = msg.motor_status

        for idx in range(20):
            self.gait_table[idx] = msg.gait_table[idx]
        
        for idx in range(12):
            self.motor_error_status[idx] = msg.motor_error_status[idx]
            self.motor_temp[idx] = msg.motor_temp[idx]
            self.motor_voltage[idx] = msg.motor_voltage[idx]
            self.motor_position[idx] = msg.motor_position[idx]
            self.motor_velocity[idx] = msg.motor_velocity[idx]
            self.motor_torque[idx] = msg.motor_torque[idx]
            self.motor_desired_position[idx] = msg.motor_desired_position[idx]
            self.motor_desired_velocity[idx] = msg.motor_desired_velocity[idx]
            self.motor_desired_torque[idx] = msg.motor_desired_torque[idx]

        for idx in range(7):
            self.arm_motor_error_status[idx] = msg.arm_motor_error_status[idx]
            self.arm_motor_temp[idx] = msg.arm_motor_temp[idx]
            self.arm_motor_voltage[idx] = msg.arm_motor_voltage[idx]
            self.arm_motor_position[idx] = msg.arm_motor_position[idx]
            self.arm_motor_velocity[idx] = msg.arm_motor_velocity[idx]
            self.arm_motor_torque[idx] = msg.arm_motor_torque[idx]
            self.arm_motor_desired_position[idx] = msg.arm_motor_desired_position[idx]
            self.arm_motor_desired_velocity[idx] = msg.arm_motor_desired_velocity[idx]
            self.arm_motor_desired_torque[idx] = msg.arm_motor_desired_torque[idx]

        for idx in range(4):
            self.contact_state[idx] = msg.contact_state[idx]

    
        self.global_base_position = [msg.global_base_position.x, msg.global_base_position.y, msg.global_base_position.z]
        self.global_base_velocity = [msg.global_base_velocity.x, msg.global_base_velocity.y, msg.global_base_velocity.z]
        self.global_base_euler_angle = [msg.global_base_euler_angle.x, msg.global_base_euler_angle.y, msg.global_base_euler_angle.z]
        self.global_base_angular_velocity = [msg.global_base_angular_velocity.x, msg.global_base_angular_velocity.y, msg.global_base_angular_velocity.z]
        self.global_base_desired_position = [msg.global_base_desired_position.x, msg.global_base_desired_position.y, msg.global_base_desired_position.z]
        self.global_base_desired_velocity = [msg.global_base_desired_velocity.x, msg.global_base_desired_velocity.y, msg.global_base_desired_velocity.z]
        self.global_base_desired_euler_angle = [msg.global_base_desired_euler_angle.x, msg.global_base_desired_euler_angle.y, msg.global_base_desired_euler_angle.z]
        self.global_base_desired_angular_velocity = [msg.global_base_desired_angular_velocity.x, msg.global_base_desired_angular_velocity.y, msg.global_base_desired_angular_velocity.z]
        self.body_base_velocity = [msg.body_base_velocity.x, msg.body_base_velocity.y, msg.body_base_velocity.z]
        self.body_base_desired_velocity = [msg.body_base_desired_velocity.x, msg.body_base_desired_velocity.y, msg.body_base_desired_velocity.z]
        self.body_base_angular_velocity = [msg.body_base_angular_velocity.x, msg.body_base_angular_velocity.y, msg.body_base_angular_velocity.z]
        self.body_base_desired_angular_velocity = [msg.body_base_desired_angular_velocity.x, msg.body_base_desired_angular_velocity.y, msg.body_base_desired_angular_velocity.z]
        self.global_base_quaternion = [msg.global_base_quaternion.x, msg.global_base_quaternion.y, msg.global_base_quaternion.z, msg.global_base_quaternion.w]
        self.global_base_desired_quaternion = [msg.global_base_desired_quaternion.x, msg.global_base_desired_quaternion.y, msg.global_base_desired_quaternion.z, msg.global_base_desired_quaternion.w]

        self.get_logger().info(f'Local Time: {self.local_time}')
        print('Gait Table \n',self.gait_table)
        print('Global Base Position \n',self.global_base_position)
        print('Global Base Velocity \n',self.global_base_velocity)
        print('Body Base Velocity \n',self.body_base_velocity)
        print('Global Base Quaternion \n',self.global_base_quaternion)
        

    def timer_callback(self):
        msg = CANINECommand()
        msg.command = self.command.value
        msg.reference_base_velocity = [0.0, 0.0]
        msg.reference_base_yaw_velocity = 0.0
        msg.reference_arm_position = [0.0, 0.0, 0.0]
        msg.reference_arm_euler_angle = [0.0, 0.0, 0.0]
        msg.reference_arm_tele_operation_linear_vel = [0.0, 0.0, 0.0]
        msg.reference_arm_tele_operation_angular_vel = [0.0, 0.0, 0.0]
        self.publisher_canine_command.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    print("test")
    node = ROSCommunicatoin()
    print("test")
    try:
        rclpy.spin(node)
        print("test")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    print("test2")

if __name__ == '__main__':
    main()
    print("test3")