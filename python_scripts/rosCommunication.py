import numpy as np
from rclpy.node import Node
from canine_msgs.msg import CANINECommand,CANINEState
from sensor_msgs.msg import LaserScan

class ROSCommunication(Node):
    def __init__(self,SharedMemoryManager):
        super().__init__('ros_python_communication')
        self.subscription_canine_states = self.create_subscription(
            CANINEState,
            'canine_states',
            self.topic_callback_canine_states,
            10
        )

        self.subscription_rplidar_a1 =  self.create_subscription(
            LaserScan,
            'scan',
            self.topic_callback_rplidar_a1,
            10
        )

        self.publisher_canine_command = self.create_publisher(
            CANINECommand,
            'canine_command'
            ,10
        )

        self.timer_canine_command = self.create_timer(0.02, self.timer_callback)

        self.subscription_canine_states
        self.subscription_rplidar_a1
        self.publisher_canine_command
        self.timer_canine_command
        
        self.shm = SharedMemoryManager
        self.cnt = 0
        self.prevCMD = -1

    def topic_callback_canine_states(self, msg):
        self.shm.middleware_connected = True
        self.shm.local_time = msg.local_time
        self.shm.fsm_state = msg.fsm_state
        self.shm.arm_fsm_state = msg.arm_fsm_state
        self.shm.motor_status = msg.motor_status

        for idx in range(20):
            self.shm.gait_table[idx] = msg.gait_table[idx]
        
        for idx in range(12):
            self.shm.motor_error_status[idx] = msg.motor_error_status[idx]
            self.shm.motor_temp[idx] = msg.motor_temp[idx]
            self.shm.motor_voltage[idx] = msg.motor_voltage[idx]
            self.shm.motor_position[idx] = msg.motor_position[idx]
            self.shm.motor_velocity[idx] = msg.motor_velocity[idx]
            self.shm.motor_torque[idx] = msg.motor_torque[idx]
            self.shm.motor_desired_position[idx] = msg.motor_desired_position[idx]
            self.shm.motor_desired_velocity[idx] = msg.motor_desired_velocity[idx]
            self.shm.motor_desired_torque[idx] = msg.motor_desired_torque[idx]

        for idx in range(7):
            self.shm.arm_motor_error_status[idx] = msg.arm_motor_error_status[idx]
            self.shm.arm_motor_temp[idx] = msg.arm_motor_temp[idx]
            self.shm.arm_motor_voltage[idx] = msg.arm_motor_voltage[idx]
            self.shm.arm_motor_position[idx] = msg.arm_motor_position[idx]
            self.shm.arm_motor_velocity[idx] = msg.arm_motor_velocity[idx]
            self.shm.arm_motor_torque[idx] = msg.arm_motor_torque[idx]
            self.shm.arm_motor_desired_position[idx] = msg.arm_motor_desired_position[idx]
            self.shm.arm_motor_desired_velocity[idx] = msg.arm_motor_desired_velocity[idx]
            self.shm.arm_motor_desired_torque[idx] = msg.arm_motor_desired_torque[idx]


        for idx in range(3):
            self.shm.arm_end_effector_position[idx] = msg.arm_end_effector_position[idx]
            self.shm.arm_end_effector_euler_angle[idx] = msg.arm_end_effector_euler_angle[idx]
            self.shm.arm_end_effector_velocity[idx] = msg.arm_end_effector_velocity[idx]
            self.shm.arm_end_effector_angular_velocity[idx] = msg.arm_end_effector_angular_velocity[idx]

        for idx in range(4):
            self.shm.contact_state[idx] = msg.contact_state[idx]

        self.shm.global_base_position = np.array([msg.global_base_position.x, msg.global_base_position.y, msg.global_base_position.z])
        self.shm.global_base_velocity = np.array([msg.global_base_velocity.x, msg.global_base_velocity.y, msg.global_base_velocity.z])
        self.shm.global_base_euler_angle = np.array([msg.global_base_euler_angle.x, msg.global_base_euler_angle.y, msg.global_base_euler_angle.z])
        self.shm.global_base_angular_velocity = np.array([msg.global_base_angular_velocity.x, msg.global_base_angular_velocity.y, msg.global_base_angular_velocity.z])
        self.shm.global_base_desired_position = np.array([msg.global_base_desired_position.x, msg.global_base_desired_position.y, msg.global_base_desired_position.z])
        self.shm.global_base_desired_velocity = np.array([msg.global_base_desired_velocity.x, msg.global_base_desired_velocity.y, msg.global_base_desired_velocity.z])
        self.shm.global_base_desired_euler_angle = np.array([msg.global_base_desired_euler_angle.x, msg.global_base_desired_euler_angle.y, msg.global_base_desired_euler_angle.z])
        self.shm.global_base_desired_angular_velocity = np.array([msg.global_base_desired_angular_velocity.x, msg.global_base_desired_angular_velocity.y, msg.global_base_desired_angular_velocity.z])
        self.shm.body_base_velocity = np.array([msg.body_base_velocity.x, msg.body_base_velocity.y, msg.body_base_velocity.z])
        self.shm.body_base_desired_velocity = np.array([msg.body_base_desired_velocity.x, msg.body_base_desired_velocity.y, msg.body_base_desired_velocity.z])
        self.shm.body_base_angular_velocity = np.array([msg.body_base_angular_velocity.x, msg.body_base_angular_velocity.y, msg.body_base_angular_velocity.z])
        self.shm.body_base_desired_angular_velocity = np.array([msg.body_base_desired_angular_velocity.x, msg.body_base_desired_angular_velocity.y, msg.body_base_desired_angular_velocity.z])
        self.shm.global_base_quaternion = np.array([msg.global_base_quaternion.x, msg.global_base_quaternion.y, msg.global_base_quaternion.z, msg.global_base_quaternion.w])
        self.shm.global_base_desired_quaternion = np.array([msg.global_base_desired_quaternion.x, msg.global_base_desired_quaternion.y, msg.global_base_desired_quaternion.z, msg.global_base_desired_quaternion.w])

        # For debugging
        # self.get_logger().info(f'Local Time: {self.shm.local_time}')
        # print('Gait Table \n',self.shm.gait_table)
        # print('Global Base Position \n',self.shm.global_base_position)
        # print('Global Base Velocity \n',self.shm.global_base_velocity)
        # print('Body Base Velocity \n',self.shm.body_base_velocity)
        # print('Global Base Quaternion \n',self.shm.global_base_quaternion)

    def topic_callback_rplidar_a1(self, msg):
        # Topic information: Laser Scan        
        # stamp:
        #     sec: 
        #     nanosec: 
        # frame_id: 
        # angle_min: 
        # angle_max: 
        # angle_increment: 
        # time_increment: 
        # scan_time: 
        # range_min: 
        # range_max: 
        # ranges: 
        # intensities: 

        self.shm.lidar_msg = msg

        # For debugging
        # self.get_logger().info(f'Received LaserScan: {len(self.shm.lidar_msg.ranges)} points')
        # angle_increment = self.shm.lidar_msg.angle_increment
        # idx = 1
        # theta = (idx - 1) * angle_increment
        # distance = self.shm.lidar_msg.ranges[idx]
        # print("theta: ",theta,"distance: ",distance)





    def timer_callback(self):
        msg = CANINECommand()
        msg = self.shm.cmd

        if self.prevCMD == self.shm.cmd.command:
            self.cnt = self.cnt + 1
        else:
            self.cnt = 0

        if self.cnt > 50:
            self.cnt = 0
            self.shm.cmd.command = 0

        self.prevCMD = self.shm.cmd.command
        self.publisher_canine_command.publish(msg)

