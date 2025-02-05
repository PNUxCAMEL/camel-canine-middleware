import rclpy
import threading
import time
from rosCommand import Command
from rosCommunication import ROSCommunication
from sharedMemory import SharedMemoryManager

shm = SharedMemoryManager()

def rosCommunicationThread(args=None):
    rclpy.init(args=args)
    node = ROSCommunication(shm)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

def userCommandThread(args=None):
    shm.cmd.command = Command.START.value
    time.sleep(1)

    shm.cmd.command = Command.HOME_UP.value
    time.sleep(5)

    shm.cmd.command = Command.TROT_SLOW.value
    time.sleep(3)

    shm.cmd.reference_base_velocity = [0.1, 0.0]  
    time.sleep(1)
    print("(",shm.local_time,") body vel:",shm.body_base_velocity)

    time.sleep(1)
    shm.cmd.reference_base_velocity = [-0.1, 0.0]  
    time.sleep(1)
    print("(",shm.local_time,") body vel:",shm.body_base_velocity)

    time.sleep(1)
    shm.cmd.reference_base_velocity = [0.0, 0.1]  
    time.sleep(1)
    print("(",shm.local_time,") body vel:",shm.body_base_velocity)

    time.sleep(1)
    shm.cmd.reference_base_velocity = [0.0, -0.1]  
    time.sleep(1)
    print("(",shm.local_time,") body vel:",shm.body_base_velocity)

    shm.cmd.command = Command.TROT_STOP.value
    time.sleep(4)

    shm.cmd.command = Command.ARM_MOVE.value
    shm.cmd.reference_arm_position = [0.55, 0.0, 0.35]
    shm.cmd.reference_arm_euler_angle = [0.0, 0.0, 0.0]
    time.sleep(4)
    print("(",shm.local_time,") arm pos:\n",shm.arm_end_effector_position)
    print("(",shm.local_time,") arm euler:\n",shm.arm_end_effector_euler_angle)

    shm.cmd.command = Command.ARM_MOVE.value
    shm.cmd.reference_arm_position = [0.55, 0.0, 0.35]
    shm.cmd.reference_arm_euler_angle = [0.0, 0.0, 30 * 3.141592 / 180]
    time.sleep(4)
    print("(",shm.local_time,") arm pos:\n",shm.arm_end_effector_position)
    print("(",shm.local_time,") arm euler:\n",shm.arm_end_effector_euler_angle)

    shm.cmd.command = Command.ARM_HOME.value
    time.sleep(4)

    shm.cmd.command = Command.ARM_MOVE.value
    shm.cmd.reference_arm_position = [0.55, 0.0, 0.25]
    shm.cmd.reference_arm_euler_angle = [0.0, 0.0, -30 * 3.141592 / 180]
    time.sleep(4)
    print("(",shm.local_time,") arm pos:\n",shm.arm_end_effector_position)
    print("(",shm.local_time,") arm euler:\n",shm.arm_end_effector_euler_angle)

    shm.cmd.command = Command.HOME_DOWN.value
    time.sleep(5)

    shm.cmd.command = Command.EMERGENCY_STOP.value
    time.sleep(2)

    shm.cmd.command = Command.RESTART.value
    time.sleep(1)


thread_ros_communication = threading.Thread(target=rosCommunicationThread)
thread_ros_communication.start()
thread_user = threading.Thread(target=userCommandThread)
thread_user.start()