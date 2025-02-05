//
// Created by jh on 25. 2. 3.
//

#include "ROSCommunication.hpp"

ROSCommunication::ROSCommunication()
    : Node("canine_middleware")
{
    sharedMemory = SharedMemory::getInstance();

    subscription_canine_command = this->create_subscription<canine_msgs::msg::CANINECommand>(
        "canine_command", 10,std::bind(&ROSCommunication::topic_callback_canine_command, this, std::placeholders::_1));
    publisher_canine_states = this->create_publisher<canine_msgs::msg::CANINEState>("canine_states", 10);

    timer_canine_states = this->create_wall_timer(
                std::chrono::milliseconds(20),
                std::bind(&ROSCommunication::timer_callback, this));
    std::cout << "[ROS] Generated C++ ROS Communication thread." << std::endl;
}

void ROSCommunication::timer_callback()
{
    if (sharedMemory->isTCPConnected)
    {
        auto msg = canine_msgs::msg::CANINEState();
        package_canine_state_msg(msg);
        publisher_canine_states->publish(msg);
    }
}

void ROSCommunication::package_canine_state_msg(canine_msgs::msg::CANINEState& msg)
{
    msg.gait_table.resize(20);
    msg.contact_state.resize(4);
    msg.motor_error_status.resize(12);
    msg.motor_temp.resize(12);
    msg.motor_voltage.resize(12);
    msg.motor_position.resize(12);
    msg.motor_velocity.resize(12);
    msg.motor_torque.resize(12);
    msg.motor_desired_position.resize(12);
    msg.motor_desired_velocity.resize(12);
    msg.motor_desired_torque.resize(12);
    msg.arm_motor_error_status.resize(7);
    msg.arm_motor_temp.resize(7);
    msg.arm_motor_voltage.resize(7);
    msg.arm_motor_position.resize(7);
    msg.arm_motor_velocity.resize(7);
    msg.arm_motor_torque.resize(7);
    msg.arm_motor_desired_position.resize(7);
    msg.arm_motor_desired_velocity.resize(7);
    msg.arm_motor_desired_torque.resize(7);
    msg.arm_end_effector_position.resize(3);
    msg.arm_end_effector_euler_angle.resize(3);
    msg.arm_end_effector_velocity.resize(3);
    msg.arm_end_effector_angular_velocity.resize(3);

    msg.local_time = sharedMemory->localTime;
    msg.fsm_state = sharedMemory->FSMState;
    std::copy_n(sharedMemory->gaitTable, 20, msg.gait_table.data());
    std::copy_n(sharedMemory->contactState, 4, msg.contact_state.begin());
    msg.motor_status = sharedMemory->motorStatus;

    for (size_t idx = 0; idx < MOTOR_NUM_LEG; idx++)
    {
        msg.motor_error_status[idx] = sharedMemory->motorErrorStatus[idx];
        msg.motor_temp[idx] = sharedMemory->motorTemp[idx];
        msg.motor_position[idx] = sharedMemory->motorPosition[idx];
        msg.motor_velocity[idx] = sharedMemory->motorVelocity[idx];
        msg.motor_torque[idx] = sharedMemory->motorTorque[idx];
        msg.motor_desired_position[idx] = sharedMemory->motorDesiredPosition[idx];
        msg.motor_desired_velocity[idx] = sharedMemory->motorDesiredVelocity[idx];
        msg.motor_desired_torque[idx] = sharedMemory->motorDesiredTorque[idx];
    }

    for (size_t idx = 0; idx < MOTOR_NUM_ARM; idx++)
    {
        msg.arm_motor_error_status[idx] = sharedMemory->armMotorErrorStatus[idx];
        msg.arm_motor_temp[idx] = sharedMemory->armMotorTemp[idx];
        msg.arm_motor_position[idx] = sharedMemory->armMotorPosition[idx];
        msg.arm_motor_velocity[idx] = sharedMemory->armMotorVelocity[idx];
        msg.arm_motor_torque[idx] = sharedMemory->armMotorTorque[idx];
        msg.arm_motor_desired_position[idx] = sharedMemory->armMotorDesiredPosition[idx];
        msg.arm_motor_desired_velocity[idx] = sharedMemory->armMotorDesiredVelocity[idx];
        msg.arm_motor_desired_torque[idx] = sharedMemory->armMotorDesiredTorque[idx];
    }

    for (size_t idx = 0; idx < 3; idx++)
    {
        msg.arm_end_effector_position[idx] = sharedMemory->currentEndEffectorPosition[idx];
        msg.arm_end_effector_euler_angle[idx] = sharedMemory->currentEndEffectorEulerAngle[idx];
        msg.arm_end_effector_velocity[idx] = sharedMemory->currentEndEffectorVelocity[idx];
        msg.arm_end_effector_angular_velocity[idx] = sharedMemory->currentEndEffectorAngularVelocity[idx];
    }

    msg.global_base_position.x = sharedMemory->globalBasePosition.x();
    msg.global_base_position.y = sharedMemory->globalBasePosition.y();
    msg.global_base_position.z = sharedMemory->globalBasePosition.z();
    msg.global_base_velocity.x = sharedMemory->globalBaseVelocity.x();
    msg.global_base_velocity.y = sharedMemory->globalBaseVelocity.y();
    msg.global_base_velocity.z = sharedMemory->globalBaseVelocity.z();
    msg.global_base_euler_angle.x = sharedMemory->globalBaseEulerAngle.x();
    msg.global_base_euler_angle.y = sharedMemory->globalBaseEulerAngle.y();
    msg.global_base_euler_angle.z = sharedMemory->globalBaseEulerAngle.z();
    msg.global_base_quaternion.x = sharedMemory->globalBaseQuaternion[1];
    msg.global_base_quaternion.y = sharedMemory->globalBaseQuaternion[2];
    msg.global_base_quaternion.z = sharedMemory->globalBaseQuaternion[3];
    msg.global_base_quaternion.w = sharedMemory->globalBaseQuaternion[0];
    msg.global_base_angular_velocity.x = sharedMemory->globalBaseAngularVelocity.x();
    msg.global_base_angular_velocity.y = sharedMemory->globalBaseAngularVelocity.y();
    msg.global_base_angular_velocity.z = sharedMemory->globalBaseAngularVelocity.z();

    msg.global_base_desired_position.x = sharedMemory->globalBaseDesiredPosition.x();
    msg.global_base_desired_position.y = sharedMemory->globalBaseDesiredPosition.y();
    msg.global_base_desired_position.z = sharedMemory->globalBaseDesiredPosition.z();
    msg.global_base_desired_velocity.x = sharedMemory->globalBaseDesiredVelocity.x();
    msg.global_base_desired_velocity.y = sharedMemory->globalBaseDesiredVelocity.y();
    msg.global_base_desired_velocity.z = sharedMemory->globalBaseDesiredVelocity.z();
    msg.global_base_desired_euler_angle.x = sharedMemory->globalBaseDesiredEulerAngle.x();
    msg.global_base_desired_euler_angle.y = sharedMemory->globalBaseDesiredEulerAngle.y();
    msg.global_base_desired_euler_angle.z = sharedMemory->globalBaseDesiredEulerAngle.z();
    msg.global_base_desired_quaternion.x = sharedMemory->globalBaseDesiredQuaternion[1];
    msg.global_base_desired_quaternion.y = sharedMemory->globalBaseDesiredQuaternion[2];
    msg.global_base_desired_quaternion.z = sharedMemory->globalBaseDesiredQuaternion[3];
    msg.global_base_desired_quaternion.w = sharedMemory->globalBaseDesiredQuaternion[0];
    msg.global_base_desired_angular_velocity.x = sharedMemory->globalBaseDesiredAngularVelocity.x();
    msg.global_base_desired_angular_velocity.y = sharedMemory->globalBaseDesiredAngularVelocity.y();
    msg.global_base_desired_angular_velocity.z = sharedMemory->globalBaseDesiredAngularVelocity.z();

    msg.body_base_velocity.x = sharedMemory->bodyBaseVelocity.x();
    msg.body_base_velocity.y = sharedMemory->bodyBaseVelocity.y();
    msg.body_base_velocity.z = sharedMemory->bodyBaseVelocity.z();
    msg.body_base_desired_velocity.x = sharedMemory->bodyBaseDesiredVelocity.x();
    msg.body_base_desired_velocity.y = sharedMemory->bodyBaseDesiredVelocity.y();
    msg.body_base_desired_velocity.z = sharedMemory->bodyBaseDesiredVelocity.z();
    msg.body_base_angular_velocity.x = sharedMemory->bodyBaseAngularVelocity.x();
    msg.body_base_angular_velocity.y = sharedMemory->bodyBaseAngularVelocity.y();
    msg.body_base_angular_velocity.z = sharedMemory->bodyBaseAngularVelocity.z();
    msg.body_base_desired_angular_velocity.x = sharedMemory->bodyBaseDesiredAngularVelocity.x();
    msg.body_base_desired_angular_velocity.y = sharedMemory->bodyBaseDesiredAngularVelocity.y();
    msg.body_base_desired_angular_velocity.z = sharedMemory->bodyBaseDesiredAngularVelocity.z();
}

void ROSCommunication::topic_callback_canine_command(const canine_msgs::msg::CANINECommand::SharedPtr msg) const
{
    msg->reference_base_velocity.resize(2);
    msg->reference_arm_position.resize(3);
    msg->reference_arm_euler_angle.resize(3);
    msg->reference_arm_tele_operation_linear_vel.resize(3);
    msg->reference_arm_tele_operation_angular_vel.resize(3);

    sharedMemory->rosCommand.joyCommand = msg->command;
    sharedMemory->rosCommand.userLinVel[0] = msg->reference_base_velocity[0];
    sharedMemory->rosCommand.userLinVel[1] = msg->reference_base_velocity[1];
    sharedMemory->rosCommand.userLinVel[2] = 0.0;
    sharedMemory->rosCommand.userAngVel[0] = 0.0;
    sharedMemory->rosCommand.userAngVel[1] = 0.0;
    sharedMemory->rosCommand.userAngVel[2] = msg->reference_base_yaw_velocity;

    for (int i = 0 ; i < 3 ; i++)
    {
        sharedMemory->rosCommand.desiredEndEffectorPosition[i] = msg->reference_arm_position[i];
        sharedMemory->rosCommand.desiredEndEffectorEulerAngle[i] = msg->reference_arm_euler_angle[i];
        sharedMemory->rosCommand.desiredTeleOperationLinearVelocity[i] = msg->reference_arm_tele_operation_linear_vel[i];
        sharedMemory->rosCommand.desiredTeleOperationAngularVelocity[i] = msg->reference_arm_tele_operation_angular_vel[i];
    }
}


