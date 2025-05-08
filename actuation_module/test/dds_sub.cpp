#include "common/node/node.hpp"
#include "common/clock/clock.hpp"
#include "common/sequence/sequence.hpp"
#include "common/logger/logger.hpp"
using namespace common::logger;

// Msgs
#include "SteeringReport.h"
#include "Trajectory.h"
#include "Odometry.h"
#include "AccelWithCovarianceStamped.h"
#include "OperationModeState.h"
using SteeringReportMsg = autoware_vehicle_msgs_msg_SteeringReport;
using TrajectoryMsg = autoware_planning_msgs_msg_Trajectory;
using OdometryMsg = nav_msgs_msg_Odometry;
using AccelerationMsg = geometry_msgs_msg_AccelWithCovarianceStamped;
using OperationModeStateMsg = autoware_adapi_v1_msgs_msg_OperationModeState;

// Stack sizes for node and timer threads
#if defined(NATIVE_SIM)
static unsigned char node_stack[CONFIG_THREAD_STACK_SIZE];
static unsigned char timer_stack[CONFIG_THREAD_STACK_SIZE];
#else
static K_THREAD_STACK_DEFINE(node_stack, CONFIG_THREAD_STACK_SIZE);
static K_THREAD_STACK_DEFINE(timer_stack, CONFIG_THREAD_STACK_SIZE);
#define STACK_SIZE (K_THREAD_STACK_SIZEOF(node_stack))
#endif

/*
    This test is used to test the DDS communication between ROS2 and Zephyr
    It is used to validate the message conversion between ROS2 and Zephyr
    It is used to validate the sequence wrapper    
*/
static void handle_steering_report(SteeringReportMsg& msg) {
    log_info("------ STEERING REPORT ------\n");
    log_info("Timestamp: %d\n", Clock::toDouble(msg.stamp));
    log_info("Steering tire angle: %lf\n", msg.steering_tire_angle);
    log_info("-------------------------------\n");
}

static void handle_operation_mode_state(OperationModeStateMsg& msg) {
    log_info("------ OPERATION MODE STATE ------\n");
    log_info("Timestamp: %d\n", Clock::toDouble(msg.stamp));
    log_info("Mode: %d\n", msg.mode);
    log_info("Autoware control enabled: %d\n", msg.is_autoware_control_enabled);
    log_info("In transition: %d\n", msg.is_in_transition);
    log_info("-------------------------------\n");
}

static void handle_odometry(OdometryMsg& msg) {
    log_info("------ ODOMETRY ------\n");
    log_info("Timestamp: %d\n", Clock::toDouble(msg.header.stamp));
    log_info("Position: %lf, %lf, %lf\n", msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);
    log_info("Linear Twist: %lf, %lf, %lf\n", msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z);
    log_info("-------------------------------\n");
}

static void handle_acceleration(AccelerationMsg& msg) {
    log_info("------ ACCELERATION ------\n");
    log_info("Timestamp: %d\n", Clock::toDouble(msg.header.stamp));
    log_info("Linear acceleration: %lf, %lf, %lf\n", msg.accel.accel.linear.x, msg.accel.accel.linear.y, msg.accel.accel.linear.z);
    log_info("Angular acceleration: %lf, %lf, %lf\n", msg.accel.accel.angular.x, msg.accel.accel.angular.y, msg.accel.accel.angular.z);
    log_info("-------------------------------\n");
}

static void handle_trajectory(TrajectoryMsg& msg) {
    log_info("------ TRAJECTORY ------\n");
    log_info("Timestamp: %d\n", Clock::toDouble(msg.header.stamp));
    log_info("Trajectory size: %d\n", msg.points._length);
    log_info("-------------------------------\n");
}

int main(void) {
    log_info("--------------------------------\n");
    log_info("Starting DDS subscriber\n");
    log_info("--------------------------------\n");
    log_info("Waiting for Network interface to be ready\n");
    sleep(5);

    // Setting time using SNTP
    if (Clock::init_clock_via_sntp() < 0) {
        log_error("Failed to set time using SNTP\n");
    }
    else {
        log_info("Time set using SNTP\n");
    }
    
    // Create a node
    Node node("dds_test_sub", node_stack, STACK_SIZE, timer_stack, STACK_SIZE);

    // Create subscribers
    auto subscriber = node.create_subscription<SteeringReportMsg>("/vehicle/status/steering_status",
                                                                &autoware_vehicle_msgs_msg_SteeringReport_desc,
                                                                handle_steering_report);
    auto subscriber_trajectory = node.create_subscription<TrajectoryMsg>("/planning/scenario_planning/trajectory",
                                                                &autoware_planning_msgs_msg_Trajectory_desc,
                                                                handle_trajectory);
    auto subscriber_odometry = node.create_subscription<OdometryMsg>("/localization/kinematic_state",
                                                                &nav_msgs_msg_Odometry_desc,
                                                                handle_odometry);
    auto subscriber_acceleration = node.create_subscription<AccelerationMsg>("/localization/acceleration",
                                                                &geometry_msgs_msg_AccelWithCovarianceStamped_desc,
                                                                handle_acceleration);
    auto subscriber_operation_mode_state = node.create_subscription<OperationModeStateMsg>("/system/operation_mode/state",
                                                                &autoware_adapi_v1_msgs_msg_OperationModeState_desc,
                                                                handle_operation_mode_state);

    log_info("--------------------------------\n");
    log_info("DDS subscriber started\n");
    log_info("--------------------------------\n");

    while(true) {
        sleep(1);
    }

    return 0;
}