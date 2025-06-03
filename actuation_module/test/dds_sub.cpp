#include "common/node/node.hpp"
#include "common/clock/clock.hpp"
#include "common/dds/sequence.hpp"
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
static void handle_steering_report(const SteeringReportMsg* msg, void* arg) {
    log_info("\n------ STEERING REPORT ------\n");
    log_info("Timestamp: %d\n", Clock::toDouble(msg->stamp));
    log_info("Steering tire angle: %lf\n", msg->steering_tire_angle);
    log_info("-------------------------------\n");
}

static void handle_operation_mode_state(const OperationModeStateMsg* msg, void* arg) {
    log_info("\n------ OPERATION MODE STATE ------\n");
    log_info("Timestamp: %d\n", Clock::toDouble(msg->stamp));
    log_info("Mode: %d\n", msg->mode);
    log_info("Autoware control enabled: %d\n", msg->is_autoware_control_enabled);
    log_info("In transition: %d\n", msg->is_in_transition);
    log_info("-------------------------------\n");
}

static void handle_odometry(const OdometryMsg* msg, void* arg) {
    log_info("\n------ ODOMETRY ------\n");
    log_info("Timestamp: %d\n", Clock::toDouble(msg->header.stamp));
    log_info("Position: %lf, %lf, %lf\n", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    log_info("Linear Twist: %lf, %lf, %lf\n", msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    log_info("-------------------------------\n");
}

static void handle_acceleration(const AccelerationMsg* msg, void* arg) {
    log_info("\n------ ACCELERATION ------\n");
    log_info("Timestamp: %d\n", Clock::toDouble(msg->header.stamp));
    log_info("Linear acceleration: %lf, %lf, %lf\n", msg->accel.accel.linear.x, msg->accel.accel.linear.y, msg->accel.accel.linear.z);
    log_info("Angular acceleration: %lf, %lf, %lf\n", msg->accel.accel.angular.x, msg->accel.accel.angular.y, msg->accel.accel.angular.z);
    log_info("-------------------------------\n");
}

static void handle_trajectory(const TrajectoryMsg* msg, void* arg) {
    log_info("\n------ TRAJECTORY ------\n");
    log_info("Timestamp: %d\n", Clock::toDouble(msg->header.stamp));
    log_info("Trajectory size: %d\n", msg->points._length);
    auto points = wrap(msg->points);
    size_t count = 0;
    for (auto point : points) {
        log_info("--------------------------------\n");
        if (count >= 10) break;
        log_info("Long. Velocity: %lf\n", point.longitudinal_velocity_mps);
        log_info("Lat. Velocity: %lf\n", point.lateral_velocity_mps);
        log_info("Accelleration: %lf\n", point.acceleration_mps2);
        log_info("Position: %lf, %lf, %lf\n", point.pose.position.x, point.pose.position.y, point.pose.position.z);
        count++;
    }
    if (points.size() > 10) {
        log_info("... and %zu more points\n", points.size() - 10);
    }
    log_info("-------------------------------\n");
}


static void callbackTimer(void* arg) {
    Node* node = static_cast<Node*>(arg);
    log_info("Callback timer\n");
}

int main(void) {
    log_info("--------------------------------\n");
    log_info("Starting DDS subscriber\n");
    log_info("--------------------------------\n");
    log_info("Waiting for Network interface to be ready\n");
    sleep(5);

    // TODO: IF WE SET TIME USING SNTP, ROSBAGS ARE NOT WORKING
    // // Setting time using SNTP
    // if (Clock::init_clock_via_sntp() < 0) {
    //     log_error("Failed to set time using SNTP\n");
    // }
    // else {
    //     log_info("Time set using SNTP\n");
    // }
    
    // Create a node
    Node node("dds_test_sub", node_stack, STACK_SIZE, timer_stack, STACK_SIZE);

    // Create test timer
    node.create_timer(500, callbackTimer, &node);

    // Create subscribers
    node.create_subscription<SteeringReportMsg>("/vehicle/status/steering_status",
                                                                &autoware_vehicle_msgs_msg_SteeringReport_desc,
                                                                handle_steering_report, &node);
    node.create_subscription<TrajectoryMsg>("/planning/scenario_planning/trajectory",
                                                                &autoware_planning_msgs_msg_Trajectory_desc,
                                                                handle_trajectory, &node);
    node.create_subscription<OdometryMsg>("/localization/kinematic_state",
                                                                &nav_msgs_msg_Odometry_desc,
                                                                handle_odometry, &node);
    node.create_subscription<AccelerationMsg>("/localization/acceleration",
                                                                &geometry_msgs_msg_AccelWithCovarianceStamped_desc,
                                                                handle_acceleration, &node);
    node.create_subscription<OperationModeStateMsg>("/system/operation_mode/state",
                                                                &autoware_adapi_v1_msgs_msg_OperationModeState_desc,
                                                                handle_operation_mode_state, &node);

    log_info("--------------------------------\n");
    log_info("DDS subscriber started\n");
    log_info("--------------------------------\n");
    node.spin();

    while(true) {
        sleep(1);
    }

    return 0;
}