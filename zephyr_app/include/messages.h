#ifndef ACTUATION_AUTOWARE__MESSAGES_H_
#define ACTUATION_AUTOWARE__MESSAGES_H_

#include "Point.h"
#include "Path.h"
#include "Trajectory.h"
#include "TrajectoryPoint.h"
#include "PoseStamped.h"
#include "PoseWithCovarianceStamped.h"
#include "Quaternion.h"
#include "Transform.h"
#include "TransformStamped.h"
#include "Twist.h"
#include "TwistStamped.h"
#include "TwistWithCovariance.h"
#include "Odometry.h"
#include "Vector3.h"
#include "MarkerArray.h"
#include "PathPoint.h"
#include "PathWithLaneId.h"
#include "PathPointWithLaneId.h"

// Message types for actuation
using geometryMsgsPoint = geometry_msgs_msg_Point;
using geometryMsgsPose = geometry_msgs_msg_Pose;
using geometryMsgsVector3 = geometry_msgs_msg_Vector3;
using geometryMsgsPoseStamped = geometry_msgs_msg_PoseStamped;
using geometryMsgsPoseWithCovarianceStamped = geometry_msgs_msg_PoseWithCovarianceStamped;
using geometryMsgsQuaternion = geometry_msgs_msg_Quaternion;
using geometryMsgsTransformStamped = geometry_msgs_msg_TransformStamped;
using geometryMsgsTwist = geometry_msgs_msg_Twist;
using geometryMsgsTwistStamped = geometry_msgs_msg_TwistStamped;
using navMsgsOdometry = nav_msgs_msg_Odometry;
using visualizationMsgsMarkerArray = visualization_msgs_msg_MarkerArray;
using awfPlanningMsgsTrajectory = autoware_planning_msgs_msg_Trajectory;
using awfPlanningMsgsTrajectoryPoint = autoware_planning_msgs_msg_TrajectoryPoint;
using awfPlanningMsgsPathPoint = autoware_planning_msgs_msg_PathPoint;
using tier4PlanningMsgsPathPointWithLaneId = tier4_planning_msgs_msg_PathPointWithLaneId;
using geometryMsgsTransform = geometry_msgs_msg_Transform;

// Simplified message names
using PointMsg = geometryMsgsPoint;
using PoseMsg = geometryMsgsPose;
using Vector3Msg = geometryMsgsVector3;
using PoseStampedMsg = geometryMsgsPoseStamped;
using PoseWithCovarianceStampedMsg = geometryMsgsPoseWithCovarianceStamped;
using QuaternionMsg = geometryMsgsQuaternion;
using TransformStampedMsg = geometryMsgsTransformStamped;
using TwistMsg = geometryMsgsTwist;
using TwistStampedMsg = geometryMsgsTwistStamped;
using OdometryMsg = navMsgsOdometry;
using MarkerArrayMsg = visualizationMsgsMarkerArray;
using TrajectoryMsg = awfPlanningMsgsTrajectory;
using TrajectoryPointMsg = awfPlanningMsgsTrajectoryPoint;
using PathPointMsg = awfPlanningMsgsPathPoint;
using PathPointWithLaneIdMsg = tier4PlanningMsgsPathPointWithLaneId;
using TransformMsg = geometryMsgsTransform;

#endif  // ACTUATION_AUTOWARE__MESSAGES_H_