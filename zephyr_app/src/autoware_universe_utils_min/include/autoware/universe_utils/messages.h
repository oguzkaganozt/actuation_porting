#ifndef AUTOWARE__UNIVERSE_UTILS__MESSAGES_H_
#define AUTOWARE__UNIVERSE_UTILS__MESSAGES_H_

#include "Path.h"
#include "Trajectory.h"
#include "Point32.h"
#include "Point.h"
#include "PoseStamped.h"
#include "PoseWithCovarianceStamped.h"
#include "Quaternion.h"
#include "TransformStamped.h"
#include "Twist.h"
#include "TwistWithCovariance.h"
#include "Vector3.h"
#include "PathWithLaneId.h"
#include "MarkerArray.h"

// Message types for autoware_universe_utils_min
using geometryMsgsPoint = geometry_msgs_msg_Point;
using geometryMsgsPose = geometry_msgs_msg_Pose;
using geometryMsgsVector3 = geometry_msgs_msg_Vector3;
using geometryMsgsPoseStamped = geometry_msgs_msg_PoseStamped;
using geometryMsgsPoseWithCovarianceStamped = geometry_msgs_msg_PoseWithCovarianceStamped;
using geometryMsgsQuaternion = geometry_msgs_msg_Quaternion;
using geometryMsgsTransformStamped = geometry_msgs_msg_TransformStamped;
using visualizationMsgsMarkerArray = visualization_msgs_msg_MarkerArray;
using awfPlanningMsgsTrajectory = autoware_planning_msgs_msg_Trajectory;
using awfPlanningMsgsPathPoint = autoware_planning_msgs_msg_PathPoint;
using tier4PlanningMsgsPathPointWithLaneId = tier4_planning_msgs_msg_PathPointWithLaneId;

#endif  // AUTOWARE__UNIVERSE_UTILS__MESSAGES_H_