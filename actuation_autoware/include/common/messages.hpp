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
#include "DetectedObject.h"
#include "DetectedObjects.h"
#include "Float32MultiArrayStamped.h"
#include "Float64MultiArrayStamped.h"
#include "Int32MultiArrayStamped.h"
#include "Int64MultiArrayStamped.h"
#include "ImageMarker.h"
#include "LaneletRoute.h"
#include "LaneletSegment.h"
#include "Marker.h"
#include "MultiArrayLayout.h"
#include "PoseArray.h"
#include "PredictedObject.h"
#include "PredictedObjects.h"
#include "PredictedObjectKinematics.h"
#include "ProcessingTimeTree.h"
#include "SystemUsageArray.h"
#include "TrackedObjects.h"
#include "TrafficLightGroup.h"
#include "TrafficLightGroupArray.h"
#include "TrafficSignal.h"
#include "TrafficSignalArray.h"
#include "Lateral.h"
#include "Longitudinal.h"
#include "AccelWithCovarianceStamped.h"
#include "SteeringReport.h"
#include "OperationModeState.h"
#include "Engage.h"

#include "common/sequence.hpp"

// FROM AUTOWARE_MSGS
// Message types from autoware_msgs
using geometryMsgsPoint = geometry_msgs_msg_Point;
using geometryMsgsPose = geometry_msgs_msg_Pose;
using geometryMsgsVector3 = geometry_msgs_msg_Vector3;
using geometryMsgsPoseStamped = geometry_msgs_msg_PoseStamped;
using geometryMsgsPoseWithCovarianceStamped = geometry_msgs_msg_PoseWithCovarianceStamped;
using geometryMsgsQuaternion = geometry_msgs_msg_Quaternion;
using geometryMsgsTransform = geometry_msgs_msg_Transform;
using geometryMsgsTransformStamped = geometry_msgs_msg_TransformStamped;
using geometryMsgsTwist = geometry_msgs_msg_Twist;
using geometryMsgsTwistStamped = geometry_msgs_msg_TwistStamped;
using navMsgsOdometry = nav_msgs_msg_Odometry;
using visualizationMsgsMarkerArray = visualization_msgs_msg_MarkerArray;
using awfPlanningMsgsTrajectory = autoware_planning_msgs_msg_Trajectory;
using awfPlanningMsgsTrajectoryPoint = autoware_planning_msgs_msg_TrajectoryPoint;
using awfPlanningMsgsPathPoint = autoware_planning_msgs_msg_PathPoint;
using awfPlanningMsgsPathWithLaneId = tier4_planning_msgs_msg_PathWithLaneId;
using tier4PlanningMsgsPathPointWithLaneId = tier4_planning_msgs_msg_PathPointWithLaneId;
using awfControlMsgsLateral = autoware_control_msgs_msg_Lateral;
using awfControlMsgsLongitudinal = autoware_control_msgs_msg_Longitudinal;
using geometryMsgsAccelWithCovarianceStamped = geometry_msgs_msg_AccelWithCovarianceStamped;
using awfVehicleMsgsSteeringReport = autoware_vehicle_msgs_msg_SteeringReport;
using awfAdapiV1MsgsOperationModeState = autoware_adapi_v1_msgs_msg_OperationModeState;
using awfVehicleMsgsEngage = autoware_vehicle_msgs_msg_Engage;

// Sequence types from autoware_msgs
using awfPlanningMsgsSequenceTrajectoryPoint = dds_sequence_autoware_planning_msgs_msg_TrajectoryPoint;
using awfPerceptionMsgsSequenceObjectClassification = dds_sequence_autoware_perception_msgs_msg_ObjectClassification;
using awfPerceptionMsgsSequenceDetectedObject = dds_sequence_autoware_perception_msgs_msg_DetectedObject;
using awfPlanningMsgsSequenceLaneletSegment = dds_sequence_autoware_planning_msgs_msg_LaneletSegment;
using awfPlanningMsgsSequenceLaneletPrimitive = dds_sequence_autoware_planning_msgs_msg_LaneletPrimitive;
using visualizationMsgsSequenceUVCoordinate = dds_sequence_visualization_msgs_msg_UVCoordinate;
using visualizationMsgsSequenceMarker = dds_sequence_visualization_msgs_msg_Marker;
using tier4DebugMsgsSequenceMultiArrayDimension = dds_sequence_tier4_debug_msgs_msg_MultiArrayDimension;
using awfPlanningMsgsSequencePathPoint = dds_sequence_autoware_planning_msgs_msg_PathPoint;
using tier4PlanningMsgsSequencePathPointWithLaneId = dds_sequence_tier4_planning_msgs_msg_PathPointWithLaneId;
using geometryMsgsSequencePoint32 = dds_sequence_geometry_msgs_msg_Point32;
using geometryMsgsSequencePose = dds_sequence_geometry_msgs_msg_Pose;
using awfPerceptionMsgsSequencePredictedPath = dds_sequence_autoware_perception_msgs_msg_PredictedPath;
using awfPerceptionMsgsSequencePredictedObject = dds_sequence_autoware_perception_msgs_msg_PredictedObject;
using tier4DebugMsgsSequenceProcessingTimeNode = dds_sequence_tier4_debug_msgs_msg_ProcessingTimeNode;
using tier4DebugMsgsSequenceSystemUsage = dds_sequence_tier4_debug_msgs_msg_SystemUsage;
using awfPerceptionMsgsSequenceTrackedObject = dds_sequence_autoware_perception_msgs_msg_TrackedObject;
using awfPerceptionMsgsSequenceTrafficLightElement = dds_sequence_autoware_perception_msgs_msg_TrafficLightElement;
using awfPerceptionMsgsSequenceTrafficLightGroup = dds_sequence_autoware_perception_msgs_msg_TrafficLightGroup;
using awfPerceptionMsgsSequenceTrafficSignalElement = dds_sequence_autoware_perception_msgs_msg_TrafficSignalElement;
using awfPerceptionMsgsSequenceTrafficSignal = dds_sequence_autoware_perception_msgs_msg_TrafficSignal;

// SIMPLIFIED
// Simplified message names
using PointMsg = geometryMsgsPoint;
using PoseMsg = geometryMsgsPose;
using Vector3Msg = geometryMsgsVector3;
using PoseStampedMsg = geometryMsgsPoseStamped;
using PoseWithCovarianceStampedMsg = geometryMsgsPoseWithCovarianceStamped;
using QuaternionMsg = geometryMsgsQuaternion;
using TransformMsg = geometryMsgsTransform;
using TransformStampedMsg = geometryMsgsTransformStamped;
using TwistMsg = geometryMsgsTwist;
using TwistStampedMsg = geometryMsgsTwistStamped;
using OdometryMsg = navMsgsOdometry;
using MarkerArrayMsg = visualizationMsgsMarkerArray;
using TrajectoryMsg = awfPlanningMsgsTrajectory;
using TrajectoryPointMsg = awfPlanningMsgsTrajectoryPoint;
using PathPointMsg = awfPlanningMsgsPathPoint;
using PathWithLaneIdMsg = awfPlanningMsgsPathWithLaneId;
using PathPointWithLaneIdMsg = tier4PlanningMsgsPathPointWithLaneId;
using LateralMsg = awfControlMsgsLateral;
using LongitudinalMsg = awfControlMsgsLongitudinal;
using AccelWithCovarianceStampedMsg = geometryMsgsAccelWithCovarianceStamped;
using SteeringReportMsg = awfVehicleMsgsSteeringReport;
using OperationModeStateMsg = awfAdapiV1MsgsOperationModeState;
using EngageMsg = awfVehicleMsgsEngage;

// Simplified sequence types
using FloatSeq = Sequence<dds_sequence_float>;
using DoubleSeq = Sequence<dds_sequence_double>;
using UInt8Seq = Sequence<dds_sequence_uint8>;
using Int32Seq = Sequence<dds_sequence_int32>;
using Int64Seq = Sequence<dds_sequence_int64>;
using PointSeq = Sequence<dds_sequence_geometry_msgs_msg_Point>;
using ColorRGBASeq = Sequence<dds_sequence_std_msgs_msg_ColorRGBA>;
using ObjectClassificationSeq = Sequence<awfPerceptionMsgsSequenceObjectClassification>;
using DetectedObjectSeq = Sequence<awfPerceptionMsgsSequenceDetectedObject>;
using TrajectoryPointSeq = Sequence<awfPlanningMsgsSequenceTrajectoryPoint>;
using LaneletSegmentSeq = Sequence<awfPlanningMsgsSequenceLaneletSegment>;
using LaneletPrimitiveSeq = Sequence<awfPlanningMsgsSequenceLaneletPrimitive>;
using UVCoordinateSeq = Sequence<visualizationMsgsSequenceUVCoordinate>;
using MarkerSeq = Sequence<visualizationMsgsSequenceMarker>;
using MultiArrayDimensionSeq = Sequence<tier4DebugMsgsSequenceMultiArrayDimension>;
using PathPointSeq = Sequence<awfPlanningMsgsSequencePathPoint>;
using PathPointWithLaneIdSeq = Sequence<tier4PlanningMsgsSequencePathPointWithLaneId>;
using Point32Seq = Sequence<geometryMsgsSequencePoint32>;
using PoseSeq = Sequence<geometryMsgsSequencePose>;
using PredictedPathSeq = Sequence<awfPerceptionMsgsSequencePredictedPath>;
using PredictedObjectSeq = Sequence<awfPerceptionMsgsSequencePredictedObject>;
using ProcessingTimeNodeSeq = Sequence<tier4DebugMsgsSequenceProcessingTimeNode>;
using SystemUsageSeq = Sequence<tier4DebugMsgsSequenceSystemUsage>;
using TrackedObjectSeq = Sequence<awfPerceptionMsgsSequenceTrackedObject>;
using TrafficLightElementSeq = Sequence<awfPerceptionMsgsSequenceTrafficLightElement>;
using TrafficLightGroupSeq = Sequence<awfPerceptionMsgsSequenceTrafficLightGroup>;
using TrafficSignalElementSeq = Sequence<awfPerceptionMsgsSequenceTrafficSignalElement>;
using TrafficSignalSeq = Sequence<awfPerceptionMsgsSequenceTrafficSignal>;

#endif  // ACTUATION_AUTOWARE__MESSAGES_H_