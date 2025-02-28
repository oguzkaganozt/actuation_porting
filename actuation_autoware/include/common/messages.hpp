#ifndef ACTUATION_AUTOWARE__MESSAGES_H_
#define ACTUATION_AUTOWARE__MESSAGES_H_

#include "common/sequence.hpp"

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

// Message types from autoware_msgs
using PointMsg = geometry_msgs_msg_Point;
using PoseMsg = geometry_msgs_msg_Pose;
using Vector3Msg = geometry_msgs_msg_Vector3;
using PoseStampedMsg = geometry_msgs_msg_PoseStamped;
using PoseWithCovarianceStampedMsg = geometry_msgs_msg_PoseWithCovarianceStamped;
using QuaternionMsg = geometry_msgs_msg_Quaternion;
using TransformMsg = geometry_msgs_msg_Transform;
using TransformStampedMsg = geometry_msgs_msg_TransformStamped;
using TwistMsg = geometry_msgs_msg_Twist;
using TwistStampedMsg = geometry_msgs_msg_TwistStamped;
using OdometryMsg = nav_msgs_msg_Odometry;
using MarkerArrayMsg = visualization_msgs_msg_MarkerArray;
using TrajectoryMsg = autoware_planning_msgs_msg_Trajectory;
using TrajectoryPointMsg = autoware_planning_msgs_msg_TrajectoryPoint;
using PathPointMsg = autoware_planning_msgs_msg_PathPoint;
using PathWithLaneIdMsg = tier4_planning_msgs_msg_PathWithLaneId;
using PathPointWithLaneIdMsg = tier4_planning_msgs_msg_PathPointWithLaneId;
using LateralMsg = autoware_control_msgs_msg_Lateral;
using LongitudinalMsg = autoware_control_msgs_msg_Longitudinal;
using AccelWithCovarianceStampedMsg = geometry_msgs_msg_AccelWithCovarianceStamped;
using SteeringReportMsg = autoware_vehicle_msgs_msg_SteeringReport;
using OperationModeStateMsg = autoware_adapi_v1_msgs_msg_OperationModeState;
using EngageMsg = autoware_vehicle_msgs_msg_Engage;

// Raw Sequence types from autoware_msgs
using FloatSeqRaw = dds_sequence_float;
using DoubleSeqRaw = dds_sequence_double;
using UInt8SeqRaw = dds_sequence_uint8;
using Int32SeqRaw = dds_sequence_int32;
using Int64SeqRaw = dds_sequence_int64;
using PointSeqRaw = dds_sequence_geometry_msgs_msg_Point;
using ColorRGBASeqRaw = dds_sequence_std_msgs_msg_ColorRGBA;
using TrajectoryPointSeqRaw = dds_sequence_autoware_planning_msgs_msg_TrajectoryPoint;
using ObjectClassificationSeqRaw = dds_sequence_autoware_perception_msgs_msg_ObjectClassification;
using DetectedObjectSeqRaw = dds_sequence_autoware_perception_msgs_msg_DetectedObject;
using LaneletSegmentSeqRaw = dds_sequence_autoware_planning_msgs_msg_LaneletSegment;
using LaneletPrimitiveSeqRaw = dds_sequence_autoware_planning_msgs_msg_LaneletPrimitive;
using UVCoordinateSeqRaw = dds_sequence_visualization_msgs_msg_UVCoordinate;
using MarkerSeqRaw = dds_sequence_visualization_msgs_msg_Marker;
using MultiArrayDimensionSeqRaw = dds_sequence_tier4_debug_msgs_msg_MultiArrayDimension;
using PathPointSeqRaw = dds_sequence_autoware_planning_msgs_msg_PathPoint;
using PathPointWithLaneIdSeqRaw = dds_sequence_tier4_planning_msgs_msg_PathPointWithLaneId;
using Point32SeqRaw = dds_sequence_geometry_msgs_msg_Point32;
using PoseSeqRaw = dds_sequence_geometry_msgs_msg_Pose;
using PredictedPathSeqRaw = dds_sequence_autoware_perception_msgs_msg_PredictedPath;
using PredictedObjectSeqRaw = dds_sequence_autoware_perception_msgs_msg_PredictedObject;
using ProcessingTimeNodeSeqRaw = dds_sequence_tier4_debug_msgs_msg_ProcessingTimeNode;
using SystemUsageSeqRaw = dds_sequence_tier4_debug_msgs_msg_SystemUsage;
using TrackedObjectSeqRaw = dds_sequence_autoware_perception_msgs_msg_TrackedObject;
using TrafficLightElementSeqRaw = dds_sequence_autoware_perception_msgs_msg_TrafficLightElement;
using TrafficLightGroupSeqRaw = dds_sequence_autoware_perception_msgs_msg_TrafficLightGroup;
using TrafficSignalElementSeqRaw = dds_sequence_autoware_perception_msgs_msg_TrafficSignalElement;
using TrafficSignalSeqRaw = dds_sequence_autoware_perception_msgs_msg_TrafficSignal;

// Wrapped sequence types
using FloatSeq = Sequence<FloatSeqRaw>;
using DoubleSeq = Sequence<DoubleSeqRaw>;
using UInt8Seq = Sequence<UInt8SeqRaw>;
using Int32Seq = Sequence<Int32SeqRaw>;
using Int64Seq = Sequence<Int64SeqRaw>;
using PointSeq = Sequence<PointSeqRaw>;
using ColorRGBASeq = Sequence<ColorRGBASeqRaw>;
using ObjectClassificationSeq = Sequence<ObjectClassificationSeqRaw>;
using DetectedObjectSeq = Sequence<DetectedObjectSeqRaw>;
using TrajectoryPointSeq = Sequence<TrajectoryPointSeqRaw>;
using LaneletSegmentSeq = Sequence<LaneletSegmentSeqRaw>;
using LaneletPrimitiveSeq = Sequence<LaneletPrimitiveSeqRaw>;
using UVCoordinateSeq = Sequence<UVCoordinateSeqRaw>;
using MarkerSeq = Sequence<MarkerSeqRaw>;
using MultiArrayDimensionSeq = Sequence<MultiArrayDimensionSeqRaw>;
using PathPointSeq = Sequence<PathPointSeqRaw>;
using PathPointWithLaneIdSeq = Sequence<PathPointWithLaneIdSeqRaw>;
using Point32Seq = Sequence<Point32SeqRaw>;
using PoseSeq = Sequence<PoseSeqRaw>;
using PredictedPathSeq = Sequence<PredictedPathSeqRaw>;
using PredictedObjectSeq = Sequence<PredictedObjectSeqRaw>;
using ProcessingTimeNodeSeq = Sequence<ProcessingTimeNodeSeqRaw>;
using SystemUsageSeq = Sequence<SystemUsageSeqRaw>;
using TrackedObjectSeq = Sequence<TrackedObjectSeqRaw>;
using TrafficLightElementSeq = Sequence<TrafficLightElementSeqRaw>;
using TrafficLightGroupSeq = Sequence<TrafficLightGroupSeqRaw>;
using TrafficSignalElementSeq = Sequence<TrafficSignalElementSeqRaw>;
using TrafficSignalSeq = Sequence<TrafficSignalSeqRaw>;

#endif  // ACTUATION_AUTOWARE__MESSAGES_H_