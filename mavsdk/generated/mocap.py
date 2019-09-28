# -*- coding: utf-8 -*-
from .._base import AsyncBase
from ..generated import mocap_pb2, mocap_pb2_grpc
from enum import Enum


class VisionPositionEstimate:
    """
     Global position/attitude estimate from a vision source.

     Parameters
     ----------
     time_usec : uint64_t
          Position frame timestamp UNIX Epoch time (0 to use Backend timestamp)

     position : Position
          Global position (m)

     roll : float
          Roll angle (rad)

     pitch : float
          Pitch angle (rad)

     yaw : float
          Yaw angle (rad)

     covariance : Covariance
          Pose cross-covariance matrix.

     """

    

    def __init__(
            self,
            time_usec,
            position,
            roll,
            pitch,
            yaw,
            covariance):
        """ Initializes the VisionPositionEstimate object """
        self.time_usec = time_usec
        self.position = position
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.covariance = covariance

    def __equals__(self, to_compare):
        """ Checks if two VisionPositionEstimate are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # VisionPositionEstimate object
            return \
                (self.time_usec == to_compare.time_usec) and \
                (self.position == to_compare.position) and \
                (self.roll == to_compare.roll) and \
                (self.pitch == to_compare.pitch) and \
                (self.yaw == to_compare.yaw) and \
                (self.covariance == to_compare.covariance)

        except AttributeError:
            return False

    def __str__(self):
        """ VisionPositionEstimate in string representation """
        struct_repr = ", ".join([
                "time_usec: " + str(self.time_usec),
                "position: " + str(self.position),
                "roll: " + str(self.roll),
                "pitch: " + str(self.pitch),
                "yaw: " + str(self.yaw),
                "covariance: " + str(self.covariance)
                ])

        return f"VisionPositionEstimate: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcVisionPositionEstimate):
        """ Translates a gRPC struct to the SDK equivalent """
        return VisionPositionEstimate(
                
                rpcVisionPositionEstimate.time_usec,
                
                
                Position.translate_from_rpc(rpcVisionPositionEstimate.position),
                
                
                rpcVisionPositionEstimate.roll,
                
                
                rpcVisionPositionEstimate.pitch,
                
                
                rpcVisionPositionEstimate.yaw,
                
                
                Covariance.translate_from_rpc(rpcVisionPositionEstimate.covariance)
                )

    def translate_to_rpc(self, rpcVisionPositionEstimate):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcVisionPositionEstimate.time_usec = self.time_usec
            
        
        
        
            
        self.position.translate_to_rpc(rpcVisionPositionEstimate.position)
            
        
        
        
            
        rpcVisionPositionEstimate.roll = self.roll
            
        
        
        
            
        rpcVisionPositionEstimate.pitch = self.pitch
            
        
        
        
            
        rpcVisionPositionEstimate.yaw = self.yaw
            
        
        
        
            
        self.covariance.translate_to_rpc(rpcVisionPositionEstimate.covariance)
            
        
        


class AttitudePositionMocap:
    """
     Motion capture attitude and position

     Parameters
     ----------
     time_usec : uint64_t
          Position frame timestamp UNIX Epoch time (0 to use Backend timestamp)

     q : Quaternion
          Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)

     position : Position
          Position (NED)

     covariance : Covariance
          Pose cross-covariance matrix.

     """

    

    def __init__(
            self,
            time_usec,
            q,
            position,
            covariance):
        """ Initializes the AttitudePositionMocap object """
        self.time_usec = time_usec
        self.q = q
        self.position = position
        self.covariance = covariance

    def __equals__(self, to_compare):
        """ Checks if two AttitudePositionMocap are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # AttitudePositionMocap object
            return \
                (self.time_usec == to_compare.time_usec) and \
                (self.q == to_compare.q) and \
                (self.position == to_compare.position) and \
                (self.covariance == to_compare.covariance)

        except AttributeError:
            return False

    def __str__(self):
        """ AttitudePositionMocap in string representation """
        struct_repr = ", ".join([
                "time_usec: " + str(self.time_usec),
                "q: " + str(self.q),
                "position: " + str(self.position),
                "covariance: " + str(self.covariance)
                ])

        return f"AttitudePositionMocap: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcAttitudePositionMocap):
        """ Translates a gRPC struct to the SDK equivalent """
        return AttitudePositionMocap(
                
                rpcAttitudePositionMocap.time_usec,
                
                
                Quaternion.translate_from_rpc(rpcAttitudePositionMocap.q),
                
                
                Position.translate_from_rpc(rpcAttitudePositionMocap.position),
                
                
                Covariance.translate_from_rpc(rpcAttitudePositionMocap.covariance)
                )

    def translate_to_rpc(self, rpcAttitudePositionMocap):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcAttitudePositionMocap.time_usec = self.time_usec
            
        
        
        
            
        self.q.translate_to_rpc(rpcAttitudePositionMocap.q)
            
        
        
        
            
        self.position.translate_to_rpc(rpcAttitudePositionMocap.position)
            
        
        
        
            
        self.covariance.translate_to_rpc(rpcAttitudePositionMocap.covariance)
            
        
        


class Odometry:
    """
     Odometry message to communicate odometry information with an external interface.

     Parameters
     ----------
     time_usec : uint64_t
          Timestamp (0 to use Backend timestamp).

     frame_id : MavFrame
          Coordinate frame of reference for the pose data.

     child_frame_id : MavFrame
          Coordinate frame of reference for the velocity in free space (twist) data.

     position : Position
          Position.

     q : Quaternion
          Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation).

     vx : float
          X linear speed (m/s).

     vy : float
          Y linear speed (m/s).

     vz : float
          Z linear speed (m/s).

     rollspeed : float
          Roll angular speed (rad/s).

     pitchspeed : float
          Pitch angular speed (rad/s).

     yawspeed : float
          Yaw angular speed (rad/s).

     pose_covariance : Covariance
          Pose cross-covariance matrix.

     velocity_covariance : Covariance
          Velocity cross-covariance matrix.

     estimator_type : MavEstimatorType
          Type of estimator that is providing the odometry (not used at 2019-09).

     """

    
    
    class MavFrame(Enum):
        """
         Mavlink frame id

         Values
         ------
         MAV_FRAME_GLOBAL
              Global (WGS84) coordinate frame + MSL altitude.

         MAV_FRAME_LOCAL_NED
              Local coordinate frame, Z-down (x: north, y: east, z: down).

         MAV_FRAME_MISSION
              NOT a coordinate frame, indicates a mission command.

         MAV_FRAME_GLOBAL_RELATIVE_ALT
              Global (WGS84) coordinate frame + altitude relative to the home position.

         MAV_FRAME_GLOBAL_INT
              Global (WGS84) coordinate frame (scaled) + MSL altitude.

         MAV_FRAME_GLOBAL_RELATIVE_ALT_INT
              Global (WGS84) coordinate frame (scaled) + altitude relative to the home position.

         MAV_FRAME_LOCAL_OFFSET_NED
              Offset to the current local frame. Anything expressed in this frame should be added to the current local frame position.

         MAV_FRAME_BODY_FRD
              Body fixed frame of reference, Z-down (x: forward, y: right, z: down). Replacement for MAV_FRAME_BODY_NED, MAV_FRAME_BODY_OFFSET_NED.

         MAV_FRAME_BODY_FLU
              Body fixed frame of reference, Z-up (x: forward, y: left, z: up).

         MAV_FRAME_LOCAL_FRD
              Forward, Right, Down coordinate frame. This is a local frame with Z-down and arbitrary F/R alignment (i.e. not aligned with NED/earth frame). Replacement for MAV_FRAME_MOCAP_NED, MAV_FRAME_VISION_NED, MAV_FRAME_ESTIM_NED.

         MAV_FRAME_LOCAL_FLU
              Forward, Left, Up coordinate frame. This is a local frame with Z-up and arbitrary F/L alignment (i.e. not aligned with ENU/earth frame). Replacement for MAV_FRAME_MOCAP_ENU, MAV_FRAME_VISION_ENU, MAV_FRAME_ESTIM_ENU.

         """

        
        MAV_FRAME_GLOBAL = 0
        MAV_FRAME_LOCAL_NED = 1
        MAV_FRAME_MISSION = 2
        MAV_FRAME_GLOBAL_RELATIVE_ALT = 3
        MAV_FRAME_GLOBAL_INT = 5
        MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
        MAV_FRAME_LOCAL_OFFSET_NED = 7
        MAV_FRAME_BODY_FRD = 12
        MAV_FRAME_BODY_FLU = 13
        MAV_FRAME_LOCAL_FRD = 20
        MAV_FRAME_LOCAL_FLU = 21

        def translate_to_rpc(self, rpcMavFrame):
            return {
                    0: mocap_pb2.Odometry.MAV_FRAME_GLOBAL,
                    1: mocap_pb2.Odometry.MAV_FRAME_LOCAL_NED,
                    2: mocap_pb2.Odometry.MAV_FRAME_MISSION,
                    3: mocap_pb2.Odometry.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    5: mocap_pb2.Odometry.MAV_FRAME_GLOBAL_INT,
                    6: mocap_pb2.Odometry.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    7: mocap_pb2.Odometry.MAV_FRAME_LOCAL_OFFSET_NED,
                    12: mocap_pb2.Odometry.MAV_FRAME_BODY_FRD,
                    13: mocap_pb2.Odometry.MAV_FRAME_BODY_FLU,
                    20: mocap_pb2.Odometry.MAV_FRAME_LOCAL_FRD,
                    21: mocap_pb2.Odometry.MAV_FRAME_LOCAL_FLU
                }.get(self.value, None)

        @staticmethod
        def translate_from_rpc(rpc_enum_value):
            """ Parses a gRPC response """
            return {
                    0: Odometry.MavFrame.MAV_FRAME_GLOBAL,
                    1: Odometry.MavFrame.MAV_FRAME_LOCAL_NED,
                    2: Odometry.MavFrame.MAV_FRAME_MISSION,
                    3: Odometry.MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    5: Odometry.MavFrame.MAV_FRAME_GLOBAL_INT,
                    6: Odometry.MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    7: Odometry.MavFrame.MAV_FRAME_LOCAL_OFFSET_NED,
                    12: Odometry.MavFrame.MAV_FRAME_BODY_FRD,
                    13: Odometry.MavFrame.MAV_FRAME_BODY_FLU,
                    20: Odometry.MavFrame.MAV_FRAME_LOCAL_FRD,
                    21: Odometry.MavFrame.MAV_FRAME_LOCAL_FLU,
                }.get(rpc_enum_value, None)

        def __str__(self):
            return self.name


    class MavEstimatorType(Enum):
        """
         Enumeration of estimator types

         Values
         ------
         MAV_ESTIMATOR_TYPE_UNKNOWN
              Unknown type of the estimator.

         MAV_ESTIMATOR_TYPE_NAIVE
              This is a naive estimator without any real covariance feedback.

         MAV_ESTIMATOR_TYPE_VISION
              Computer vision based estimate. Might be up to scale.

         MAV_ESTIMATOR_TYPE_VIO
              Visual-inertial estimate.

         MAV_ESTIMATOR_TYPE_GPS
              Plain GPS estimate.

         MAV_ESTIMATOR_TYPE_GPS_INS
              Estimator integrating GPS and inertial sensing.

         MAV_ESTIMATOR_TYPE_MOCAP
              Estimate from external motion capturing system.

         MAV_ESTIMATOR_TYPE_LIDAR
              Estimator based on lidar sensor input.

         MAV_ESTIMATOR_TYPE_AUTOPILOT
              Estimator on autopilot.

         """

        MAV_ESTIMATOR_TYPE_UNKNOWN = 0
        MAV_ESTIMATOR_TYPE_NAIVE = 1
        MAV_ESTIMATOR_TYPE_VISION = 2
        MAV_ESTIMATOR_TYPE_VIO = 3
        MAV_ESTIMATOR_TYPE_GPS = 4
        MAV_ESTIMATOR_TYPE_GPS_INS = 5
        MAV_ESTIMATOR_TYPE_MOCAP = 6
        MAV_ESTIMATOR_TYPE_LIDAR = 7
        MAV_ESTIMATOR_TYPE_AUTOPILOT = 8

        def translate_to_rpc(self, rpcMavEstimatorType):
            return {
                0: mocap_pb2.Odometry.MAV_ESTIMATOR_TYPE_UNKNOWN,
                1: mocap_pb2.Odometry.MAV_ESTIMATOR_TYPE_NAIVE,
                2: mocap_pb2.Odometry.MAV_ESTIMATOR_TYPE_VISION,
                3: mocap_pb2.Odometry.MAV_ESTIMATOR_TYPE_VIO,
                4: mocap_pb2.Odometry.MAV_ESTIMATOR_TYPE_GPS,
                5: mocap_pb2.Odometry.MAV_ESTIMATOR_TYPE_GPS_INS,
                6: mocap_pb2.Odometry.MAV_ESTIMATOR_TYPE_MOCAP,
                7: mocap_pb2.Odometry.MAV_ESTIMATOR_TYPE_LIDAR,
                8: mocap_pb2.Odometry.MAV_ESTIMATOR_TYPE_AUTOPILOT
            }.get(self.value, None)

        @staticmethod
        def translate_from_rpc(rpc_enum_value):
            """ Parses a gRPC response """
            return {
                0: Odometry.MavEstimatorType.MAV_ESTIMATOR_TYPE_UNKNOWN,
                1: Odometry.MavEstimatorType.MAV_ESTIMATOR_TYPE_NAIVE,
                2: Odometry.MavEstimatorType.MAV_ESTIMATOR_TYPE_VISION,
                3: Odometry.MavEstimatorType.MAV_ESTIMATOR_TYPE_VIO,
                4: Odometry.MavEstimatorType.MAV_ESTIMATOR_TYPE_GPS,
                5: Odometry.MavEstimatorType.MAV_ESTIMATOR_TYPE_GPS_INS,
                6: Odometry.MavEstimatorType.MAV_ESTIMATOR_TYPE_MOCAP,
                7: Odometry.MavEstimatorType.MAV_ESTIMATOR_TYPE_LIDAR,
                8: Odometry.MavEstimatorType.MAV_ESTIMATOR_TYPE_AUTOPILOT,
            }.get(rpc_enum_value, None)

        def __str__(self):
            return self.name


    def __init__(
            self,
            time_usec,
            frame_id,
            child_frame_id,
            position,
            q,
            vx,
            vy,
            vz,
            rollspeed,
            pitchspeed,
            yawspeed,
            pose_covariance,
            velocity_covariance,
            estimator_type):
        """ Initializes the Odometry object """
        self.time_usec = time_usec
        self.frame_id = frame_id
        self.child_frame_id = child_frame_id
        self.position = position
        self.q = q
        self.vx = vx
        self.vy = vy
        self.vz = vz
        self.rollspeed = rollspeed
        self.pitchspeed = pitchspeed
        self.yawspeed = yawspeed
        self.pose_covariance = pose_covariance
        self.velocity_covariance = velocity_covariance
        self.estimator_type = estimator_type

    def __equals__(self, to_compare):
        """ Checks if two Odometry are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # Odometry object
            return \
                (self.time_usec == to_compare.time_usec) and \
                (self.frame_id == to_compare.frame_id) and \
                (self.child_frame_id == to_compare.child_frame_id) and \
                (self.position == to_compare.position) and \
                (self.q == to_compare.q) and \
                (self.vx == to_compare.vx) and \
                (self.vy == to_compare.vy) and \
                (self.vz == to_compare.vz) and \
                (self.rollspeed == to_compare.rollspeed) and \
                (self.pitchspeed == to_compare.pitchspeed) and \
                (self.yawspeed == to_compare.yawspeed) and \
                (self.pose_covariance == to_compare.pose_covariance) and \
                (self.velocity_covariance == to_compare.velocity_covariance) and \
                (self.estimator_type == to_compare.estimator_type)

        except AttributeError:
            return False

    def __str__(self):
        """ Odometry in string representation """
        struct_repr = ", ".join([
                "time_usec: " + str(self.time_usec),
                "frame_id: " + str(self.frame_id),
                "child_frame_id: " + str(self.child_frame_id),
                "position: " + str(self.position),
                "q: " + str(self.q),
                "vx: " + str(self.vx),
                "vy: " + str(self.vy),
                "vz: " + str(self.vz),
                "rollspeed: " + str(self.rollspeed),
                "pitchspeed: " + str(self.pitchspeed),
                "yawspeed: " + str(self.yawspeed),
                "pose_covariance: " + str(self.pose_covariance),
                "velocity_covariance: " + str(self.velocity_covariance),
                "estimator_type: " + str(self.estimator_type)
                ])

        return f"Odometry: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcOdometry):
        """ Translates a gRPC struct to the SDK equivalent """
        return Odometry(
                
                rpcOdometry.time_usec,
                
                
                Odometry.MavFrame.translate_from_rpc(rpcOdometry.frame_id),
                
                
                Odometry.MavFrame.translate_from_rpc(rpcOdometry.child_frame_id),
                
                
                Position.translate_from_rpc(rpcOdometry.position),
                
                
                Quaternion.translate_from_rpc(rpcOdometry.q),
                
                
                rpcOdometry.vx,
                
                
                rpcOdometry.vy,
                
                
                rpcOdometry.vz,
                
                
                rpcOdometry.rollspeed,
                
                
                rpcOdometry.pitchspeed,
                
                
                rpcOdometry.yawspeed,
                
                
                Covariance.translate_from_rpc(rpcOdometry.pose_covariance),
                
                
                Covariance.translate_from_rpc(rpcOdometry.velocity_covariance),
                
                
                Odometry.MavEstimatorType.translate_from_rpc(rpcOdometry.estimator_type)
                )

    def translate_to_rpc(self, rpcOdometry):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcOdometry.time_usec = self.time_usec
            
        
        
        
            
        self.frame_id.translate_to_rpc(rpcOdometry.frame_id)
            
        
        
        
            
        self.child_frame_id.translate_to_rpc(rpcOdometry.child_frame_id)
            
        
        
        
            
        self.position.translate_to_rpc(rpcOdometry.position)
            
        
        
        
            
        self.q.translate_to_rpc(rpcOdometry.q)
            
        
        
        
            
        rpcOdometry.vx = self.vx
            
        
        
        
            
        rpcOdometry.vy = self.vy
            
        
        
        
            
        rpcOdometry.vz = self.vz
            
        
        
        
            
        rpcOdometry.rollspeed = self.rollspeed
            
        
        
        
            
        rpcOdometry.pitchspeed = self.pitchspeed
            
        
        
        
            
        rpcOdometry.yawspeed = self.yawspeed
            
        
        
        
            
        self.pose_covariance.translate_to_rpc(rpcOdometry.pose_covariance)
            
        
        
        
            
        self.velocity_covariance.translate_to_rpc(rpcOdometry.velocity_covariance)
            
        
        
        
            
        self.estimator_type.translate_to_rpc(rpcOdometry.estimator_type)
            
        
        


class Position:
    """
     Position type

     Parameters
     ----------
     x : float
          X position.

     y : float
          Y position.

     z : float
          Z position.

     """

    

    def __init__(
            self,
            x,
            y,
            z):
        """ Initializes the Position object """
        self.x = x
        self.y = y
        self.z = z

    def __equals__(self, to_compare):
        """ Checks if two Position are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # Position object
            return \
                (self.x == to_compare.x) and \
                (self.y == to_compare.y) and \
                (self.z == to_compare.z)

        except AttributeError:
            return False

    def __str__(self):
        """ Position in string representation """
        struct_repr = ", ".join([
                "x: " + str(self.x),
                "y: " + str(self.y),
                "z: " + str(self.z)
                ])

        return f"Position: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcPosition):
        """ Translates a gRPC struct to the SDK equivalent """
        return Position(
                
                rpcPosition.x,
                
                
                rpcPosition.y,
                
                
                rpcPosition.z
                )

    def translate_to_rpc(self, rpcPosition):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcPosition.x = self.x
            
        
        
        
            
        rpcPosition.y = self.y
            
        
        
        
            
        rpcPosition.z = self.z
            
        
        


class Covariance:
    """
     Covariance type.
     Row-major representation of a 6x6 cross-covariance matrix
     upper right triangle.
     Set first to NaN if unknown.

     Parameters
     ----------
     c : [float]
         
     """

    

    def __init__(
            self,
            c):
        """ Initializes the Covariance object """
        self.c = c

    def __equals__(self, to_compare):
        """ Checks if two Covariance are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # Covariance object
            return \
                (self.c == to_compare.c)

        except AttributeError:
            return False

    def __str__(self):
        """ Covariance in string representation """
        struct_repr = ", ".join([
                "c: " + str(self.c)
                ])

        return f"Covariance: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcCovariance):
        """ Translates a gRPC struct to the SDK equivalent """
        return Covariance(
                
                rpcCovariance.c
                )

    def translate_to_rpc(self, rpcCovariance):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        for elem in self.c:
          rpcCovariance.c.append(elem)
            
        
        


class Quaternion:
    """
     Quaternion type.

     All rotations and axis systems follow the right-hand rule.
     The Hamilton quaternion product definition is used.
     A zero-rotation quaternion is represented by (1,0,0,0).
     The quaternion could also be written as w + xi + yj + zk.

     For more info see: https://en.wikipedia.org/wiki/Quaternion

     Parameters
     ----------
     w : float
          Quaternion entry 0, also denoted as a

     x : float
          Quaternion entry 1, also denoted as b

     y : float
          Quaternion entry 2, also denoted as c

     z : float
          Quaternion entry 3, also denoted as d

     """

    

    def __init__(
            self,
            w,
            x,
            y,
            z):
        """ Initializes the Quaternion object """
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def __equals__(self, to_compare):
        """ Checks if two Quaternion are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # Quaternion object
            return \
                (self.w == to_compare.w) and \
                (self.x == to_compare.x) and \
                (self.y == to_compare.y) and \
                (self.z == to_compare.z)

        except AttributeError:
            return False

    def __str__(self):
        """ Quaternion in string representation """
        struct_repr = ", ".join([
                "w: " + str(self.w),
                "x: " + str(self.x),
                "y: " + str(self.y),
                "z: " + str(self.z)
                ])

        return f"Quaternion: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcQuaternion):
        """ Translates a gRPC struct to the SDK equivalent """
        return Quaternion(
                
                rpcQuaternion.w,
                
                
                rpcQuaternion.x,
                
                
                rpcQuaternion.y,
                
                
                rpcQuaternion.z
                )

    def translate_to_rpc(self, rpcQuaternion):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        rpcQuaternion.w = self.w
            
        
        
        
            
        rpcQuaternion.x = self.x
            
        
        
        
            
        rpcQuaternion.y = self.y
            
        
        
        
            
        rpcQuaternion.z = self.z
            
        
        


class MocapResult:
    """
     Result type.

     Parameters
     ----------
     result : Result
          Result enum value

     result_str : std::string
          Human-readable English string describing the result

     """

    
    
    class Result(Enum):
        """
         Possible results returned for mocap requests

         Values
         ------
         UNKNOWN
              Unknown error

         SUCCESS
              Request succeeded

         NO_SYSTEM
              No system is connected

         CONNECTION_ERROR
              Connection error

         INVALID_REQUEST_DATA
              Invalid request data

         """

        
        UNKNOWN = 0
        SUCCESS = 1
        NO_SYSTEM = 2
        CONNECTION_ERROR = 3
        INVALID_REQUEST_DATA = 4

        def translate_to_rpc(self, rpcResult):
            return {
                    0: mocap_pb2.MocapResult.UNKNOWN,
                    1: mocap_pb2.MocapResult.SUCCESS,
                    2: mocap_pb2.MocapResult.NO_SYSTEM,
                    3: mocap_pb2.MocapResult.CONNECTION_ERROR,
                    4: mocap_pb2.MocapResult.INVALID_REQUEST_DATA
                }.get(self.value, None)

        @staticmethod
        def translate_from_rpc(rpc_enum_value):
            """ Parses a gRPC response """
            return {
                    0: MocapResult.Result.UNKNOWN,
                    1: MocapResult.Result.SUCCESS,
                    2: MocapResult.Result.NO_SYSTEM,
                    3: MocapResult.Result.CONNECTION_ERROR,
                    4: MocapResult.Result.INVALID_REQUEST_DATA,
                }.get(rpc_enum_value, None)

        def __str__(self):
            return self.name
    

    def __init__(
            self,
            result,
            result_str):
        """ Initializes the MocapResult object """
        self.result = result
        self.result_str = result_str

    def __equals__(self, to_compare):
        """ Checks if two MocapResult are the same """
        try:
            # Try to compare - this likely fails when it is compared to a non
            # MocapResult object
            return \
                (self.result == to_compare.result) and \
                (self.result_str == to_compare.result_str)

        except AttributeError:
            return False

    def __str__(self):
        """ MocapResult in string representation """
        struct_repr = ", ".join([
                "result: " + str(self.result),
                "result_str: " + str(self.result_str)
                ])

        return f"MocapResult: [{struct_repr}]"

    @staticmethod
    def translate_from_rpc(rpcMocapResult):
        """ Translates a gRPC struct to the SDK equivalent """
        return MocapResult(
                
                MocapResult.Result.translate_from_rpc(rpcMocapResult.result),
                
                
                rpcMocapResult.result_str
                )

    def translate_to_rpc(self, rpcMocapResult):
        """ Translates this SDK object into its gRPC equivalent """

        
        
            
        self.result.translate_to_rpc(rpcMocapResult.result)
            
        
        
        
            
        rpcMocapResult.result_str = self.result_str
            
        
        



class MocapError(Exception):
    """ Raised when a MocapResult is a fail code """

    def __init__(self, result, origin, *params):
        self._result = result
        self._origin = origin
        self._params = params

    def __str__(self):
        return f"{self._result.result}: '{self._result.result_str}'; origin: {self._origin}; params: {self._params}"


class Mocap(AsyncBase):
    """
     *
     Motion Capture allow vehicles to navigate when a global
     position source is unavailable or unreliable
     (e.g. indoors, or when flying under a bridge. etc.).

     Generated by dcsdkgen - MAVSDK Mocap API
    """

    # Plugin name
    name = "Mocap"

    def _setup_stub(self, channel):
        """ Setups the api stub """
        self._stub = mocap_pb2_grpc.MocapServiceStub(channel)

    
    def _extract_result(self, response):
        """ Returns the response status and description """
        return MocapResult.translate_from_rpc(response.mocap_result)
    

    async def set_vision_position_estimate(self, vision_position_estimate):
        """
         Send Global position/attitude estimate from a vision source.

         Parameters
         ----------
         vision_position_estimate : VisionPositionEstimate
             
         Raises
         ------
         MocapError
             If the request fails. The error contains the reason for the failure.
        """

        request = mocap_pb2.SetVisionPositionEstimateRequest()
        
        vision_position_estimate.translate_to_rpc(request.vision_position_estimate)
                
            
        response = await self._stub.SetVisionPositionEstimate(request)

        
        result = self._extract_result(response)

        if result.result is not MocapResult.Result.SUCCESS:
            raise MocapError(result, "set_vision_position_estimate()", vision_position_estimate)
        

    async def set_attitude_position_mocap(self, attitude_position_mocap):
        """
         Send motion capture attitude and position.

         Parameters
         ----------
         attitude_position_mocap : AttitudePositionMocap
             
         Raises
         ------
         MocapError
             If the request fails. The error contains the reason for the failure.
        """

        request = mocap_pb2.SetAttitudePositionMocapRequest()
        
        attitude_position_mocap.translate_to_rpc(request.attitude_position_mocap)
                
            
        response = await self._stub.SetAttitudePositionMocap(request)

        
        result = self._extract_result(response)

        if result.result is not MocapResult.Result.SUCCESS:
            raise MocapError(result, "set_attitude_position_mocap()", attitude_position_mocap)
        

    async def set_odometry(self, odometry):
        """
         Send odometry information with an external interface.

         Parameters
         ----------
         odometry : Odometry
             
         Raises
         ------
         MocapError
             If the request fails. The error contains the reason for the failure.
        """

        request = mocap_pb2.SetOdometryRequest()
        
        odometry.translate_to_rpc(request.odometry)
                
            
        response = await self._stub.SetOdometry(request)

        
        result = self._extract_result(response)

        if result.result is not MocapResult.Result.SUCCESS:
            raise MocapError(result, "set_odometry()", odometry)
        