@0xaa5809a21d953489;

using PauseResume = import "pause_resume_cmd.capnp";
using SetDestination = import "set_destination_cmd.capnp";
using Relocalization = import "relocalization_cmd.capnp";
using TrafficLightOverride = import "traffic_light_override_cmd.capnp";
using NudgeForward = import "nudge_forward_cmd.capnp";
using AdjustPositionByDistance = import "adjust_position_by_distance_cmd.capnp";
using Horn = import "horn_cmd.capnp";
using HazardLight = import "hazard_light_cmd.capnp";
using EmergencyBrake = import "emergency_brake_cmd.capnp";
using TrajectoryOverride = import "trajectory_override_cmd.capnp";
using SpeedLimit = import "speed_limit_cmd.capnp";
using ManualReroute = import "manual_reroute_cmd.capnp";
using SetNonYardDestination = import "set_non_yard_destination_cmd.capnp";
using HeadLight = import "head_light_cmd.capnp";
using PrecedenceOverride = import "precedence_override_cmd.capnp";


struct RemoteOpsCommand {
  sender @0 :Text;
  receiver @1 :Text;
  command :union {
    pauseResumeCmd @2 :PauseResume.PauseResumeCmd;
    setDestinationCmd @3 :SetDestination.SetDestinationCmd;
    relocalizationCmd @4 :Relocalization.RelocalizationCmd;
    trafficLightOverrideCmd @5 :TrafficLightOverride.TrafficLightOverrideCmd;
    nudgeForwardCmd @6 :NudgeForward.NudgeForwardCmd;
    adjustPositionByDistanceCmd @7 :AdjustPositionByDistance.AdjustPositionByDistanceCmd;
    hornCmd @8 :Horn.HornCmd;
    hazardLightCmd @9 :HazardLight.HazardLightCmd;
    emergencyBrakeCmd @10 :EmergencyBrake.EmergencyBrakeCmd;
    trajectoryOverrideCmd @11 :TrajectoryOverride.TrajectoryOverrideCmd;
    speedLimitCmd @12 :SpeedLimit.SpeedLimitCmd;
    manualRerouteCmd @13 :ManualReroute.ManualRerouteCmd;
    setNonYardDestinationCmd @14 :SetNonYardDestination.SetNonYardDestinationCmd;
    headLightCmd @15 :HeadLight.HeadLightCmd;
    precedenceOverrideCmd @16 :PrecedenceOverride.PrecedenceOverrideCmd;
  }
}

