@0xad2643663322a2e2;

struct RemoteOpsRequest {
  redAlerts @0 : List(UInt32);
  redActionableAlerts @1 : List(UInt32);
  amberAlerts @2 : List(UInt32);
  amberActionableAlerts @3 : List(UInt32);

  recommendedActions @4 : List(UInt8);
  restrictedActions @5 : List(UInt8);

  highRiskType @6 : List(UInt8);
  highRiskDist @7 : List(Float32);

  metricTypes @8 : List(UInt32);
  metricValues @9 : List(Text);

  jobDoneAllowed @10 : Bool;
}
