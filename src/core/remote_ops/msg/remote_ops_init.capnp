@0xda53f20f98951706;

struct RemoteOpsInit {
  metricsType @0 : List(UInt8);
  metricsValueType @1 : List(UInt8);
  alertType @2 : List(UInt32);
  alertThreshold @3 : List(Text);
  alertIds @4 : List(UInt32);
  alertDescriptions @5 : List(Text);
}
